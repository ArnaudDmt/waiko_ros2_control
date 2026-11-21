#include "waiko_ros2_control/waiko_estimator.hpp"

#include <algorithm>

namespace waiko_ros2_control
{

namespace so = stateObservation;
using LoContact = so::odometry::LoContact;
using LeggedOdometryManager = so::odometry::LeggedOdometryManager;

WaikoEstimator::WaikoEstimator(
  const WaikoConfiguration & configuration, const Eigen::Vector3d & local_linear_velocity,
  const Eigen::Vector3d & tilt, const Eigen::Quaterniond & world_imu_orientation,
  const Eigen::Vector3d & world_imu_position)
: configuration_(configuration),
  observer_(
    configuration.initial_gains.alpha, configuration.initial_gains.beta,
    configuration.initial_gains.gamma, configuration.initial_gains.rho,
    configuration.initial_gains.mu, configuration.initial_gains.psi)
{
  observer_.setWithOriCorrectFromContactPos(configuration_.with_ori_correct_from_contact_pos);

  // The local position is the IMU position in the frame of the IMU orientation centred on the anchor,
  // which is the world origin at the initialization: p_local = R^T p.
  const Eigen::Matrix3d R = world_imu_orientation.toRotationMatrix();
  observer_.initEstimator(
    local_linear_velocity, tilt, Eigen::Vector4d(world_imu_orientation.coeffs()),
    R.transpose() * world_imu_position);

  so::kine::Kinematics world_imu_pose;
  world_imu_pose.position = world_imu_position;
  world_imu_pose.orientation = R;
  odometry_.kappa(configuration_.kappa);
  odometry_.lambdaInf(configuration_.lambda_inf);
  odometry_.init(
    LeggedOdometryManager::Configuration(so::odometry::OdometryType::Odometry6d)
      .correctContacts(configuration_.correct_contacts),
    world_imu_pose.toVector(so::kine::Kinematics::Flags::pose));

  update_estimate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void WaikoEstimator::update(
  double dt, const Eigen::Vector3d & accelerometer, const Eigen::Vector3d & gyrometer,
  const std::vector<ContactMeasurement> & contacts, bool reset_local_linear_velocity)
{
  time_ += dt;
  odometry_.setSamplingTime(dt);

  const bool contacts_changed = update_contacts(contacts);

  // Only the contacts set before this step have a world reference, so only they carry information.
  const bool has_support = !odometry_.maintainedContacts().empty();
  apply_gains(has_support);

  // Kinematic measurement of the local linear velocity: the anchor point (weighted mean of the
  // contacts) is assumed fixed in the world. Without support, the IMU is assumed static.
  Eigen::Vector3d kinematic_velocity = Eigen::Vector3d::Zero();
  if (has_support)
  {
    const so::kine::Kinematics imu_anchor = odometry_.getAnchorKineInBody(true);
    kinematic_velocity = -gyrometer.cross(imu_anchor.position()) - imu_anchor.linVel();
  }

  const so::TimeIndex k = observer_.getCurrentTime();
  observer_.setInput(
    dt, kinematic_velocity, accelerometer, gyrometer, k, reset_local_linear_velocity);
  if (has_support)
  {
    for (const LoContact * contact : odometry_.maintainedContacts())
    {
      observer_.addContactPosInput(
        contact->worldRefKine_.position(), contact->bodyContactKine_.position(), contact->lambda(), k);
    }
    // Orientation of the IMU given by the orientations of the contacts and their references.
    const so::kine::LocalKinematics world_imu_from_contacts =
      odometry_.getWorldBodyLocalKineFromAnchor();
    if (configuration_.with_ori_correct_from_contact_ori && world_imu_from_contacts.orientation.isSet())
    {
      observer_.addOriInput(world_imu_from_contacts.orientation.toMatrix3(), k);
    }
  }
  observer_.getEstimatedState(k + 1);

  // The leg odometry takes the new pose of the IMU: it corrects the references of the maintained
  // contacts and computes the references of the new ones.
  const Eigen::Vector3d world_imu_position = observer_.getEstimatedWorldPosition();
  const Eigen::Matrix3d world_imu_orientation = observer_.getEstimatedOrientation();
  so::kine::Kinematics world_imu_pose;
  odometry_.run(
    LeggedOdometryManager::KineParams(world_imu_pose)
      .attitudeMeasurement(world_imu_orientation)
      .positionMeas(world_imu_position));

  if (configuration_.reset_position_anchor_on_contact_change && contacts_changed)
  {
    reset_position_anchor_to_support();
  }

  update_estimate(gyrometer, kinematic_velocity);
}

bool WaikoEstimator::update_contacts(const std::vector<ContactMeasurement> & contacts)
{
  // Updated in place, so that it allocates only when the set of contacts changes.
  for (const ContactMeasurement & contact : contacts)
  {
    if (contact.active)
    {
      active_contacts_.insert(contact.name);
    }
    else
    {
      active_contacts_.erase(contact.name);
    }
  }

  // Called by the leg odometry on each contact of active_contacts_, new or maintained.
  auto set_measurement = [&contacts](LoContact & contact)
  {
    const auto measurement = std::find_if(
      contacts.begin(), contacts.end(),
      [&contact](const ContactMeasurement & m) { return m.name == contact.surfaceName(); });
    contact.bodyContactKine_.position = measurement->position;
    contact.bodyContactKine_.orientation = so::Matrix3(measurement->orientation.toRotationMatrix());
    contact.bodyContactKine_.linVel = measurement->linear_velocity;
    contact.lambda(measurement->lambda);
  };

  return odometry_.initLoop(
    active_contacts_,
    LeggedOdometryManager::ContactUpdateFunctions()
      .onNewContact(set_measurement)
      .onMaintainedContact(set_measurement),
    nullptr, nullptr);
}

void WaikoEstimator::apply_gains(bool has_support)
{
  const WaikoGains & gains = time_ > configuration_.gain_switch_time ? configuration_.final_gains
                                                                     : configuration_.initial_gains;
  observer_.setAlpha(has_support ? gains.alpha : configuration_.no_contact_alpha);
  observer_.setBeta(has_support ? gains.beta : configuration_.no_contact_beta);
  observer_.setGamma(gains.gamma);
  observer_.setRho(gains.rho);
  observer_.setMu(gains.mu);
  observer_.setPsi(gains.psi);
}

// World frame of the position and position anchor
// ------------------------------------------------
// WAIKO does not estimate the world position p of the IMU directly but a local position p_local,
// expressed in the IMU frame and measured from a point of the world called the position anchor a:
//
//     p = a + R p_local
//
// The orientation corrections rotate R, so their effect on p grows with the lever arm |p_local|.
// Keeping the anchor at the support keeps that lever arm short, i.e. orientation errors barely move
// the position. The reset is a change of origin only: a is moved to the new anchor and p_local is
// shifted by R^T (a_old - a_new), so p, R, x1 and x2 are unchanged (see
// WaikoHumanoid::resetPositionAnchor).
//
// It runs at the end of the step in which the contacts changed: the leg odometry has then given a
// world reference to every current contact, the new ones included. The anchor is the plain centroid
// of these references; their weights are not used because those of the new contacts are not yet
// normalized with the others. Without any contact the anchor is kept.
void WaikoEstimator::reset_position_anchor_to_support()
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const LoContact * contact : odometry_.maintainedContacts())
  {
    centroid += contact->worldRefKine_.position();
  }
  for (const LoContact * contact : odometry_.newContacts())
  {
    centroid += contact->worldRefKine_.position();
  }

  const std::size_t count = odometry_.maintainedContacts().size() + odometry_.newContacts().size();
  if (count == 0)
  {
    return;
  }
  observer_.resetPositionAnchor(centroid / static_cast<double>(count));
}

void WaikoEstimator::update_estimate(
  const Eigen::Vector3d & angular_velocity, const Eigen::Vector3d & kinematic_velocity)
{
  estimate_.local_linear_velocity = observer_.getEstimatedLocLinVel();
  estimate_.tilt = observer_.getEstimatedTilt();
  estimate_.orientation = Eigen::Quaterniond(observer_.getEstimatedOrientation());
  estimate_.position = observer_.getEstimatedWorldPosition();
  estimate_.position_anchor = observer_.getPositionAnchor();
  estimate_.angular_velocity = angular_velocity;
  estimate_.kinematic_velocity = kinematic_velocity;
  estimate_.orientation_correction_from_contact_orientations = observer_.getOriCorrFromOriMeas();
  estimate_.position_correction_from_contact_positions = observer_.getPosCorrectionFromContactPos();
  estimate_.maintained_contacts_count = odometry_.maintainedContacts().size();

  // The entries are overwritten in place, so that the names are not reallocated at each update.
  std::size_t count = 0;
  for (const auto & [name, contact] : odometry_.contactsManager().contacts())
  {
    if (!contact.isSet())
    {
      continue;
    }
    if (count == estimate_.contacts.size())
    {
      estimate_.contacts.emplace_back();
    }
    ContactState & state = estimate_.contacts[count++];
    state.name = name;
    state.lambda = contact.lambda();
    state.imu_position = contact.bodyContactKine_.position();
    state.imu_orientation = Eigen::Quaterniond(contact.bodyContactKine_.orientation.toMatrix3());
    state.world_ref_position = contact.worldRefKine_.position();
    state.world_ref_orientation = Eigen::Quaterniond(contact.worldRefKine_.orientation.toMatrix3());
  }
  estimate_.contacts.resize(count);
}

}  // namespace waiko_ros2_control
