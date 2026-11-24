#ifndef WAIKO_ROS2_CONTROL__WAIKO_ESTIMATOR_HPP_
#define WAIKO_ROS2_CONTROL__WAIKO_ESTIMATOR_HPP_

#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <state-observation/observer/waiko-humanoid.hpp>
#include <state-observation/tools/odometry/legged-odometry-manager.hpp>

#include "waiko_ros2_control/msg/waiko_state.hpp"
#include "waiko_ros2_control/waiko_estimator_parameters.hpp"

namespace waiko_ros2_control
{

/// Measurement of one contact, given by the robot. Kinematics are expressed in the IMU frame.
struct ContactMeasurement
{
  std::string name;
  /// True when the contact is set with the environment.
  bool active;
  /// Weight of the contact in the leg odometry.
  double lambda;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_velocity;
};

/// Contact tracked by the leg odometry.
struct ContactState
{
  std::string name;
  double lambda;
  /// Pose of the contact in the IMU frame.
  Eigen::Vector3d imu_position;
  Eigen::Quaterniond imu_orientation;
  /// Reference pose of the contact in the world.
  Eigen::Vector3d world_ref_position;
  Eigen::Quaterniond world_ref_orientation;
};

/// Output of WaikoEstimator after the initialization or an update.
struct WaikoEstimate
{
  /// x1: linear velocity of the IMU in the world, expressed in the IMU frame.
  Eigen::Vector3d local_linear_velocity;
  /// x2: intermediate estimate of the tilt R^T e_z.
  Eigen::Vector3d tilt;
  /// Pose of the IMU in the world.
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  /// Origin of the frame of the local position: position = position_anchor + R p_local.
  Eigen::Vector3d position_anchor;
  /// Gyrometer measurement of the update.
  Eigen::Vector3d angular_velocity;
  /// Kinematic measurement yv of the local linear velocity.
  Eigen::Vector3d kinematic_velocity;
  /// Correction terms of the update, as a local angular velocity and a local linear velocity.
  Eigen::Vector3d orientation_correction_from_contact_orientations;
  Eigen::Vector3d position_correction_from_contact_positions;
  /// Number of contacts used for the correction: the ones set before this update.
  std::size_t maintained_contacts_count;
  std::vector<ContactState> contacts;
};

/// WAIKO observer fed with the IMU and the contact kinematics, as MCWaiko in mc_state_observation.
///
/// Each update:
///  1. passes the contact measurements to the leg odometry, which tracks the contacts by name;
///  2. builds the kinematic measurement of the local linear velocity, yv = -w x p_anchor - v_anchor,
///     from the contacts that were already set (the anchor point is their weighted mean);
///  3. runs one WAIKO step, corrected by the world references of those contacts;
///  4. gives the new pose of the IMU to the leg odometry, which updates the contact references;
///  5. optionally moves the position anchor to the support (see reset_position_anchor_to_support()).
class WaikoEstimator
{
public:
  /// Builds the observer at its initial state, with no contact set. The initial position anchor is the
  /// world origin.
  /// @param params Gains and options, see src/waiko_estimator_parameters.yaml.
  /// @param local_linear_velocity Initial x1.
  /// @param tilt Initial x2, usually R^T e_z.
  /// @param world_imu_orientation, world_imu_position Initial pose of the IMU in the world.
  WaikoEstimator(
    const waiko_estimator::Params & params, const Eigen::Vector3d & local_linear_velocity,
    const Eigen::Vector3d & tilt, const Eigen::Quaterniond & world_imu_orientation,
    const Eigen::Vector3d & world_imu_position);

  /// Runs one estimation step of duration dt (s).
  /// @param accelerometer, gyrometer IMU measurements.
  /// @param contacts Measurements of all the contacts of the robot, active or not.
  /// @param reset_local_linear_velocity Sets x1 to yv before the step.
  void update(
    double dt, const Eigen::Vector3d & accelerometer, const Eigen::Vector3d & gyrometer,
    const std::vector<ContactMeasurement> & contacts, bool reset_local_linear_velocity = false);

  const WaikoEstimate & estimate() const { return estimate_; }

private:
  /// Gives the measurements of the active contacts to the leg odometry.
  /// @return true if contacts were created or removed.
  bool update_contacts(const std::vector<ContactMeasurement> & contacts);

  /// Selects the gains for the current time and support.
  void apply_gains(bool has_support);

  /// Moves the position anchor to the centroid of the world references of the current contacts.
  void reset_position_anchor_to_support();

  /// Fills estimate_ from the state of the observer and of the leg odometry.
  void update_estimate(const Eigen::Vector3d & angular_velocity, const Eigen::Vector3d & kinematic_velocity);

  waiko_estimator::Params params_;
  stateObservation::WaikoHumanoid observer_;
  stateObservation::odometry::LeggedOdometryManager odometry_;
  /// Time since the construction, accumulated over the updates (s).
  double time_ = 0.0;
  std::set<std::string> active_contacts_;
  WaikoEstimate estimate_;
};

/// Fills every field of the state message but its header.
void to_message(const WaikoEstimate & estimate, msg::WaikoState & message);

}  // namespace waiko_ros2_control

#endif  // WAIKO_ROS2_CONTROL__WAIKO_ESTIMATOR_HPP_
