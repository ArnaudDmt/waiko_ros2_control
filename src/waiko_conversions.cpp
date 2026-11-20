#include "waiko_ros2_control/waiko_conversions.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

namespace waiko_ros2_control
{

namespace
{

/// Gains from one of the gain groups of the parameters, which have the same fields.
template <typename GainParameters>
WaikoGains make_gains(const GainParameters & gains)
{
  return {gains.alpha, gains.beta, gains.gamma, gains.rho, gains.mu, gains.psi};
}

}  // namespace

WaikoConfiguration make_configuration(const waiko_estimator::Params & params)
{
  WaikoConfiguration configuration;
  configuration.initial_gains = make_gains(params.initial_gains);
  configuration.final_gains = make_gains(params.final_gains);
  configuration.gain_switch_time = params.gain_switch_time;
  configuration.no_contact_alpha = params.no_contact_gains.alpha;
  configuration.no_contact_beta = params.no_contact_gains.beta;
  configuration.with_ori_correct_from_contact_ori = params.with_ori_correct_from_contact_ori;
  configuration.with_ori_correct_from_contact_pos = params.with_ori_correct_from_contact_pos;
  configuration.correct_contacts = params.leg_odometry.correct_contacts;
  configuration.kappa = params.leg_odometry.kappa;
  configuration.lambda_inf = params.leg_odometry.lambda_inf;
  configuration.reset_position_anchor_on_contact_change =
    params.reset_position_anchor_on_contact_change;
  return configuration;
}

void to_message(const WaikoEstimate & estimate, msg::WaikoState & message)
{
  message.contacts_active = estimate.maintained_contacts_count > 0;
  message.active_contacts_count = static_cast<uint32_t>(estimate.maintained_contacts_count);
  tf2::toMsg(estimate.kinematic_velocity, message.computed_yv);
  tf2::toMsg(estimate.local_linear_velocity, message.x1);
  tf2::toMsg(estimate.tilt, message.x2);
  message.orientation = tf2::toMsg(estimate.orientation);
  message.position = tf2::toMsg(estimate.position);
  message.position_anchor = tf2::toMsg(estimate.position_anchor);
  tf2::toMsg(estimate.angular_velocity, message.angular_velocity);
  tf2::toMsg(
    estimate.orientation_correction_from_contact_orientations,
    message.orientation_correction_from_meas);
  tf2::toMsg(
    estimate.position_correction_from_contact_positions, message.position_correction_from_meas);

  message.contacts.resize(estimate.contacts.size());
  for (std::size_t i = 0; i < estimate.contacts.size(); ++i)
  {
    const ContactState & contact = estimate.contacts[i];
    msg::WaikoContactState & contact_message = message.contacts[i];
    contact_message.name = contact.name;
    contact_message.active = true;
    contact_message.contact_lambda = contact.lambda;
    contact_message.world_ref_position = tf2::toMsg(contact.world_ref_position);
    contact_message.world_ref_orientation = tf2::toMsg(contact.world_ref_orientation);
    contact_message.imu_contact_position = tf2::toMsg(contact.imu_position);
    contact_message.imu_contact_orientation = tf2::toMsg(contact.imu_orientation);
  }
}

}  // namespace waiko_ros2_control
