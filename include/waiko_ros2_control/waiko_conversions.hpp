#ifndef WAIKO_ROS2_CONTROL__WAIKO_CONVERSIONS_HPP_
#define WAIKO_ROS2_CONTROL__WAIKO_CONVERSIONS_HPP_

#include "waiko_ros2_control/msg/waiko_state.hpp"
#include "waiko_ros2_control/waiko_estimator.hpp"
#include "waiko_ros2_control/waiko_estimator_parameters.hpp"

namespace waiko_ros2_control
{

/// Configuration of the estimator from its ROS parameters.
WaikoConfiguration make_configuration(const waiko_estimator::Params & params);

/// Fills every field of the state message but its header.
void to_message(const WaikoEstimate & estimate, msg::WaikoState & message);

}  // namespace waiko_ros2_control

#endif  // WAIKO_ROS2_CONTROL__WAIKO_CONVERSIONS_HPP_
