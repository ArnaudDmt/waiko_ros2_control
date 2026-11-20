#ifndef WAIKO_ROS2_CONTROL__WAIKO_CONTROLLER_HPP_
#define WAIKO_ROS2_CONTROL__WAIKO_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "waiko_ros2_control/msg/waiko_state.hpp"
#include "waiko_ros2_control/waiko_controller_parameters.hpp"
#include "waiko_ros2_control/waiko_estimator.hpp"
#include "waiko_ros2_control/waiko_estimator_parameters.hpp"

namespace waiko_ros2_control
{

/// ros2_control controller running WAIKO on the IMU and contact state interfaces of the robot.
///
/// It claims no command interface. It reads:
///  - <imu_name>/angular_velocity.{x,y,z} and <imu_name>/linear_acceleration.{x,y,z};
///  - for each contact, <contact>/active, <contact>/lambda, <contact>/position.{x,y,z},
///    <contact>/orientation.{x,y,z,w} and <contact>/linear_velocity.{x,y,z}, in the IMU frame.
/// These contact kinematics are given by the hardware interface or an upstream controller.
///
/// It publishes, at each update:
///  - ~/odometry (nav_msgs/Odometry): pose of the IMU in the world, twist in the IMU frame;
///  - ~/estimated_state (waiko_ros2_control/WaikoState): full state of the estimator.
///
/// The estimator starts from initial_imu_position and initial_imu_orientation at each activation.
class WaikoController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Reads the IMU and contact measurements; false if a state interface could not be read.
  bool read_measurements();

  void publish(const rclcpp::Time & time);

  std::shared_ptr<waiko_controller::ParamListener> controller_parameter_listener_;
  std::shared_ptr<waiko_estimator::ParamListener> estimator_parameter_listener_;
  waiko_controller::Params parameters_;

  /// Full names of the claimed state interfaces, in reading order.
  std::vector<std::string> state_interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    ordered_state_interfaces_;
  std::vector<double> state_values_;

  Eigen::Vector3d accelerometer_;
  Eigen::Vector3d gyrometer_;
  std::vector<ContactMeasurement> contacts_;
  std::optional<WaikoEstimator> estimator_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_;
  nav_msgs::msg::Odometry odometry_message_;

  rclcpp::Publisher<msg::WaikoState>::SharedPtr state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<msg::WaikoState>> realtime_state_publisher_;
  msg::WaikoState state_message_;
};

}  // namespace waiko_ros2_control

#endif  // WAIKO_ROS2_CONTROL__WAIKO_CONTROLLER_HPP_
