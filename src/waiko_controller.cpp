#include "waiko_ros2_control/waiko_controller.hpp"

#include <array>
#include <exception>

#include <controller_interface/helpers.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


namespace waiko_ros2_control
{

namespace
{

/// State interfaces of the IMU, in reading order.
const std::array<std::string, 6> kImuInterfaces = {
  "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

/// State interfaces of each contact, in reading order.
const std::array<std::string, 12> kContactInterfaces = {
  "active",        "lambda",        "position.x",        "position.y",
  "position.z",    "orientation.x", "orientation.y",     "orientation.z",
  "orientation.w", "linear_velocity.x", "linear_velocity.y", "linear_velocity.z"};

}  // namespace

controller_interface::CallbackReturn WaikoController::on_init()
{
  try
  {
    controller_parameter_listener_ = std::make_shared<waiko_controller::ParamListener>(get_node());
    estimator_parameter_listener_ = std::make_shared<waiko_estimator::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare the parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration WaikoController::command_interface_configuration()
  const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration WaikoController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_interface_names_};
}

controller_interface::CallbackReturn WaikoController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  parameters_ = controller_parameter_listener_->get_params();

  state_interface_names_.clear();
  for (const std::string & interface : kImuInterfaces)
  {
    state_interface_names_.push_back(parameters_.imu_name + "/" + interface);
  }
  contacts_.clear();
  for (const std::string & contact : parameters_.contacts)
  {
    for (const std::string & interface : kContactInterfaces)
    {
      state_interface_names_.push_back(contact + "/" + interface);
    }
    contacts_.push_back({contact, false, 0.0, {}, {}, {}});
  }
  state_values_.resize(state_interface_names_.size());

  odometry_publisher_ =
    get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odometry", rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);
  odometry_message_.header.frame_id = parameters_.world_frame_id;
  odometry_message_.child_frame_id = parameters_.imu_frame_id;

  state_publisher_ =
    get_node()->create_publisher<msg::WaikoState>("~/estimated_state", rclcpp::SystemDefaultsQoS());
  realtime_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<msg::WaikoState>>(state_publisher_);
  state_message_.header.frame_id = parameters_.world_frame_id;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WaikoController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ordered_state_interfaces_.clear();
  if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, state_interface_names_, "", ordered_state_interfaces_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu state interfaces, got %zu.",
      state_interface_names_.size(), ordered_state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const Eigen::Vector3d position(parameters_.initial_imu_position.data());
  const Eigen::Vector3d linear_velocity(parameters_.initial_imu_linear_velocity.data());
  const Eigen::Quaterniond orientation =
    Eigen::Quaterniond(parameters_.initial_imu_orientation.data()).normalized();  // [x, y, z, w]
  const Eigen::Vector3d tilt = orientation.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  estimator_.emplace(
    estimator_parameter_listener_->get_params(), linear_velocity, tilt, orientation, position);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WaikoController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ordered_state_interfaces_.clear();
  estimator_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WaikoController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!read_measurements())
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "A state interface could not be read, the estimation step is skipped.");
    return controller_interface::return_type::OK;
  }

  estimator_->update(period.seconds(), accelerometer_, gyrometer_, contacts_);
  publish(time);
  return controller_interface::return_type::OK;
}

bool WaikoController::read_measurements()
{
  for (std::size_t i = 0; i < ordered_state_interfaces_.size(); ++i)
  {
    const std::optional<double> value = ordered_state_interfaces_[i].get().get_optional();
    if (!value)
    {
      return false;
    }
    state_values_[i] = *value;
  }

  // The values follow the order of kImuInterfaces, then of kContactInterfaces for each contact.
  const double * value = state_values_.data();
  auto next_scalar = [&value]() { return *value++; };
  auto next_vector3 = [&value]()
  {
    const Eigen::Vector3d vector(value);
    value += 3;
    return vector;
  };
  auto next_quaternion = [&value]()
  {
    const Eigen::Quaterniond quaternion(value);  // from [x, y, z, w]
    value += 4;
    return quaternion.normalized();
  };

  gyrometer_ = next_vector3();
  accelerometer_ = next_vector3();
  for (ContactMeasurement & contact : contacts_)
  {
    contact.active = next_scalar() != 0.0;
    contact.lambda = next_scalar();
    contact.position = next_vector3();
    contact.orientation = next_quaternion();
    contact.linear_velocity = next_vector3();
  }
  return true;
}

void WaikoController::publish(const rclcpp::Time & time)
{
  const WaikoEstimate & estimate = estimator_->estimate();

  odometry_message_.header.stamp = time;
  odometry_message_.pose.pose.position = tf2::toMsg(estimate.position);
  odometry_message_.pose.pose.orientation = tf2::toMsg(estimate.orientation);
  tf2::toMsg(estimate.local_linear_velocity, odometry_message_.twist.twist.linear);
  tf2::toMsg(estimate.angular_velocity, odometry_message_.twist.twist.angular);
  realtime_odometry_publisher_->try_publish(odometry_message_);

  state_message_.header.stamp = time;
  to_message(estimate, state_message_);
  realtime_state_publisher_->try_publish(state_message_);
}

}  // namespace waiko_ros2_control

PLUGINLIB_EXPORT_CLASS(waiko_ros2_control::WaikoController, controller_interface::ControllerInterface)
