#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "waiko_ros2_control/msg/waiko_initial_state.hpp"
#include "waiko_ros2_control/msg/waiko_input.hpp"
#include "waiko_ros2_control/msg/waiko_state.hpp"
#include "waiko_ros2_control/waiko_estimator.hpp"

namespace waiko_ros2_control
{

/// Runs WAIKO on the samples received on ~/input, with a fixed sampling period, and publishes the
/// estimate of each sample on ~/estimated_state. The estimator is (re)built on each ~/init_state.
class WaikoEstimatorNode : public rclcpp::Node
{
public:
  WaikoEstimatorNode()
  : Node("waiko_estimator"),
    parameters_(std::make_shared<waiko_estimator::ParamListener>(get_node_parameters_interface())),
    dt_(declare_parameter("dt", 0.005))
  {
    init_subscription_ = create_subscription<msg::WaikoInitialState>(
      "~/init_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](const msg::WaikoInitialState & message) { on_initial_state(message); });
    input_subscription_ = create_subscription<msg::WaikoInput>(
      "~/input", rclcpp::QoS(10).reliable(),
      [this](const msg::WaikoInput & message) { on_input(message); });
    state_publisher_ =
      create_publisher<msg::WaikoState>("~/estimated_state", rclcpp::QoS(10).reliable());
  }

private:
  void on_initial_state(const msg::WaikoInitialState & message)
  {
    Eigen::Vector3d local_linear_velocity, tilt, position;
    Eigen::Quaterniond orientation;
    tf2::fromMsg(message.x1, local_linear_velocity);
    tf2::fromMsg(message.x2, tilt);
    tf2::fromMsg(message.orientation, orientation);
    tf2::fromMsg(message.position, position);

    estimator_.emplace(
      parameters_->get_params(), local_linear_velocity, tilt, orientation,
      position);
    RCLCPP_INFO(get_logger(), "WAIKO initialized.");

    publish_estimate(message.header.stamp);
  }

  void on_input(const msg::WaikoInput & message)
  {
    if (!estimator_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring ~/input until ~/init_state is received.");
      return;
    }

    Eigen::Vector3d accelerometer, gyrometer;
    tf2::fromMsg(message.imu.linear_acceleration, accelerometer);
    tf2::fromMsg(message.imu.angular_velocity, gyrometer);

    contacts_.resize(message.contacts.size());
    for (std::size_t i = 0; i < message.contacts.size(); ++i)
    {
      const msg::WaikoContact & contact_message = message.contacts[i];
      ContactMeasurement & contact = contacts_[i];
      contact.name = contact_message.name;
      contact.active = contact_message.active;
      contact.lambda = contact_message.contact_lambda;
      tf2::fromMsg(contact_message.imu_contact_position, contact.position);
      tf2::fromMsg(contact_message.imu_contact_orientation, contact.orientation);
      tf2::fromMsg(contact_message.imu_contact_linear_velocity, contact.linear_velocity);
    }

    estimator_->update(dt_, accelerometer, gyrometer, contacts_, message.reset_local_linear_velocity);
    publish_estimate(message.header.stamp);
  }

  void publish_estimate(const builtin_interfaces::msg::Time & stamp)
  {
    msg::WaikoState message;
    message.header.stamp = stamp;
    message.header.frame_id = "world";
    to_message(estimator_->estimate(), message);
    state_publisher_->publish(message);
  }

  std::shared_ptr<waiko_estimator::ParamListener> parameters_;
  /// Sampling period of the inputs (s).
  double dt_;
  std::optional<WaikoEstimator> estimator_;
  std::vector<ContactMeasurement> contacts_;

  rclcpp::Subscription<msg::WaikoInitialState>::SharedPtr init_subscription_;
  rclcpp::Subscription<msg::WaikoInput>::SharedPtr input_subscription_;
  rclcpp::Publisher<msg::WaikoState>::SharedPtr state_publisher_;
};

}  // namespace waiko_ros2_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waiko_ros2_control::WaikoEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
