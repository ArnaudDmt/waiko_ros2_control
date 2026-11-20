#include <gmock/gmock.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_control_test_assets/descriptions.hpp>
#include <state-observation/tools/definitions.hpp>

#include "waiko_ros2_control/waiko_controller.hpp"

namespace waiko_ros2_control
{
namespace
{

using controller_interface::CallbackReturn;
using controller_interface::return_type;
using hardware_interface::StateInterface;
using lifecycle_msgs::msg::State;

constexpr double kDt = 0.005;

TEST(WaikoControllerLoad, LoadsAsPlugin)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager manager(
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");
  manager.set_parameter(
    {"test_waiko_controller.params_file", std::string(TEST_FILES_DIRECTORY) + "/waiko_controller.yaml"});
  manager.set_parameter({"test_waiko_controller.type", "waiko_ros2_control/WaikoController"});

  EXPECT_NE(manager.load_controller("test_waiko_controller"), nullptr);
}

/// Controller fed with state interfaces of a robot standing still on two feet: identity
/// orientation, IMU 1 m above the world origin, feet fixed on the ground.
class WaikoControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ = std::make_unique<WaikoController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_waiko_controller";
    params.update_rate = 200;
    params.controller_manager_update_rate = 200;
    params.node_options.parameter_overrides(
      {{"imu_name", "imu"},
       {"contacts", std::vector<std::string>{"left_foot", "right_foot"}},
       {"initial_imu_position", std::vector<double>{0.0, 0.0, 1.0}}});
    ASSERT_EQ(controller_->init(params), return_type::OK);
  }

  /// Creates and sets the state interfaces of the robot, and loans them to the controller in the
  /// reverse of the claimed order.
  void assign_state_interfaces(const std::vector<std::string> & names)
  {
    std::vector<hardware_interface::LoanedStateInterface> loaned;
    for (auto name = names.rbegin(); name != names.rend(); ++name)
    {
      const std::size_t separator = name->find('/');
      auto interface =
        std::make_shared<StateInterface>(name->substr(0, separator), name->substr(separator + 1));
      EXPECT_TRUE(interface->set_value(value_of(*name)));
      state_interfaces_.push_back(interface);
      loaned.emplace_back(interface);
    }
    controller_->assign_interfaces({}, std::move(loaned));
  }

  /// Value of a state interface of the robot standing still.
  static double value_of(const std::string & name)
  {
    const std::string interface = name.substr(name.find('/') + 1);
    if (name == "imu/linear_acceleration.z")
    {
      return stateObservation::cst::gravityConstant;
    }
    if (interface == "active" || interface == "lambda" || interface == "orientation.w")
    {
      return 1.0;
    }
    if (name == "left_foot/position.y")
    {
      return 0.1;
    }
    if (name == "right_foot/position.y")
    {
      return -0.1;
    }
    if (interface == "position.z")
    {
      return -1.0;
    }
    return 0.0;
  }

  std::unique_ptr<WaikoController> controller_;
  std::vector<StateInterface::SharedPtr> state_interfaces_;
};

TEST_F(WaikoControllerTest, ClaimsTheImuAndContactInterfaces)
{
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  const auto configuration = controller_->state_interface_configuration();
  EXPECT_EQ(configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_THAT(
    configuration.names,
    ::testing::ElementsAre(
      "imu/angular_velocity.x", "imu/angular_velocity.y", "imu/angular_velocity.z",
      "imu/linear_acceleration.x", "imu/linear_acceleration.y", "imu/linear_acceleration.z",
      "left_foot/active", "left_foot/lambda", "left_foot/position.x", "left_foot/position.y",
      "left_foot/position.z", "left_foot/orientation.x", "left_foot/orientation.y",
      "left_foot/orientation.z", "left_foot/orientation.w", "left_foot/linear_velocity.x",
      "left_foot/linear_velocity.y", "left_foot/linear_velocity.z", "right_foot/active",
      "right_foot/lambda", "right_foot/position.x", "right_foot/position.y",
      "right_foot/position.z", "right_foot/orientation.x", "right_foot/orientation.y",
      "right_foot/orientation.z", "right_foot/orientation.w", "right_foot/linear_velocity.x",
      "right_foot/linear_velocity.y", "right_foot/linear_velocity.z"));
  EXPECT_EQ(
    controller_->command_interface_configuration().type,
    controller_interface::interface_configuration_type::NONE);
}

TEST_F(WaikoControllerTest, ActivationFailsWithoutAllInterfaces)
{
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  std::vector<std::string> names = controller_->state_interface_configuration().names;
  names.pop_back();
  assign_state_interfaces(names);

  // The activation returns ERROR, which the lifecycle handles by going back to unconfigured.
  EXPECT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(WaikoControllerTest, PublishesTheRestingPose)
{
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_state_interfaces(controller_->state_interface_configuration().names);
  ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);

  nav_msgs::msg::Odometry::SharedPtr odometry;
  auto listener = std::make_shared<rclcpp::Node>("test_waiko_listener");
  auto subscription = listener->create_subscription<nav_msgs::msg::Odometry>(
    "/test_waiko_controller/odometry", rclcpp::SystemDefaultsQoS(),
    [&odometry](nav_msgs::msg::Odometry::SharedPtr message) { odometry = message; });

  rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);
  auto step = [&]()
  {
    time += period;
    return controller_->update(time, period);
  };
  for (int i = 0; i < 400; ++i)
  {
    ASSERT_EQ(step(), return_type::OK);
  }

  // The realtime publisher sends from its own thread and drops the messages given while it is
  // busy, so the robot keeps standing until a message of a later step is received.
  const rclcpp::Time settled = time;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((!odometry || rclcpp::Time(odometry->header.stamp, RCL_ROS_TIME) < settled) &&
         std::chrono::steady_clock::now() < deadline)
  {
    ASSERT_EQ(step(), return_type::OK);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_NE(odometry, nullptr);
  EXPECT_EQ(odometry->header.frame_id, "world");
  EXPECT_EQ(odometry->child_frame_id, "imu_link");
  EXPECT_NEAR(odometry->pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(odometry->pose.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(odometry->pose.pose.position.z, 1.0, 1e-9);
  EXPECT_NEAR(odometry->pose.pose.orientation.w, 1.0, 1e-9);
  EXPECT_NEAR(odometry->twist.twist.linear.x, 0.0, 1e-9);

  EXPECT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
}

}  // namespace
}  // namespace waiko_ros2_control

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
