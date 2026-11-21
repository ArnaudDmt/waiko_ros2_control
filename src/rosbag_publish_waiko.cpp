#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "waiko_ros2_control/msg/waiko_initial_state.hpp"
#include "waiko_ros2_control/msg/waiko_input.hpp"
#include "waiko_ros2_control/msg/waiko_state.hpp"

namespace waiko_ros2_control
{

namespace
{

constexpr char kInitTopic[] = "/waiko_estimator/init_state";
constexpr char kInputTopic[] = "/waiko_estimator/input";
constexpr char kStateTopic[] = "/waiko_estimator/estimated_state";

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  const rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  MessageT message;
  rclcpp::Serialization<MessageT>().deserialize_message(&serialized, &message);
  return message;
}

}  // namespace

/// Replays the WAIKO initial state and inputs of a bag in lockstep with the estimator: each message
/// is published once the estimate of the previous one is received, so that no sample is dropped and
/// the result does not depend on the replay speed. Shuts down after the last estimate.
class RosbagPublishWaiko : public rclcpp::Node
{
public:
  RosbagPublishWaiko() : Node("rosbag_publish_waiko")
  {
    const std::string bag_path = declare_parameter<std::string>("bag_path");
    read_bag(bag_path);

    init_publisher_ = create_publisher<msg::WaikoInitialState>(
      kInitTopic, rclcpp::QoS(1).reliable().transient_local());
    input_publisher_ = create_publisher<msg::WaikoInput>(kInputTopic, rclcpp::QoS(10).reliable());
    state_subscription_ = create_subscription<msg::WaikoState>(
      kStateTopic, rclcpp::QoS(10).reliable(),
      [this](const msg::WaikoState & state) { on_state(state); });

    // The estimator answers the initial state with its initial estimate, which starts the replay.
    expected_stamp_ = initial_state_.header.stamp;
    init_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { offer_initial_state(); });
    RCLCPP_INFO(
      get_logger(), "Replaying '%s': initial state and %zu inputs.", bag_path.c_str(),
      inputs_.size());
  }

private:
  void read_bag(const std::string & bag_path)
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag_path);

    std::optional<msg::WaikoInitialState> initial_state;
    while (reader.has_next())
    {
      const auto bag_message = reader.read_next();
      if (bag_message->topic_name == kInitTopic && !initial_state)
      {
        initial_state = deserialize<msg::WaikoInitialState>(*bag_message);
      }
      else if (bag_message->topic_name == kInputTopic)
      {
        inputs_.push_back(deserialize<msg::WaikoInput>(*bag_message));
      }
    }

    if (!initial_state)
    {
      throw std::runtime_error(std::string("No ") + kInitTopic + " in bag " + bag_path);
    }
    if (inputs_.empty())
    {
      throw std::runtime_error(std::string("No ") + kInputTopic + " in bag " + bag_path);
    }
    initial_state_ = *initial_state;
  }

  /// Called each period until the estimator answers. Waits for the estimator to subscribe to the
  /// inputs, so that the first one is not lost, then publishes the initial state. It is sent again
  /// while unanswered because an answer published before the estimator discovered this node is lost;
  /// receiving it again only rebuilds the estimator identically.
  void offer_initial_state()
  {
    if (input_publisher_->get_subscription_count() > 0)
    {
      init_publisher_->publish(initial_state_);
    }
  }

  void on_state(const msg::WaikoState & state)
  {
    if (state.header.stamp != expected_stamp_)
    {
      return;
    }
    init_timer_->cancel();
    if (next_input_ == inputs_.size())
    {
      RCLCPP_INFO(get_logger(), "Replay finished.");
      rclcpp::shutdown();
      return;
    }
    const msg::WaikoInput & input = inputs_[next_input_++];
    expected_stamp_ = input.header.stamp;
    input_publisher_->publish(input);
  }

  msg::WaikoInitialState initial_state_;
  std::vector<msg::WaikoInput> inputs_;
  /// Index of the next input to publish.
  std::size_t next_input_ = 0;
  /// Stamp of the last published message, carried by the estimate that answers it.
  builtin_interfaces::msg::Time expected_stamp_;

  rclcpp::Publisher<msg::WaikoInitialState>::SharedPtr init_publisher_;
  rclcpp::Publisher<msg::WaikoInput>::SharedPtr input_publisher_;
  rclcpp::Subscription<msg::WaikoState>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace waiko_ros2_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waiko_ros2_control::RosbagPublishWaiko>());
  rclcpp::shutdown();
  return 0;
}
