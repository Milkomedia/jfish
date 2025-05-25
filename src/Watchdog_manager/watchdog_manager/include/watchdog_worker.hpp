#ifndef WATCHDOG_WORKER_HPP
#define WATCHDOG_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <string>
#include "watchdog_interfaces/msg/node_state.hpp"

#include <vector>
#include <chrono>

class WatchDogNode : public rclcpp::Node {
public:
  WatchDogNode();

private:
  // common handler for all monitored nodes
  void commonCallback(const std::string & node_name, const watchdog_interfaces::msg::NodeState::SharedPtr msg);

  // per-node subscription callbacks
  void sbusCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void controllerCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void allocatorCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void dynamixelCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void imuCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void optitrackCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void armCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);

  // timer callback: evaluate handshake timeout & runtime heartbeats
  void publishKill();

  void watchdog_Thinking();

  // subscriptions & publisher
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr controller_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr allocator_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr dynamixel_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr imu_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr optitrack_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr arm_subscription_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr watchdog_publisher_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr thinking_timer_;

  // per-node state tracking
  std::unordered_map<std::string, bool>     is_node_initiated_;   // received 42?
  std::unordered_map<std::string, uint8_t>  last_state_;   // last state counter
  std::unordered_map<std::string, rclcpp::Time> last_time_; // timestamp of last message
  
  // handshake & failure flags
  rclcpp::Time start_time_;       // node launch time
  bool      handshake_checked_;   // did we already enforce 1 s handshake window?
  bool      failure_detected_;    // once true, we always publish kill

  // timeouts (in seconds)
  static constexpr double HANDSHAKE_TIMEOUT_SEC = 5.0;
  static constexpr double HEARTBEAT_TIMEOUT_SEC = 0.2;
};

#endif // WATCHDOG_WORKER_HPP