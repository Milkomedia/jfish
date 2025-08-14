#ifndef ARM_CHANGER_WORKER_HPP
#define ARM_CHANGER_WORKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sbus_interfaces/msg/sbus_signal.hpp"
#include "sbus_interfaces/msg/kill_cmd.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include <array>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class ArmChangerWorker : public rclcpp::Node {
public:
  ArmChangerWorker();
  ~ArmChangerWorker() = default;

private:
  // Callback to handle received PwmVal messages
  void sbus_callback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void killCmd_callback(const sbus_interfaces::msg::KillCmd::SharedPtr msg);
  void watchdog_callback(const watchdog_interfaces::msg::NodeState::SharedPtr msg); // 뭐하는 새끼임?
  std::array<double, 5> compute_ik(const double x, const double y, const double z, const Eigen::Vector3d &heading);
  void heartbeat_timer_callback();

  // Publishers
  rclcpp::Publisher<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;

  // Subscribers
  rclcpp::Subscription<sbus_interfaces::msg::KillCmd>::SharedPtr killcmd_subscription_;
  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;

  // Timers
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // DH params
  const double a1_ = 134.;
  const double a2_ = 115.;
  const double a3_ = 110.;
  const double a4_ = 24.;
  const double a5_ = 104.;

  // workspace constrain
  const double x_min_   = 264.; 
  const double x_max_   = 325.;
  const double z_min_   = 186.;
  const double z_max_   = 250.;

  // heartbeat state
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag

  // Watchdog state
  uint8_t watchdog_state_ = 1; // default(normal) is 1.
  bool kill_activated_ = true;
};

#endif // ARM_CHANGER_WORKER_HPP