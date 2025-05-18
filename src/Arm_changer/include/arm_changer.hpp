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

constexpr double PI = 3.1415926535897932384626433832706;

class ArmChangerWorker : public rclcpp::Node {
public:
  ArmChangerWorker();
  ~ArmChangerWorker() = default;

private:
  // Callback to handle received PwmVal messages
  void sbus_callback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void killCmd_callback(const sbus_interfaces::msg::KillCmd::SharedPtr msg);
  void watchdog_callback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void compute_ik(const double x, const double y, const double z, const Eigen::Vector3d &heading);
  void joint_callback();
  void heartbeat_timer_callback();

  // Publishers
  rclcpp::Publisher<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;

  // Subscribers
  rclcpp::Subscription<sbus_interfaces::msg::KillCmd>::SharedPtr killcmd_subscription_;
  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;

  // Timers
  rclcpp::TimerBase::SharedPtr joint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // DH params
  const double a1_ = 134.;
  const double a2_ = 115.;
  const double a3_ = 110.;
  const double a4_ = 24.;
  const double a5_ = 104.;

  inline Eigen::Matrix3d R01(double th1) {
    Eigen::Matrix3d R;
    R << std::cos(th1), 0, std::sin(th1),
         std::sin(th1), 0, -std::cos(th1),
         0,             1, 0;
    return R;
  }  
  
  inline Eigen::Matrix3d R12(double th2) {
    Eigen::Matrix3d R;
    R << std::cos(th2), -std::sin(th2), 0,
         std::sin(th2),  std::cos(th2), 0,
         0,              0,             1;
    return R;
  }
  
  inline Eigen::Matrix3d R23(double th3) {
    Eigen::Matrix3d R;
    R << std::cos(th3), -std::sin(th3), 0,
         std::sin(th3),  std::cos(th3), 0,
         0,              0,             1;
    return R;
  }
  
  inline Eigen::Matrix3d R34(double th4) {
    Eigen::Matrix3d R;
    R << std::cos(th4), 0, -std::sin(th4),
         std::sin(th4), 0,  std::cos(th4),
         0,            -1, 0;
    return R;
  }

  // workspace constrain
  double x_min_; 
  double x_max_;
  double y_fixed_;
  double z_min_;
  double z_max_;

  const Eigen::Vector3d heading_fixed_ = Eigen::Vector3d(0.0, 0.0, 1.0); // only z-up

  // Latest Joint values
  double th1_ = 0.0;            // [rad]
  double th2_ = -0.785398163;   // [rad]
  double th3_ = 1.57079633;     // [rad]
  double th4_ = 0.785398163;    // [rad]
  double th5_ = 0.0;            // [rad]

  std::array<double, 5> a1_q, a2_q, a3_q, a4_q;

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag

  // Watchdog state
  uint8_t watchdog_state_ = 1; // default(normal) is 1.
  bool kill_activated_ = true;
};

#endif // ARM_CHANGER_WORKER_HPP