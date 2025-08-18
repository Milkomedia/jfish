#ifndef ARM_CHANGER_HPP
#define ARM_CHANGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sbus_interfaces/msg/sbus_signal.hpp"
#include "sbus_interfaces/msg/kill_cmd.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include <array>
#include <Eigen/Dense>
#include <vector>

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
  bool ik_check(const std::array<double,5>& q, const Eigen::Vector3d& pos_des, const Eigen::Vector3d& heading_des) const;
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

  double tilted_rad = 0.0872665; // 5 deg
};


static inline Eigen::Matrix4d T_dh(double a, double alpha, double d, double theta) {
  using std::cos; using std::sin;
  Eigen::Matrix4d T;

  T <<  cos(theta),        -sin(theta) * cos(alpha),   sin(theta) * sin(alpha),    a * cos(theta),
        sin(theta),         cos(theta) * cos(alpha),  -cos(theta) * sin(alpha),    a * sin(theta),
        0.0,                sin(alpha),               cos(alpha),                  d,
        0.0,                0.0,                      0.0,                         1.0;

  return T;
}

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

static inline double map(double input, double in_min, double in_max, double out_min, double out_max) {
  return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // ARM_CHANGER_HPP