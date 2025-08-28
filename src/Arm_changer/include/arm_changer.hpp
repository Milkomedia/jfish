#ifndef ARM_CHANGER_WORKER_HPP
#define ARM_CHANGER_WORKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sbus_interfaces/msg/sbus_signal.hpp"
#include "sbus_interfaces/msg/kill_cmd.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include "controller_interfaces/msg/estimator_output.hpp"
#include "allocator_interfaces/msg/tilt_angle_val.hpp"
#include <array>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

constexpr double half_sqrt2 = 1.4142135623730950488016887/2.0;

class ArmChangerWorker : public rclcpp::Node {
public:
  ArmChangerWorker();
  ~ArmChangerWorker() = default;

private:
  // Callback to handle received PwmVal messages
  void sbus_callback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void killCmd_callback(const sbus_interfaces::msg::KillCmd::SharedPtr msg);
  std::array<double,5> compute_ik(const Eigen::Vector3d &p05, const Eigen::Vector3d &heading);
  void estimator_callback(const controller_interfaces::msg::EstimatorOutput::SharedPtr msg);
  void joint_callback();
  void heartbeat_timer_callback();
  void TiltAngle_callback(const allocator_interfaces::msg::TiltAngleVal::SharedPtr msg);

  //check funtion
  bool ik_check(const std::array<double,5>& q, const Eigen::Vector3d& pos_des, const Eigen::Vector3d& heading_des) const;
  bool path_check(const Eigen::Vector3d& prev_pos, const Eigen::Vector3d& curr_pos, const double dt) const;
  bool collision_check(const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& p4) const;

  // Publishers
  rclcpp::Publisher<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;

  // Subscribers
  rclcpp::Subscription<controller_interfaces::msg::EstimatorOutput>::SharedPtr controller_subscription_;
  rclcpp::Subscription<sbus_interfaces::msg::KillCmd>::SharedPtr killcmd_subscription_;
  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<allocator_interfaces::msg::TiltAngleVal>::SharedPtr tilt_angle_val_subscription_;

  // Timers
  rclcpp::TimerBase::SharedPtr joint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // CoM position
  Eigen::Vector3d CoM_pos_ = Eigen::Vector3d::Zero();
  const double lambda_ = 0.05;
  double delta_x_ = 0;
  double delta_y_ = 0;

  // DH params
  const double DH_a1_ = 134.;
  const double DH_a2_ = 115.;
  const double DH_a3_ = 110.;
  const double DH_a4_ = 24.;
  const double DH_a5_ = 68.;


  // workspace constrain
  double x_min_ = -23.0; 
  double x_max_ =  23.0;
  double y_min_ = -23.0;
  double y_max_ =  23.0;  

  Eigen::VectorXd a1_q_ = (Eigen::VectorXd(5) << 0., 0.095993089, 0.67544228, 0.806341947, 0.0).finished(); // [rad]
  Eigen::VectorXd a2_q_ = (Eigen::VectorXd(5) << 0., 0.095993089, 0.67544228, 0.806341947, 0.0).finished(); // [rad]
  Eigen::VectorXd a3_q_ = (Eigen::VectorXd(5) << 0., 0.095993089, 0.67544228, 0.806341947, 0.0).finished(); // [rad]
  Eigen::VectorXd a4_q_ = (Eigen::VectorXd(5) << 0., 0.095993089, 0.67544228, 0.806341947, 0.0).finished(); // [rad]

  //Tilt value
  Eigen::Vector4d C2_;           // calculated tilted angle [rad]

  // path_check
  std::array<bool, 4> has_last_des_{};
  std::array<rclcpp::Time, 4> last_des_time_;
  std::array<Eigen::Vector3d, 4> last_des_pos_{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())
  };
  
  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag

  // Watchdog state
  bool kill_activated_ = true;
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

inline std::pair<Eigen::Vector3d,Eigen::Vector3d> arm2body(const Eigen::Vector3d& pos_local, const Eigen::Vector3d& n_local, int arm_number)
{
  const double A_B = 120;
  const double q_B0[4] = {M_PI/4, M_PI/4 + M_PI/2, -(M_PI/4 + M_PI/2), -M_PI/4};
  const double yaw = q_B0[arm_number - 1];
  const Eigen::Matrix4d TB0 = T_dh(A_B, 0.0, 0.0, yaw);
  const Eigen::Matrix3d R   = TB0.block<3,3>(0,0);
  const Eigen::Vector3d t   = TB0.block<3,1>(0,3);

  const Eigen::Vector3d p_base = R * pos_local + t;
  const Eigen::Vector3d n_base = (R * n_local).normalized();

  return {p_base, n_base};
}

static inline bool OverLapped(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double R, double T)
{
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  const double dz = std::abs(a.z() - b.z());
  const double ddistance = dx*dx + dy*dy;

  const double rrsum = (2.0 * R) * (2.0 * R);
  if (ddistance < rrsum && dz < T) return true;
  return false;
}

#endif // ARM_CHANGER_WORKER_HPP