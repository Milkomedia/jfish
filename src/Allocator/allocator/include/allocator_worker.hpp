#ifndef ALLOCATOR_WORKER_HPP
#define ALLOCATOR_WORKER_HPP

#include <cmath> 
#include "rclcpp/rclcpp.hpp"
#include "controller_interfaces/msg/controller_output.hpp"
#include "allocator_interfaces/msg/tilt_angle_val.hpp"
#include "allocator_interfaces/msg/pwm_val.hpp"
#include "allocator_interfaces/msg/allocator_debug_val.hpp" // joint_val이 이안에 mea로 들어가짐 
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"

#include <Eigen/Dense>
#include <vector>
#include <algorithm>

class AllocatorWorker : public rclcpp::Node {
public:
  AllocatorWorker();
  ~AllocatorWorker() = default;

private:
  void controllerCallback(const controller_interfaces::msg::ControllerOutput::SharedPtr msg);
  void jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg);
  void heartbeat_timer_callback();
  void debugging_timer_callback();
  void start_allcation();

  Eigen::Matrix4d calc_A1(const Eigen::Vector4d& C1);
  Eigen::Matrix4d calc_A2(const Eigen::Vector4d& C2s);

  // Subscribers
  rclcpp::Subscription<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_subscriber_;
  rclcpp::Subscription<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_subscriber_;
  
  // Publishers
  rclcpp::Publisher<allocator_interfaces::msg::TiltAngleVal>::SharedPtr tilt_angle_publisher_;
  rclcpp::Publisher<allocator_interfaces::msg::PwmVal>::SharedPtr pwm_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<allocator_interfaces::msg::AllocatorDebugVal>::SharedPtr debug_val_publisher_;

  // Timers for publishing
  rclcpp::TimerBase::SharedPtr pwm_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr debugging_timer_;

  // Time tracking
  size_t buffer_size_ = 1000;          // Size of the moving average window
  std::vector<double> dt_buffer_;      // Circular buffer for dt values
  size_t buffer_index_  = 0;           // Current index in the circular buffer
  double dt_sum_ = 0.0;                // Sum of dt values in the buffer
  double filtered_frequency_ = 1200.0; // [Hz] calculated from average dt
  rclcpp::Time last_callback_time_;    // Timestamp of the last callback
  
  // BLDC Motor Model
  const double pwm_alpha_ = 46.5435;  // F = a * pwm^2 + b
  const double pwm_beta_  = 8.6111;   // F = a * pwm^2 + b
  const double zeta       = 0.02;     // b/k constant

  // Control Allocation params
  Eigen::Matrix<double,6,4> DH_params_; // 6x4 DH table (rows: link 0..5; cols: a, alpha, d, theta0)
  Eigen::Vector4d q_B0_;                // Body to Arm rotation angle in z axis [rad]
  Eigen::Vector4d C1_;                  // calculated thrust f_1234 [N]
  Eigen::Vector4d C2_mea_;              // FK calculated tilted angle [rad]
  Eigen::Vector4d C2_des_;              // calculated tilted angle [rad]
  Eigen::Matrix<double, 3, 4> r_mea_;   // FK calculated position vect of each arm [m]
  Eigen::Vector4d pwm_;                 // calculated pwm [0.0 ~ 1.0]
  Eigen::Vector3d Pc_;                  // CoM bias vector wrt. body frame [m]
  
  // yaw-wrench conversion params
  // const double lpf_alpha_ = 0.093158;
  const double lpf_alpha_ = 0.01;
  const double lpf_beta_  = 1.0 - lpf_alpha_;
  const double tauz_min   = -5.0; // saturation ref [Nm]
  const double tauz_max   =  5.0; // saturation ref [Nm]
  double tauz_bar_ = 0.0;

  // arm pos [rad] (mujoco or dynamixel)
  double arm_des_[4][5] = {
    {0.785398,  0.0, -1.50944, 0.0, 0.0},   // a1_des
    {2.35619,   0.0, -1.50944, 0.0, 0.0},   // a2_des
    {-2.35619,  0.0, -1.50944, 0.0, 0.0},   // a3_des
    {-0.785398, 0.0, -1.50944, 0.0, 0.0}};  // a4_des

  double arm_mea_[4][5] = { 
    {0., -0.84522, 1.50944, 0.90812, 0.},   // a1_mea
    {0., -0.84522, 1.50944, 0.90812, 0.},   // a2_mea
    {0., -0.84522, 1.50944, 0.90812, 0.},   // a3_mea
    {0., -0.84522, 1.50944, 0.90812, 0.}};  // a4_mea
  
  // heartbeat state  
  uint8_t  hb_state_;         // current heartbeat value
  bool     hb_enabled_;       // gate flag
  uint8_t  heartbeat_state_;  // previous node state

  bool     allocator_run_;    // allocator starting flag

  // ------- math utils -------
  static inline Eigen::Matrix4d compute_DH(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T;
    T << std::cos(theta), -std::sin(theta) * std::cos(alpha),  std::sin(theta) * std::sin(alpha), a * std::cos(theta),
         std::sin(theta),  std::cos(theta) * std::cos(alpha), -std::cos(theta) * std::sin(alpha), a * std::sin(theta),
         0,                std::sin(alpha),                    std::cos(alpha),                   d,
         0,                0,                                   0,                                 1;
    return T;
  }

  const double inv_sqrt2 = 0.7071067811865474617150084668537601828575;  // 1/sqrt(2)
  const double sqrt2 = 1.4142135623730951454746218587388284504414;      // sqrt(2)
};

#endif // ALLOCATOR_WORKER_HPP