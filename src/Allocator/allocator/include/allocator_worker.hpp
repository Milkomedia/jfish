#ifndef ALLOCATOR_WORKER_HPP
#define ALLOCATOR_WORKER_HPP

#include <cmath> 
#include "rclcpp/rclcpp.hpp"
#include "controller_interfaces/msg/controller_output.hpp"
#include "allocator_interfaces/msg/pwm_val.hpp"
#include "allocator_interfaces/msg/allocator_debug_val.hpp" //joint_val이 이안에 mea로 들어가짐 
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

#define A1 0.13 // Arm length [m] 
#define A2 0.15 // Arm length [m] 
#define A3 0.18 // Arm length [m] 
#define A4 0.6625 // Arm length [m]
#define A5 0.64 // Arm length [m]
#define A_B 0.06 // Arm length between {B} and {0} [m]

#define pwm_alpha_ 46.5435  // F = a * pwm^2 + b
#define pwm_beta_ 8.6111    // F = a * pwm^2 + b
#define zeta  0.21496 // b/k Constant 1.5958e-8

class AllocatorWorker : public rclcpp::Node {
public:
  AllocatorWorker();
  ~AllocatorWorker() = default;

private:
  void controllerCallback(const controller_interfaces::msg::ControllerOutput::SharedPtr msg);
  void jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg);
  void heartbeat_timer_callback();
  void debugging_timer_callback();
  void publishPwmVal();

  // Subscribers
  rclcpp::Subscription<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_subscriber_;
  rclcpp::Subscription<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_subscriber_;
  
  // Publishers
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
  
  std::array<double, 4> q_B0 = {M_PI/4, (M_PI/4+M_PI/2),-(M_PI/4+M_PI/2),-M_PI/4};

  // mujoco or dynamixel [rad]
  double arm_des[4][5] = {
    {0.785398,  0.0, -1.50944, 0.0, 0.0},   // a1_des
    {2.35619,   0.0, -1.50944, 0.0, 0.0},   // a2_des
    {-2.35619,  0.0, -1.50944, 0.0, 0.0},   // a3_des
    {-0.785398, 0.0, -1.50944, 0.0, 0.0}};  // a4_des

  double arm_mea[4][5] = { 
    {0., -0.84522, 1.50944, 0.90812, 0.},   // a1_mea
    {0., -0.84522, 1.50944, 0.90812, 0.},   // a2_mea
    {0., -0.84522, 1.50944, 0.90812, 0.},   // a3_mea
    {0., -0.84522, 1.50944, 0.90812, 0.}};  // a4_mea

  MatrixXd DH_params;
  Vector4d W1 = Vector4d::Zero();   // PID-control result [N.m N.m N.m N]
  Vector4d f = Vector4d::Zero();    // Allocated result [N N N N]
  Vector4d pwm = Vector4d::Zero();  // Allocated result [pwm pwm pwm pwm]
  Vector3d CoM = Vector3d::Zero();  // Center of Mass position [m m m]
  
  Matrix4d Transformation_a1 = Matrix4d::Identity();
  Matrix4d Transformation_a2 = Matrix4d::Identity();
  Matrix4d Transformation_a3 = Matrix4d::Identity();
  Matrix4d Transformation_a4 = Matrix4d::Identity();

  MatrixXd A_1 = MatrixXd::Zero(4, 12);
  MatrixXd A_2 = MatrixXd::Zero(12, 4);
  MatrixXd A = MatrixXd::Zero(4, 4);
  MatrixXd A_inv = MatrixXd::Zero(4,4);

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag
  uint8_t heartbeat_state_; // previous node state
};

#endif // ALLOCATOR_WORKER_HPP