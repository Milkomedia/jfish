#ifndef IMU_WORKER_HPP
#define IMU_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include "imu_interfaces/msg/imu_measured.hpp"
#include "mujoco_interfaces/msg/mu_jo_co_meas.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>
#include <deque>
#include <functional>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <thread>

constexpr double two_PI = 2.0 * M_PI;
constexpr double INV_SQRT2 = 0.7071067811865475244;

constexpr double noise_quat_std_dev = 0.001;
constexpr double noise_gyro_std_dev = 0.005;
constexpr double noise_accel_std_dev = 0.01;

struct Delayed_IMUdata
{
  rclcpp::Time stamp;
  std::array<double, 4> q;
  std::array<double, 3> w;
  std::array<double, 3> a;
};

struct IMUdata
{
  std::array<double, 4> q;
  std::array<double, 3> w;
};

class IMUnode : public rclcpp::Node {
public:
  IMUnode();

private:
  void PublishMuJoCoMeasurement();
  void PublishMicroStrainMeasurement();
  void heartbeat_timer_callback();
  void mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg);
  void microstrain_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg);

  bool imu_hz_check();

  // Publisher
  rclcpp::Publisher<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Subscriber
  rclcpp::Subscription<mujoco_interfaces::msg::MuJoCoMeas>::SharedPtr mujoco_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr microstrain_subscription_;
  rclcpp::Subscription<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_subscriber_;

  // Buffer (FIFO) to store data for delayed output
  std::deque<Delayed_IMUdata> data_buffer_;

  // Duration representing 3ms delay (3,000,000ns)
  rclcpp::Duration delay_{0, 3000000};

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag

  // Random number generator as member variables for reuse
  std::mt19937 gen_;
  std::normal_distribution<double> angle_dist_;
  std::normal_distribution<double> axis_dist_;
  std::normal_distribution<double> noise_dist_;

  // Real imu (microstrain) Data
  IMUdata real_imu_data_;

  // Buffer to store recent IMU callback timestamps for freq estimation
  std::deque<rclcpp::Time> imu_stamp_buffer_;
  const rclcpp::Duration check_horizon_{0, 300000000}; // (0.3s)

  // coordinate change data {b}2{CoT} 
  Eigen::Matrix<double,6,4> DH_params_; // 6x4 DH table (rows: link 0..5; cols: a, alpha, d, theta0)
  Eigen::Vector4d q_B0_;                // Body to Arm rotation angle in z axis [rad]
  Eigen::Matrix<double, 3, 4> B_p_arm;   // position vector group of each arm baed on FK [m]
  Eigen::Matrix<double, 3, 4> B_h_arm;   // x direction heading vector group of each arm on {B} [unit]
  Eigen::Vector3d B_p_cot;
  Eigen::Matrix3d B_R_cot;
  Eigen::Matrix3d G_R_B;

  double arm_des_[4][5] = {
    {0.785398,  0.0, -1.50944, 0.0, 0.0},   // a1_des
    {2.35619,   0.0, -1.50944, 0.0, 0.0},   // a2_des
    {-2.35619,  0.0, -1.50944, 0.0, 0.0},   // a3_des
    {-0.785398, 0.0, -1.50944, 0.0, 0.0}};  // a4_des

  static inline Eigen::Matrix4d compute_DH(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T;
    T << std::cos(theta), -std::sin(theta) * std::cos(alpha),  std::sin(theta) * std::sin(alpha), a * std::cos(theta),
         std::sin(theta),  std::cos(theta) * std::cos(alpha), -std::cos(theta) * std::sin(alpha), a * std::sin(theta),
         0,                std::sin(alpha),                    std::cos(alpha),                   d,
         0,                0,                                   0,                                1;
    return T;
  }

  static inline Eigen::Matrix3d q2rot(const double q[4]){
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;

    Eigen::Matrix3d R;
    R(0,0) = 1.0 - 2.0 * (yy + zz);
    R(0,1) = 2.0 * (xy - wz);
    R(0,2) = 2.0 * (xz + wy);

    R(1,0) = 2.0 * (xy + wz);
    R(1,1) = 1.0 - 2.0 * (xx + zz);
    R(1,2) = 2.0 * (yz - wx);

    R(2,0) = 2.0 * (xz - wy);
    R(2,1) = 2.0 * (yz + wx);
    R(2,2) = 1.0 - 2.0 * (xx + yy);

    return R;
  }

  static inline void rot2q(const Eigen::Matrix3d &R, double q_out[4]){
      double tr = R(0,0) + R(1,1) + R(2,2);

      if (tr > 0.0) {
        double S  = std::sqrt(tr + 1.0) * 2.0; // S = 4*w
        q_out[0] = 0.25 * S;
        q_out[1] = (R(2,1) - R(1,2)) / S;
        q_out[2] = (R(0,2) - R(2,0)) / S;
        q_out[3] = (R(1,0) - R(0,1)) / S;
      } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        double S  = std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2.0;
        q_out[0] = (R(2,1) - R(1,2)) / S;
        q_out[1] = 0.25 * S;
        q_out[2] = (R(0,1) + R(1,0)) / S;
        q_out[3] = (R(0,2) + R(2,0)) / S;
      } else if (R(1,1) > R(2,2)) {
        double S  = std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2.0;
        q_out[0] = (R(0,2) - R(2,0)) / S;
        q_out[1] = (R(0,1) + R(1,0)) / S;
        q_out[2] = 0.25 * S;
        q_out[3] = (R(1,2) + R(2,1)) / S;
      } else {
        double S  = std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2.0;
        q_out[0] = (R(1,0) - R(0,1)) / S;
        q_out[1] = (R(0,2) + R(2,0)) / S;
        q_out[2] = (R(1,2) + R(2,1)) / S;
        q_out[3] = 0.25 * S;
      }

      double n = std::sqrt(q_out[0]*q_out[0] + q_out[1]*q_out[1] + q_out[2]*q_out[2] + q_out[3]*q_out[3]);
      if (n > 0.0) for (int i = 0; i < 4; ++i) q_out[i] /= n;
  }

};

#endif // IMU_WORKER_HPP