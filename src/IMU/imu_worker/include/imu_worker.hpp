#ifndef IMU_WORKER_HPP
#define IMU_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include "imu_interfaces/msg/imu_measured.hpp"
#include "mujoco_interfaces/msg/mu_jo_co_meas.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>
#include <deque>
#include <functional>
#include <random>
#include <cmath>

constexpr double two_PI = 2.0 * M_PI;
constexpr double noise_quat_std_dev = 0.001;
constexpr double noise_gyro_std_dev = 0.005;

struct DelayedData
{
  rclcpp::Time stamp;
  std::array<double, 4> q;
  std::array<double, 3> w;
};

class IMUnode : public rclcpp::Node {
public:
  IMUnode();

private:
  void PublishMuJoCoMeasurement();
  void heartbeat_timer_callback();
  void mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Subscriber
  rclcpp::Subscription<mujoco_interfaces::msg::MuJoCoMeas>::SharedPtr mujoco_subscription_;

  // Buffer (FIFO) to store data for delayed output
  std::deque<DelayedData> data_buffer_;

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
};

#endif // IMU_WORKER_HPP