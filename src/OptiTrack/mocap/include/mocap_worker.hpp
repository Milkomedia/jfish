#ifndef MOCAP_WORKER_HPP
#define MOCAP_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include "mocap_interfaces/msg/mocap_measured.hpp"
#include "mujoco_interfaces/msg/mu_jo_co_meas.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "mocap_interfaces/msg/named_pose_array.hpp"
#include <chrono>
#include <deque>
#include <functional>
#include <random>
#include <cmath>
#include <thread>

constexpr double noise_pos_std_dev = 0.001;
constexpr double noise_vel_std_dev = 0.005;
constexpr double noise_acc_std_dev = 0.01;

struct Delayed_OPTIdata {
  rclcpp::Time stamp;
  std::array<double, 3> pos;
  std::array<double, 3> vel;
  std::array<double, 3> acc;
};

struct OPTIdata {
  std::array<double, 3> pos;
};

class OptiTrackNode : public rclcpp::Node {
public:
  OptiTrackNode();

private:
  void PublishOptiTrackMeasurement();
  void PublishMuJoCoMeasurement();
  void heartbeat_timer_callback();
  void optitrack_callback(const mocap_interfaces::msg::NamedPoseArray::SharedPtr msg);
  void mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg);

  bool opti_hz_check();

  // optitrack Subscriber
  rclcpp::Subscription<mocap_interfaces::msg::NamedPoseArray>::SharedPtr optitrack_mea_subscription_;
  // MuJoCo Subscriber
  rclcpp::Subscription<mujoco_interfaces::msg::MuJoCoMeas>::SharedPtr mujoco_subscription_;

  rclcpp::Publisher<mocap_interfaces::msg::MocapMeasured>::SharedPtr mocap_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Publisher&Timer for Heartbeat signal
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag
  
  // Buffer (FIFO) to store data for delayed output
  std::deque<Delayed_OPTIdata> data_buffer_;

  // Duration representing 3ms delay (3,000,000ns)
  rclcpp::Duration delay_{0, 3000000};

  uint8_t heartbeat_state_;  // previous node state

  // Random number generator as member variables for reuse
  std::mt19937 gen_;
  std::normal_distribution<double> pos_dist_;
  std::normal_distribution<double> vel_dist_;
  std::normal_distribution<double> acc_dist_;

  // optitrack data
  OPTIdata real_optitrack_data_;

  // Buffer to store recent OptiTrack callback timestamps for freq estimation
  std::deque<rclcpp::Time> opti_stamp_buffer_;
  const rclcpp::Duration check_horizon_{0, 500000000};
  rclcpp::Duration horizon_ = rclcpp::Duration::from_seconds(0.5);
};

#endif // MOCAP_WORKER_HPP