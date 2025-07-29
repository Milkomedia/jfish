#ifndef TEENSY_WORKER_HPP
#define TEENSY_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include "allocator_interfaces/msg/pwm_val.hpp"
#include "sbus_interfaces/msg/kill_cmd.hpp"
#include "mujoco_interfaces/msg/motor_thrust.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"

// for can
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>

// for delayed data
#include <deque>

// for power off logic
#include <thread>
#include <chrono>

struct DelayedData
{
  rclcpp::Time stamp;
  std::array<double, 4> pwm_val; // pwm1, pwm2, pwm3, pwm4
};

class TeensyNode : public rclcpp::Node {
public:
  TeensyNode();
  ~TeensyNode();

private:
  // Callback to handle received PwmVal messages
  void allocatorCallback_save_to_CAN_buff(const allocator_interfaces::msg::PwmVal::SharedPtr msg);
  void CAN_transmit();
  void allocatorCallback_MUJ_send(const allocator_interfaces::msg::PwmVal::SharedPtr msg);
  void KillCmdCallback(const sbus_interfaces::msg::KillCmd::SharedPtr msg);
  void watchdogCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);

  void CAN_overriding();
  void MUJOCO_overriding();

  // ROS2 subscriber for pwm_val topic
  rclcpp::Subscription<allocator_interfaces::msg::PwmVal>::SharedPtr allocator_subscription_;
  rclcpp::Subscription<sbus_interfaces::msg::KillCmd>::SharedPtr killcmd_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr watchdog_subscription_;

  // ROS2 publisher for mujoco
  rclcpp::Publisher<mujoco_interfaces::msg::MotorThrust>::SharedPtr mujoco_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_dummy_zeros_timer_;
  rclcpp::TimerBase::SharedPtr can_transmission_timer_;

  // mode parameter ("real" or "sim")
  std::string mode_;

  int sock_ = 0;              // SocketCAN socket file descriptor.
  struct sockaddr_can addr_;  // CAN interface address structure.

  // Precomputed CAN frame for zero PWM (mapped to 16383 -> 0x3FFF)
  // High byte = 63 (0x3F), Low byte = 255 (0xFF)
  inline static constexpr struct can_frame frame_zeros_ = {
    0x123,    // can_id
    8,        // can_dlc (4 channels × 2 bytes)
    0,        // __pad (must be zero)
    0,        // __res0 (must be zero)
    0,        // __res1 
    {         // data payload: 0x3FFF → {0x3F, 0xFF} 반복
      63, 255,  // channel1 high/low
      63, 255,  // channel2
      63, 255,  // channel3
      63, 255   // channel4
    }
  };

  // Latest PWM values (stored by callback, sent by timer)
  struct can_frame pending_frame_ = frame_zeros_;
  std::mutex frame_mutex_;

  // --- MuJoCo Motor Model --- //

  // Coefficients for conversions
  const double LPF_alpha_ = 0.01; // Low-pass filter coefficient (0 < alpha_ < 1)
  const double LPF_beta_ = 1.0 - LPF_alpha_; // Low-pass filter coefficient
  const double PWM_alpha_ = 70.;
  const double PWM_beta_ = 8.;
  const double PWM_zeta_ = 0.03;

  // Buffer (FIFO) to store data for delayed output
  std::deque<DelayedData> data_buffer_;

  // Duration representing 1ms delay (1,000,000ns)
  rclcpp::Duration delay_{0, 1000000};

  // Latest PWM values
  double pwm1_ = 0.0; // [0, 1]
  double pwm2_ = 0.0; // [0, 1]
  double pwm3_ = 0.0; // [0, 1]
  double pwm4_ = 0.0; // [0, 1]
  
  // Latest Force values
  double f1_ = 0.0; // [N]
  double f2_ = 0.0; // [N]
  double f3_ = 0.0; // [N]
  double f4_ = 0.0; // [N]

  // Latest Moment values
  double m1_ = 0.0; // [Nm]
  double m2_ = 0.0; // [Nm]
  double m3_ = 0.0; // [Nm]
  double m4_ = 0.0; // [Nm]

  // Watchdog state
  uint16_t can_err_cnt = 0;
  uint8_t watchdog_state_ = 1; // default(normal) is 1.
  bool pwm_overriding_ = false;
};

#endif // TEENSY_WORKER_HPP
