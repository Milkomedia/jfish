#ifndef CONTROLLER_WORKER_HPP
#define CONTROLLER_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <algorithm>
#include <cstddef>

#include "sbus_interfaces/msg/sbus_signal.hpp"
#include "controller_interfaces/msg/controller_output.hpp"
#include "controller_interfaces/msg/controller_debug_val.hpp"
#include "mocap_interfaces/msg/mocap_measured.hpp"
#include "imu_interfaces/msg/imu_measured.hpp"
#include "mujoco_interfaces/msg/mujoco_state.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"

#include "fdcl/control.hpp"
#include "controller_param.h"

#define Loop_us 500 // controller thread loop dt [us]

#define POS_CTRL_RANGE_XY 2.0 // pos cmd range mapped to [-k, k]m (x,y)
#define POS_CTRL_RANGE_Z  2.0 // pos cmd range mapped to [ 0, k]m (z)



constexpr double mapping_factor_xy = POS_CTRL_RANGE_XY / 672.0;
constexpr double mapping_factor_z  = POS_CTRL_RANGE_Z;
constexpr double mapping_factor_xy = 672.0 / POS_CTRL_RANGE_XY;


constexpr double two_PI = 2.0 * M_PI;

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  ~ControllerNode();

private:
  fdcl::state_t * state_;
  fdcl::command_t * command_;

  std::atomic<bool> thread_running_;
  std::thread controller_thread_;

  fdcl::control fdcl_controller_;

  double f_out;
  Vector3 M_out;

  void sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg);
  void imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg);
  void mujocoCallback(const mujoco_interfaces::msg::MujocoState::SharedPtr msg);
  void controller_timer_callback();
  void heartbeat_timer_callback();
  void debugging_timer_callback();
  void controller_loop();

  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<mocap_interfaces::msg::MocapMeasured>::SharedPtr optitrack_mea_subscription_;
  rclcpp::Subscription<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_mea_subscription_;
  rclcpp::Subscription<mujoco_interfaces::msg::MujocoState>::SharedPtr mujoco_subscription_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_publisher_;
  
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerDebugVal>::SharedPtr debug_val_publisher_;
  rclcpp::TimerBase::SharedPtr debugging_timer_;

  // sbus state
  int   sbus_chnl_[9] = {1024, 1024, 352, 1024, 352, 352, 352, 352, 352};
  double ref_[4] = {0.0, 0.0, 0.0, 0.0}; // x,y,z,yaw cmd [m, m, m, rad]
  double roll_[3] =  {0.0, 0.0, 0.0}; // imu r [rad, rad/s, rad/s^2]
  double pitch_[3] = {0.0, 0.0, 0.0}; // imu p [rad, rad/s, rad/s^2]
  double yaw_[3] =   {0.0, 0.0, 0.0}; // imu y [rad, rad/s, rad/s^2]
  double x_[3] = {0.0, 0.0, 0.0}; // opti x [m, m/s, m/s^2]
  double y_[3] = {0.0, 0.0, 0.0}; // opti y [m, m/s, m/s^2]
  double z_[3] = {0.0, 0.0, 0.0}; // opti z [m, m/s, m/s^2]

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag
};

#endif // CONTROLLER_WORKER_HPP
