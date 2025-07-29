#ifndef CONTROLLER_WORKER_HPP
#define CONTROLLER_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <algorithm>
#include <cstddef>
#include <Eigen/Dense>

#include "sbus_interfaces/msg/sbus_signal.hpp"
#include "controller_interfaces/msg/controller_output.hpp"
#include "controller_interfaces/msg/controller_debug_val.hpp"
#include "mocap_interfaces/msg/mocap_measured.hpp"
#include "imu_interfaces/msg/imu_measured.hpp"
#include "mujoco_interfaces/msg/mujoco_state.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"

#include "fdcl/control.hpp"
#include "controller_param.h"

//////////////////////////////////////////////////////
#include <sstream>
#include <iomanip>
///////////////////////////////////////////////

#define Loop_us 500 // controller thread loop dt [us]

#define CMD_XY_MAX 1.5     // pos cmd range mapped to [-k, k]m (x,y)
#define CMD_Z_MAX  1.5     // pos cmd range mapped to [ 0, k]m (z)
#define CMD_YAW_SPD 3.0    // speed at which the yaw command is added.

constexpr double mapping_factor_xy = CMD_XY_MAX / 672.0;
constexpr double mapping_factor_z  = CMD_Z_MAX / 1344.0;
constexpr double mapping_factor_yaw  = CMD_YAW_SPD / 672000.0;
constexpr double two_PI = 2.0 * M_PI;

constexpr double X_offset = 0.0; // [m] 
constexpr double Y_offset = 0.6; // [m] 
constexpr double Z_offset = 0.11; // [m] it must be (+) sign.

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

  bool define_initial_yaw();

  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<mocap_interfaces::msg::MocapMeasured>::SharedPtr optitrack_mea_subscription_;
  rclcpp::Subscription<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_mea_subscription_;
  rclcpp::Subscription<mujoco_interfaces::msg::MujocoState>::SharedPtr mujoco_subscription_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_publisher_;
  
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerDebugVal>::SharedPtr debug_val_publisher_;
  rclcpp::TimerBase::SharedPtr debugging_timer_;

  /////

  // init timer
  rclcpp::TimerBase::SharedPtr init_timer_;

  // flag for initial yaw
  bool initial_yaw_done_{false};

  // callback
  void init_timer_callback();

  /////

  // sbus state
  int   sbus_chnl_[9] = {1024, 1024, 352, 1024, 352, 352, 352, 352, 352};
  double ref_[4] = {0.0, 0.0, 0.0, 0.0}; // x,y,z,yaw cmd [m, m, m, rad]
  double roll_[3] =  {0.0, 0.0, 0.0}; // imu r [rad, rad/s, rad/s^2]
  double pitch_[3] = {0.0, 0.0, 0.0}; // imu p [rad, rad/s, rad/s^2]
  double yaw_[3] =   {0.0, 0.0, 0.0}; // imu y [rad, rad/s, rad/s^2]
  double x_[3] = {0.0, 0.0, 0.0}; // opti x [m, m/s, m/s^2]
  double y_[3] = {0.0, 0.0, 0.0}; // opti y [m, m/s, m/s^2]
  double z_[3] = {0.0, 0.0, 0.0}; // opti z [m, m/s, m/s^2]

  double inital_yaw_bias_ = 0.0;
  Eigen::Matrix3d R_yaw_bias_ = Eigen::Matrix3d::Identity();

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag
};

#endif // CONTROLLER_WORKER_HPP
