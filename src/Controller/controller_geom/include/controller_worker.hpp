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
#include "controller_interfaces/msg/controller_info.hpp"
#include "controller_interfaces/msg/controller_reference.hpp"
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

#define Loop_us 2500 // controller thread loop dt [us]

#define CMD_XY_MAX 1.0     // pos cmd range mapped to [-k, k]m (x,y)
#define CMD_Z_MAX  1.0     // pos cmd range mapped to [ 0, k]m (z)
#define CMD_YAW_SPD 3.0    // speed at which the yaw command is added.

constexpr double mapping_factor_xy = CMD_XY_MAX / 672.0;
constexpr double mapping_factor_z  = CMD_Z_MAX / 1344.0;
constexpr double mapping_factor_yaw  = CMD_YAW_SPD / 672000.0;
constexpr double two_PI = 2.0 * M_PI;

constexpr double X_offset = 0.0; // [m] 
constexpr double Y_offset = 0.6; // [m] 
constexpr double Z_offset = 0.0; // [m] it must be (+) sign.

static const double DT = 0.0025;    // [s] 400 Hz
static const double fc = 0.01;       // [Hz] Butterworth cutoff initial Hz = 0.3
const double wc = 2.0 * M_PI * fc;  // ωc
const double w2 = wc * wc;
const double w3 = w2 * wc;  
static const double Jx = 0.3;
static const double Jy = 0.3;
static const double Jz = 0.5318;

// workspace constrain
double x_min_ = -0.06; // [m]
double x_max_ =  0.06; // [m]
double y_min_ = -0.06; // [m]
double y_max_ =  0.06; // [m]

// init thrust
constexpr double PWM_alpha_ = 70.;
constexpr double PWM_beta_ = 8.;
const double LPF_alpha_ = 0.001;           // Low-pass filter coefficient
const double LPF_beta_ = 1.0 - LPF_alpha_; // Low-pass filter coefficient
double init_pwm_ = 0.14;                    // init goal PWM
double pwm_state_  = 0.0;

inline double pwm2thrust(double pwm) {
   return PWM_alpha_ * pwm * pwm + PWM_beta_;
 }

inline double thrust2pwm(double thrust) {
  double v = (thrust - PWM_beta_) / PWM_alpha_;
  if (v <= 0.0) return 0.0;
  return std::sqrt(v);
}
inline double pwm2total_thrust(double pwm) {
  return 4.0 * pwm2thrust(pwm);
}

static inline double map(double input, double in_min, double in_max, double out_min, double out_max) {
  return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  ~ControllerNode();

private:
  fdcl::state_t * state_;
  fdcl::command_t * command_;

  std::atomic<bool> thread_running_;
  std::atomic<double> F_cmd_pub_{0.0};
  std::thread controller_thread_;

  fdcl::control fdcl_controller_;

  void sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg);
  void imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg);
  void mujocoCallback(const mujoco_interfaces::msg::MujocoState::SharedPtr msg);
  void refCallback(const controller_interfaces::msg::ControllerReference::SharedPtr msg);
  void controller_timer_callback();
  void heartbeat_timer_callback();
  void debugging_timer_callback();
  void controller_loop();
  void pub_for_plot();
  Eigen::Vector2d DoB_update(Eigen::Vector3d rpy, Eigen::Vector2d tau_tilde_star);

  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<mocap_interfaces::msg::MocapMeasured>::SharedPtr optitrack_mea_subscription_;
  rclcpp::Subscription<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_mea_subscription_;
  rclcpp::Subscription<mujoco_interfaces::msg::MujocoState>::SharedPtr mujoco_subscription_;
  rclcpp::Subscription<controller_interfaces::msg::ControllerReference>::SharedPtr reference_subscription_;
  rclcpp::Publisher<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_publisher_;
  
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerDebugVal>::SharedPtr debug_val_publisher_;
  rclcpp::TimerBase::SharedPtr debugging_timer_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerInfo>::SharedPtr publisher_for_plot_;
  rclcpp::TimerBase::SharedPtr plot_timer_;

  // pause&resume variable
  bool is_paused_ = true; // false->resume(flight-available) / true->pause
  bool is_runup_ = true;  // false->not yet / true->thrust run up ready
  bool floor_after_resume_ = true; // RESUME 이후에도 하한 유지 여부
  uint8_t prev_paddle_state_ = 0; // 0->paddle normal / 1->paddle pushed
  uint8_t paddle_holding_cnt_ = 0;
  const uint8_t minimum_holding_time_ = 4;
  const uint8_t maximum_holding_time_ = 200;
  double overriding_coeff_ = 0.; // this must be in [0,1]
  const double turnon_coeff_  = Loop_us / 1000000. / 4.; // 4sec delay to resume
  const double turnoff_coeff_ = Loop_us / 1000000. / 5.; // 5sec delay to pause

  uint8_t estimator_state_ = 0; // 0->conventional / 1->dob / 2->com
  uint8_t prev_estimator_state_ = 0;
  uint8_t run_up_state_ = 0; // 0->thrust 0 / 1->thrust 0 /  2->run-up
  uint8_t prev_run_up_state_ = 0;

  // DOB state
  Eigen::Vector2d d_hat_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d tau_tilde_star_ = Eigen::Vector2d::Zero();

  // CoM estimate state
  Eigen::Vector2d Pc_hat_ = Eigen::Vector2d::Zero();
  double m_bar_ = 8.0;
  double gamma_ = 0.00004;
  double g_ =9.80665;

  double F_out_pub_ = 0.;
  Eigen::Vector3d M_out_pub_ = Eigen::Vector3d::Zero();

  // sbus state
  int   sbus_chnl_[10] = {1024, 1024, 352, 1024, 352, 352, 352, 352, 352, 352};
  double sbus_ref_[4] = {0.0, 0.0, 0.0, 0.0}; // x,y,z,yaw cmd [m, m, m, rad]
  double roll_[3] =  {0.0, 0.0, 0.0}; // imu r [rad, rad/s, rad/s^2]
  double pitch_[3] = {0.0, 0.0, 0.0}; // imu p [rad, rad/s, rad/s^2]
  double yaw_[3] =   {0.0, 0.0, 0.0}; // imu y [rad, rad/s, rad/s^2]
  double x_[3] = {0.0, 0.0, 0.0}; // opti x [m, m/s, m/s^2]
  double y_[3] = {0.0, 0.0, 0.0}; // opti y [m, m/s, m/s^2]
  double z_[3] = {0.0, 0.0, 0.0}; // opti z [m, m/s, m/s^2]

  // MPC state
  float mpc_ref_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x,y,z,heading [m, m, m, unit, unit, unit]

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag
};

#endif // CONTROLLER_WORKER_HPP
