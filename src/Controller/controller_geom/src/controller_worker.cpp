#include "controller_worker.hpp"

#include <iostream>
#include <vector>

ControllerNode::ControllerNode()
 : Node("controller_node"),
   state_(new fdcl::state_t()),
   command_(new fdcl::command_t()),
   thread_running_(true),
   fdcl_controller_(state_, command_)
{
  // Subscriptions
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("/sbus_signal", 1, std::bind(&ControllerNode::sbusCallback, this, std::placeholders::_1));
  optitrack_mea_subscription_ = this->create_subscription<mocap_interfaces::msg::MocapMeasured>("/optitrack_mea", 1, std::bind(&ControllerNode::optitrackCallback, this, std::placeholders::_1));
  imu_mea_subscription_ = this->create_subscription<imu_interfaces::msg::ImuMeasured>("/imu_mea", 1, std::bind(&ControllerNode::imuCallback, this, std::placeholders::_1));
  // mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MujocoState>("/mujoco_state", 1, std::bind(&ControllerNode::mujocoCallback, this, std::placeholders::_1));
 
  // Publishers
  controller_publisher_ = this->create_publisher<controller_interfaces::msg::ControllerOutput>("/controller_output", 1);
  heartbeat_publisher_  = this->create_publisher<watchdog_interfaces::msg::NodeState>("/controller_state", 1);
  debug_val_publisher_  = this->create_publisher<controller_interfaces::msg::ControllerDebugVal>("/controller_info", 1);

  // Timers
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::debugging_timer_callback, this));

  command_->xd << 0.0, 0.0, 0.0;  // I don't know why,,, but
  command_->b1d << 1.0, 0.0, 0.0; // without this init, drone crashes.

  state_->J << 0.3, 0.0006, 0.0006,
             0.0006,  0.3, 0.0006,
             0.0006, 0.0006, 0.5318;

  // main-tasking thread starts
  controller_thread_ = std::thread(&ControllerNode::controller_loop, this);

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
}

void ControllerNode::controller_timer_callback() {
  double f_out_geom;
  Vector3 M_out_geom;
  
  fdcl_controller_.position_control();
  fdcl_controller_.output_fM(f_out_geom, M_out_geom);

  tau_tilde_star_ = M_out_geom; // uses prev d_hat

  if (estimator_state_ == 0) { // conventional
    F_out_pub_ = f_out_geom;
    M_out_pub_ = M_out_geom;
  }
  else if (estimator_state_ == 1) { // dob apply
    F_out_pub_ = f_out_geom;
    tau_tilde_star_ = tau_tilde_star_ - d_hat_;
    M_out_pub_ = tau_tilde_star_;
  }
  else if (estimator_state_ == 2){ // com estimator apply
    /*
    Eigen::Vector3d acc = state_->a - g_ * state_->R * e_3_;
    Eigen::Matrix3d skew_acc;
    skew_acc << 0.0,      -acc.z(),  acc.y(),
                acc.z(),  0.0,       -acc.x(),
              -acc.y(),  acc.x(),   0.0;
    
    Eigen::Vector3d F_star(0.0, 0.0, f_out_geom); << no f_out_geom
    Eigen::Matrix3d skew_F_star;
    skew_F_star <<  0.0,         -F_star.z(),   F_star.y(),
                    F_star.z(),          0.0,  -F_star.x(),
                  -F_star.y(),   F_star.x(),          0.0;
                  
    Eigen::Matrix3d A_hat = m_bar_*J_bar_inv*skew_acc;
    filtered_A_hat_ = Qfilter_Alpha_*filtered_A_hat_ + Qfilter_Beta_*A_hat;
    Eigen::Vector3d d2 = k_bar_*J_bar_inv*skew_F_star*Pc_hat_;
    Eigen::Vector3d Pc_hat_dot = gamma_*filtered_A_hat_.transpose()*(d_hat + d2);
    Pc_hat_ += Pc_hat_dot; // 1/s
    Pc_hat_ = (Pc_hat_.cwiseMax(Eigen::Vector3d::Constant(-0.1))).cwiseMin(Eigen::Vector3d::Constant(0.1)); // saturation

    Eigen::Vector3d M_out_com_applied = state_->J * Omega_dot_star_tilde;
    M_out_pub_ = M_out_com_applied;
    */

    F_out_pub_ = f_out_geom;
    M_out_pub_ = M_out_geom; // asik anham
  }
  else { // hoxy mola
    F_out_pub_ = f_out_geom;
    M_out_pub_ = M_out_geom;
    RCLCPP_WARN(this->get_logger(), "YOU BULL-SHIT");
  }

  // d hat update
  Eigen::Vector3d RPY(roll_[0], -pitch_[0], -yaw_[0]);
  d_hat_ = DoB_update(RPY, tau_tilde_star_);

  if (is_paused_){overriding_coeff_ -= turnoff_coeff_;} // pause
  else           {overriding_coeff_ += turnon_coeff_;} // resume
  overriding_coeff_ = std::clamp(overriding_coeff_, 0.0, 1.0);
  M_out_pub_ = overriding_coeff_ * M_out_pub_;
  F_out_pub_ = overriding_coeff_ * F_out_pub_;

  //----------- Publsih -----------
  controller_interfaces::msg::ControllerOutput msg;
  msg.force = F_out_pub_;
  msg.moment = {M_out_pub_[0], -M_out_pub_[1], -M_out_pub_[2]};
  msg.d_hat = {d_hat_[0], -d_hat_[1], -d_hat_[2]};
    // msg.p_com = {Pc_hat_[0], -Pc_hat_[1], -Pc_hat_[2]};
  msg.p_com = {0, 0, 0};
  controller_publisher_->publish(msg);
  // RCLCPP_INFO(this->get_logger(), "[x=%.4f, y=%.4f, z=%.4f]", prev_d_hat_[0], -prev_d_hat_[1], -prev_d_hat_[2]);
}

void ControllerNode::sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg) {
  // save sbus_channel
  sbus_chnl_[0] = msg->ch[0];  // y command
  sbus_chnl_[1] = msg->ch[1];  // x command
  sbus_chnl_[2] = msg->ch[3];  // heading command
  sbus_chnl_[3] = msg->ch[2];  // z command
  sbus_chnl_[4] = msg->ch[9];  // kill command
  sbus_chnl_[5] = msg->ch[8];  // toggle E
  sbus_chnl_[6] = msg->ch[7];  // toggle G
  sbus_chnl_[7] = msg->ch[10]; // left-dial
  sbus_chnl_[8] = msg->ch[11]; // right-dial
  sbus_chnl_[9] = msg->ch[5];  // paddle
  
  // remap SBUS data to double <pos x,y,z in [m]>
  ref_[0] = static_cast<double>(1024 - sbus_chnl_[0]) * mapping_factor_xy;  // [m]
  ref_[1] = static_cast<double>(sbus_chnl_[1] - 1024) * mapping_factor_xy;  // [m]
  ref_[2] = static_cast<double>(352 - sbus_chnl_[3])  * mapping_factor_z; // [m]

  // remap SBUS data to double <yaw-heading in [rad]>
  double delta_yaw = (sbus_chnl_[2] < 1018 || sbus_chnl_[2] > 1030) 
    ? static_cast<double>(sbus_chnl_[2] - 1024) * mapping_factor_yaw 
    : 0.0;
  ref_[3] += delta_yaw;
  ref_[3] = fmod(ref_[3] + M_PI, two_PI);
  if (ref_[3] < 0) {ref_[3] += two_PI;}
  ref_[3] -= M_PI;
  
  command_->xd << ref_[0], ref_[1], ref_[2];
  command_->xd_dot.setZero();
  command_->xd_2dot.setZero();
  command_->xd_3dot.setZero();
  command_->xd_4dot.setZero();

  command_->b1d << std::cos(ref_[3]), std::sin(ref_[3]), 0.0;
  command_->b1d_dot.setZero();
  command_->b1d_ddot.setZero();

  if      (sbus_chnl_[6]==352){estimator_state_ = 0;}  // conventional
  else if(sbus_chnl_[6]==1024){estimator_state_ = 1;}  // dob
  else if(sbus_chnl_[6]==1696){estimator_state_ = 2;}  // com

  if      (sbus_chnl_[9]==352){ // paddle normal
    if     (prev_paddle_state_ == 0){paddle_holding_cnt_ = 0;}
    else if(prev_paddle_state_ == 1){
      if     (paddle_holding_cnt_ <= minimum_holding_time_){paddle_holding_cnt_ = 0;}
      else if(paddle_holding_cnt_ >= maximum_holding_time_){paddle_holding_cnt_ = 0;}
      else   {
        is_paused_ =  !is_paused_;
        fdcl_controller_.integral_reset();
        if(is_paused_){RCLCPP_INFO(this->get_logger(), "-- [ PAUSED ] --");}
        else          {RCLCPP_INFO(this->get_logger(), "-- [ RESUME ] --");}
      }
    }
    prev_paddle_state_ = 0;
  }
  else if(sbus_chnl_[9]==1696){ // paddle pushed
    if     (prev_paddle_state_ == 0){paddle_holding_cnt_ = 1;}
    else if(prev_paddle_state_ == 1){
      if(paddle_holding_cnt_ >= maximum_holding_time_){paddle_holding_cnt_ = maximum_holding_time_;}
      else{paddle_holding_cnt_ += 1;}
    }
    prev_paddle_state_ = 1;
  }

  if(estimator_state_ != prev_estimator_state_){
    prev_estimator_state_ = estimator_state_;
    d_hat_ =  Eigen::Vector3d::Zero();
    if     (estimator_state_==0){RCLCPP_INFO(this->get_logger(), "control mode -> [Conventional]");}
    else if(estimator_state_==1){RCLCPP_INFO(this->get_logger(), "control mode -> [DOB]");}
    else if(estimator_state_==2){RCLCPP_INFO(this->get_logger(), "control mode -> [CoM estimating]");}
  }
}

void ControllerNode::optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg) {
  // for controller-variable
  state_->x << X_offset+msg->pos[0], Y_offset-msg->pos[1], Z_offset-msg->pos[2];
  state_->v << msg->vel[0], -msg->vel[1], -msg->vel[2];
  state_->a << msg->acc[0], -msg->acc[1], -msg->acc[2];
  // for debugging-variable
  x_[0] = msg->pos[0]-X_offset; y_[0] = msg->pos[1]-Y_offset; z_[0] = msg->pos[2]-Z_offset;
  x_[1] = msg->vel[0]; y_[1] = msg->vel[1]; z_[1] = msg->vel[2];
  x_[2] = msg->acc[0]; y_[2] = msg->acc[1]; z_[2] = msg->acc[2];  
}

void ControllerNode::imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg) {
  // quat(z-up) -> R(z-down)
  const double w = msg->q[0];
  const double x = msg->q[1];
  const double y = msg->q[2];
  const double z = msg->q[3];
  
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

  R(0,0) =  1.0 - 2.0 * (yy + zz);
  R(0,1) = -2.0 * (xy - wz);
  R(0,2) = -2.0 * (xz + wy);
  R(1,0) = -2.0 * (xy + wz);
  R(1,1) =  1.0 - 2.0 * (xx + zz);
  R(1,2) =  2.0 * (yz - wx);
  R(2,0) = -2.0 * (xz - wy);
  R(2,1) =  2.0 * (yz + wx);
  R(2,2) =  1.0 - 2.0 * (xx + yy);

  state_->R = R;

  // gyro (copy to controller-state && gui-sending variable)
  state_->W << msg->w[0], -msg->w[1], -msg->w[2];
  roll_[1] = msg->w[0]; pitch_[1] = msg->w[1]; yaw_[1] = msg->w[2];

  // ZYX Tait–Bryan angles
  roll_[0]  = std::atan2(2.0*(wx + yz), 1.0 - 2.0*(xx + yy));
  pitch_[0] = std::asin (2.0*(wy - xz));
  yaw_[0]   = std::atan2(2.0*(wz + xy), 1.0 - 2.0*(yy + zz));
}

Eigen::Vector3d ControllerNode::DoB_update(Eigen::Vector3d rpy,Eigen::Vector3d tau_tilde_star)
{
  // -------- 상수 설정 --------
  static const double DT = 0.0025;     // [s] 400 Hz
  static const double fc = 0.5;       // [Hz] Butterworth cutoff
  const double wc = 2.0 * M_PI * fc;  // ωc
  const double w2 = wc * wc;
  const double w3 = w2 * wc;

  // -------- MOI (대각 성분만 사용, 오프대각 무시) --------
  static const double Jx = 0.3;
  static const double Jy = 0.3;
  static const double Jz = 0.5318;

  // -------- 상태 (Block A: Q*s^2*J*q) --------
  static double x_r1=0.0, x_r2=0.0, x_r3=0.0; // roll
  static double x_p1=0.0, x_p2=0.0, x_p3=0.0; // pitch
  static double x_y1=0.0, x_y2=0.0, x_y3=0.0; // yaw

  // -------- 상태 (Block B: Q*tau_tilde) --------
  static double y_r1=0.0, y_r2=0.0, y_r3=0.0; // roll
  static double y_p1=0.0, y_p2=0.0, y_p3=0.0; // pitch
  static double y_y1=0.0, y_y2=0.0, y_y3=0.0; // yaw

  // ================= Roll =================
  double x_dot_r1 = -2.0*wc*x_r1 - 2.0*w2*x_r2 - w3*x_r3 + rpy.x();
  double x_dot_r2 = x_r1;
  double x_dot_r3 = x_r2;
  x_r1 += x_dot_r1 * DT;
  x_r2 += x_dot_r2 * DT;
  x_r3 += x_dot_r3 * DT;
  double tau_hat_r = Jx * (w3 * x_r1);

  double y_dot_r1 = -2.0*wc*y_r1 - 2.0*w2*y_r2 - w3*y_r3 + tau_tilde_star.x();
  double y_dot_r2 = y_r1;
  double y_dot_r3 = y_r2;
  y_r1 += y_dot_r1 * DT;
  y_r2 += y_dot_r2 * DT;
  y_r3 += y_dot_r3 * DT;
  double Qtau_r = w3 * y_r3;

  double dhat_r = tau_hat_r - Qtau_r;

  // ================= Pitch =================
  double x_dot_p1 = -2.0*wc*x_p1 - 2.0*w2*x_p2 - w3*x_p3 + rpy.y();
  double x_dot_p2 = x_p1;
  double x_dot_p3 = x_p2;
  x_p1 += x_dot_p1 * DT;
  x_p2 += x_dot_p2 * DT;
  x_p3 += x_dot_p3 * DT;
  double tau_hat_p = Jy * (w3 * x_p1);

  double y_dot_p1 = -2.0*wc*y_p1 - 2.0*w2*y_p2 - w3*y_p3 + tau_tilde_star.y();
  double y_dot_p2 = y_p1;
  double y_dot_p3 = y_p2;
  y_p1 += y_dot_p1 * DT;
  y_p2 += y_dot_p2 * DT;
  y_p3 += y_dot_p3 * DT;
  double Qtau_p = w3 * y_p3;

  double dhat_p = tau_hat_p - Qtau_p;

  // ================= Yaw =================
  double x_dot_y1 = -2.0*wc*x_y1 - 2.0*w2*x_y2 - w3*x_y3 + rpy.z();
  double x_dot_y2 = x_y1;
  double x_dot_y3 = x_y2;
  x_y1 += x_dot_y1 * DT;
  x_y2 += x_dot_y2 * DT;
  x_y3 += x_dot_y3 * DT;
  double tau_hat_y = Jz * (w3 * x_y1);

  double y_dot_y1 = -2.0*wc*y_y1 - 2.0*w2*y_y2 - w3*y_y3 + tau_tilde_star.z();
  double y_dot_y2 = y_y1;
  double y_dot_y3 = y_y2;
  y_y1 += y_dot_y1 * DT;
  y_y2 += y_dot_y2 * DT;
  y_y3 += y_dot_y3 * DT;
  double Qtau_y = w3 * y_y3;

  double dhat_y = tau_hat_y - Qtau_y;

  // -------- 최종 외란 추정 --------
  Eigen::Vector3d d_hat(dhat_r, dhat_p, dhat_y);
  d_hat = (d_hat.cwiseMax(Eigen::Vector3d::Constant(-5.0))).cwiseMin(Eigen::Vector3d::Constant(5.0)); // saturation
  return d_hat;
}

void ControllerNode::mujocoCallback(const mujoco_interfaces::msg::MujocoState::SharedPtr msg) {
  // Extract the 3×3 inertia matrix (row-major) from the incoming message
  const auto &in = msg->inertia;
  state_->J << in[0], in[1], in[2],
               in[3], in[4], in[5],
               in[6], in[7], in[8];
}

void ControllerNode::heartbeat_timer_callback() {
  // gate until handshake done
  if (!hb_enabled_) {return;}

  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = hb_state_;
  heartbeat_publisher_->publish(state_msg);

  // uint8 overflow wraps automatically
  hb_state_ = static_cast<uint8_t>(hb_state_ + 1);
}

void ControllerNode::debugging_timer_callback() {
  controller_interfaces::msg::ControllerDebugVal gui_msg;

  for (int i = 0; i < 9; i++) {gui_msg.sbus_chnl[i] = sbus_chnl_[i];}

  gui_msg.pos_cmd[0] = ref_[0]; // x
  gui_msg.pos_cmd[1] = -ref_[1]; // y
  gui_msg.pos_cmd[2] = -ref_[2]; // z
  gui_msg.pos_cmd[3] = -ref_[3]; // yaw

  gui_msg.wrench_des[0] = F_out_pub_;
  gui_msg.wrench_des[1] = M_out_pub_[0];
  gui_msg.wrench_des[2] = -M_out_pub_[1];
  gui_msg.wrench_des[3] = -M_out_pub_[2];
  
  for (int i = 0; i < 3; i++) {
    gui_msg.imu_roll[i]  = roll_[i];
    gui_msg.imu_pitch[i] = pitch_[i];
    gui_msg.imu_yaw[i]   = yaw_[i];
    gui_msg.opti_x[i]    = x_[i];
    gui_msg.opti_y[i]    = y_[i];
    gui_msg.opti_z[i]    = z_[i];
  }

  debug_val_publisher_->publish(gui_msg);
}

void ControllerNode::controller_loop() {
  constexpr auto period = std::chrono::microseconds(Loop_us);
  auto next_time = std::chrono::steady_clock::now() + period;

  while (rclcpp::ok() && thread_running_) {
    controller_timer_callback();
    std::this_thread::sleep_until(next_time);
    next_time += period;
  }
}

ControllerNode::~ControllerNode() {
  thread_running_ = false;
  if (controller_thread_.joinable()) {
    controller_thread_.join();
  }

  delete state_;
  delete command_;
}

// Main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
