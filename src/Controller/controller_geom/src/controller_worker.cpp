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
  mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MujocoState>("/mujoco_state", 1, std::bind(&ControllerNode::mujocoCallback, this, std::placeholders::_1));
 
  // Publishers
  controller_publisher_ = this->create_publisher<controller_interfaces::msg::ControllerOutput>("/controller_output", 1);
  heartbeat_publisher_  = this->create_publisher<watchdog_interfaces::msg::NodeState>("/controller_state", 1);
  debug_val_publisher_  = this->create_publisher<controller_interfaces::msg::ControllerDebugVal>("/controller_info", 1);

  // Timers
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::debugging_timer_callback, this));

  command_->xd << 0.0, 0.0, 0.0;  // I don't know why,,, but
  command_->b1d << 1.0, 0.0, 0.0; // without this init, drone crashes.

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(shared_from_this());

  while (rclcpp::ok() && !define_initial_yaw()) {
    exec.spin_some();                                          // 콜백 처리
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  RCLCPP_INFO(this->get_logger(), "INIT YAW %f", inital_yaw_bias_);

  // main-tasking thread
  controller_thread_ = std::thread(&ControllerNode::controller_loop, this);

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;

  last_r_log_time_ = this->now();
}

bool ControllerNode::define_initial_yaw() {
  if (yaw_[0] == -0.12321) {
    return false;
  }
  else {
    inital_yaw_bias_ = yaw_[0];
    
    const double c = std::cos(-inital_yaw_bias_);
    const double s = std::sin(-inital_yaw_bias_);
    R_yaw_bias_ << c,  -s,  0.0,
                   s,   c,  0.0,
                   0.0,  0.0, 1.0;

    return true;
  }
}

void ControllerNode::controller_timer_callback() {
  fdcl_controller_.position_control();
  fdcl_controller_.output_fM(f_out, M_out);

  //----------- Publsih -----------
  controller_interfaces::msg::ControllerOutput msg;
  msg.force = f_out;
  msg.moment = {M_out[0], -M_out[1], -M_out[2]};
  controller_publisher_->publish(msg);

  // Vector3 X = Vector3::Zero();
  // fdcl_controller_.output_debug(X);
  // RCLCPP_INFO(this->get_logger(),
  //             "[x=%.6f, y=%.6f, z=%.6f]",
  //             X(0), X(1), X(2));
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
  
  // remap SBUS data to double <pos x,y,z in [m]>
  ref_[0] = static_cast<double>(1024 - sbus_chnl_[0]) * mapping_factor_xy;  // [m]
  ref_[1] = static_cast<double>(1024 - sbus_chnl_[1]) * mapping_factor_xy;  // [m]
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
  // quat -> R
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
  
  state_->R = R_yaw_bias_ * R;

  // Throttle printing at 10 Hz
  static rclcpp::Time last_print_time = this->now();
  auto now = this->now();
  // 100 ms 이상 경과했을 때만 출력
  if ((now - last_print_time).nanoseconds() > static_cast<int64_t>(100e6)) {
    last_print_time = now;

    // Format R matrix with fixed-point, 2 decimals
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Rotation matrix R:\n"
        << "[" << state_->R(0,0) << " " << state_->R(0,1) << " " << state_->R(0,2) << "]\n"
        << "[" << state_->R(1,0) << " " << state_->R(1,1) << " " << state_->R(1,2) << "]\n"
        << "[" << state_->R(2,0) << " " << state_->R(2,1) << " " << state_->R(2,2) << "]";

    // Print via ROS2_INFO
    RCLCPP_INFO(this->get_logger(), "\n%s", oss.str().c_str());
  }

  // gyro (copy to controller-state && gui-sending variable)
  state_->W << msg->w[0], -msg->w[1], -msg->w[2];
  roll_[1] = msg->w[0]; pitch_[1] = msg->w[1]; yaw_[1] = msg->w[2];

  // ZYX Tait–Bryan angles
  roll_[0]  = std::atan2(2.0*(wx + yz), 1.0 - 2.0*(xx + yy));
  pitch_[0] = std::asin (2.0*(wy - xz));
  yaw_[0]   = std::atan2(2.0*(wz + xy), 1.0 - 2.0*(yy + zz)) - inital_yaw_bias_;

  // RCLCPP_INFO(this->get_logger(), "gyro x : %.4f rad/s", msg->w[0]);
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

  gui_msg.wrench_des[0] = f_out;
  gui_msg.wrench_des[1] = M_out[0];
  gui_msg.wrench_des[2] = -M_out[1];
  gui_msg.wrench_des[3] = -M_out[2];
  
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
