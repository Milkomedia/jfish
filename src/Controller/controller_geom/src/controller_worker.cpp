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
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("sbus_signal", 1, std::bind(&ControllerNode::sbusCallback, this, std::placeholders::_1));
  optitrack_mea_subscription_ = this->create_subscription<mocap_interfaces::msg::MocapMeasured>("optitrack_mea", 1, std::bind(&ControllerNode::optitrackCallback, this, std::placeholders::_1));
  imu_mea_subscription_ = this->create_subscription<imu_interfaces::msg::ImuMeasured>("imu_mea", 1, std::bind(&ControllerNode::imuCallback, this, std::placeholders::_1));

  // Publishers
  controller_publisher_ = this->create_publisher<controller_interfaces::msg::ControllerOutput>("controller_output", 1);
  heartbeat_publisher_  = this->create_publisher<watchdog_interfaces::msg::NodeState>("controller_state", 1);
  debug_val_publisher_  = this->create_publisher<controller_interfaces::msg::ControllerDebugVal>("controller_info", 1);

  // Timers
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::debugging_timer_callback, this));

  command_->xd << 0.0, 0.0, -1.0;
  command_->b1d << 1.0, 0.0, 0.0;

  // main-tasking thread
  controller_thread_ = std::thread(&ControllerNode::controller_loop, this);

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
}

void ControllerNode::controller_timer_callback() {
  fdcl_controller_.position_control();
  fdcl_controller_.output_fM(f_out, M_out);

  //----------- Publsih -----------
  controller_interfaces::msg::ControllerOutput msg;
  msg.force = f_out;
  msg.moment = {M_out[0], M_out[1], M_out[2]};
  controller_publisher_->publish(msg);
}

void ControllerNode::sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg) {
  // save sbus_channel
  sbus_chnl_[0] = msg->ch[1];  // x command
  sbus_chnl_[1] = msg->ch[0];  // y command
  sbus_chnl_[2] = msg->ch[3];  // heading command
  sbus_chnl_[3] = msg->ch[2];  // z command
  sbus_chnl_[4] = msg->ch[9];  // kill command
  sbus_chnl_[5] = msg->ch[8];  // toggle E
  sbus_chnl_[6] = msg->ch[7];  // toggle G
  sbus_chnl_[7] = msg->ch[10]; // left-dial
  sbus_chnl_[8] = msg->ch[11]; // right-dial
  
  // remap SBUS data to double <pos x,y,z in [m]>
  ref_[0] = static_cast<double>(sbus_chnl_[0] - 1024) / 336.;  // [-2, 2] in m
  ref_[1] = static_cast<double>(sbus_chnl_[1] - 1024) / 336.;  // [-2, 2] in m
  ref_[2] = static_cast<double>(sbus_chnl_[3] -  352) / 896.; // [ 0, 1.5] in m (?)

  // remap SBUS data to double <yaw-heading in [rad]>
  double ref_yaw = static_cast<double>(sbus_chnl_[2] - 1024) / 672.;  // [-1, 1]
  ref_[3] += ref_yaw * 0.003;
  ref_[3] = fmod(ref_[3] + M_PI, two_PI);
  if (ref_[3] < 0) {ref_[3] += two_PI;}
  ref_[3] -= M_PI;
  
  command_->xd << ref_[0], ref_[1], -ref_[2];
  command_->xd_dot.setZero();
  command_->xd_2dot.setZero();
  command_->xd_3dot.setZero();
  command_->xd_4dot.setZero();

  command_->b1d << std::cos(ref_[3]), std::sin(ref_[3]), 0.0;
  command_->b1d_dot.setZero();
  command_->b1d_ddot.setZero();
}

void ControllerNode::optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg) {
  state_->x << msg->pos[0], msg->pos[1], msg->pos[2];
  state_->v << msg->vel[0], msg->vel[1], msg->vel[2];
  state_->a << msg->acc[0], msg->acc[1], msg->acc[2];
  
  x_[0] = msg->pos[0]; y_[0] = msg->pos[1]; z_[0] = msg->pos[2];
  x_[1] = msg->vel[0]; y_[1] = msg->vel[1]; z_[1] = msg->vel[2];
  x_[2] = msg->acc[0]; y_[2] = msg->acc[1]; z_[2] = msg->acc[2];

  state_->J << 0.058065042,  0.000104275,  -0.006291258,
               0.000104275,  0.069250561,  -0.000083225,
               -0.006291258, -0.000083225, 0.038730723;
}

void ControllerNode::imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg) {
  // quat -> rot matrix
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
  
  state_->R(0,0) = 1.0 - 2.0 * (yy + zz);
  state_->R(0,1) = 2.0 * (xy - wz);
  state_->R(0,2) = 2.0 * (xz + wy);
  state_->R(1,0) = 2.0 * (xy + wz);
  state_->R(1,1) = 1.0 - 2.0 * (xx + zz);
  state_->R(1,2) = 2.0 * (yz - wx);
  state_->R(2,0) = 2.0 * (xz - wy);
  state_->R(2,1) = 2.0 * (yz + wx);
  state_->R(2,2) = 1.0 - 2.0 * (xx + yy);

  // gyro
  state_->W << msg->w[0], msg->w[1], msg->w[2];
  roll_[1] = msg->w[0]; pitch_[1] = msg->w[1]; yaw_[1] = msg->w[2];

  // ZYX Tait–Bryan angles
  roll_[0]  = std::atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));
  pitch_[0] = std::asin (2.0*(w*y - z*x));
  yaw_[0]   = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
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
  for (int i = 0; i < 4; i++){gui_msg.pos_cmd[i] = ref_[i];}

  gui_msg.wrench_des[0] = f_out;
  gui_msg.wrench_des[1] = M_out[0];
  gui_msg.wrench_des[2] = M_out[1];
  gui_msg.wrench_des[3] = M_out[2];
  
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
