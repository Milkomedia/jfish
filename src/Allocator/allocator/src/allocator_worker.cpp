#include "allocator_worker.hpp"

AllocatorWorker::AllocatorWorker() : Node("allocator_node") {

  // u = A * f
  const double l_1 = 0.4130990258;  // Arm length
  const double l_2 = 0.4130990258;
  const double C_m = 0.21496;
  A = (Matrix4d() << l_1,  l_1,  -l_1, -l_1,
                     -l_2, l_2,  l_2,  -l_2,
                      C_m, -C_m,  C_m, -C_m,
                      1.0,  1.0,  1.0,  1.0).finished();
  A_inv = A.inverse();

  // F = a * pwm^2 + b
  pwm_alpha_ = 46.5435;
  pwm_beta_ = 8.6111;

  // Subscriber
  controller_subscriber_ = this->create_subscription<controller_interfaces::msg::ControllerOutput>("controller_output", 1, std::bind(&AllocatorWorker::controllerCallback, this, std::placeholders::_1));
  joint_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("joint_mea", 1, std::bind(&AllocatorWorker::jointValCallback, this, std::placeholders::_1));

  // Publishers
  pwm_publisher_ = this->create_publisher<allocator_interfaces::msg::PwmVal>("motor_cmd", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("allocator_state", 1);
  debug_val_publisher_ = this->create_publisher<allocator_interfaces::msg::AllocatorDebugVal>("allocator_info", 1);

  // Timers for periodic publishing
  pwm_timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&AllocatorWorker::publishPwmVal, this));
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AllocatorWorker::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AllocatorWorker::debugging_timer_callback, this));

  // Initialize times
  const double nominal_dt = 1.0 / filtered_frequency_;  
  dt_buffer_.resize(buffer_size_, nominal_dt);
  dt_sum_ = nominal_dt * buffer_size_;
  last_callback_time_ = this->now();

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
}

void AllocatorWorker::controllerCallback(const controller_interfaces::msg::ControllerOutput::SharedPtr msg) {

  //-------- Loop Time Calculate (Moving avg filter) --------
  rclcpp::Time current_callback_time = this->now();
  double dt = (current_callback_time - last_callback_time_).seconds();
  last_callback_time_ = current_callback_time;  
  dt_sum_ = dt_sum_ - dt_buffer_[buffer_index_] + dt;
  dt_buffer_[buffer_index_] = dt;
  buffer_index_ = (buffer_index_ + 1) % buffer_size_;
  double avg_dt = dt_sum_ / static_cast<double>(buffer_size_);
  filtered_frequency_ = 1.0 / avg_dt;

  // get [Mx My Mz F]
  f << msg->moment[0], msg->moment[1], msg->moment[2], msg->force;

  // compute f [f1 f2 f3 f4] in [N]
  u = A_inv * f;
  u = u.cwiseMax(8.7).cwiseMin(145.1); // this clamps to [8.7, 145.1]

  // resolve f[N] to pwm[0.0~1.0]
  pwm = ((u.array() - pwm_beta_).array() / pwm_alpha_).cwiseSqrt().matrix();
  pwm = pwm.cwiseMax(0.0).cwiseMin(1.0); // this clamps to [0, 1]

  // RCLCPP_INFO(this->get_logger(), "Clamped PWM: [f1: %.2f, f2: %.2f, f3: %.2f, f4: %.2f]", pwm[0], pwm[1], pwm[2], pwm[3]);
}

void AllocatorWorker::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg) {
  for (uint8_t i = 0; i < 5; ++i) {
    arm_mea[0][i] = msg->a1_mea[i];   // Arm 1
    arm_mea[1][i] = msg->a2_mea[i];   // Arm 2
    arm_mea[2][i] = msg->a3_mea[i];   // Arm 3
    arm_mea[3][i] = msg->a4_mea[i];   // Arm 4

    arm_des[0][i] = msg->a1_des[i];   // Arm 1
    arm_des[1][i] = msg->a2_des[i];   // Arm 2
    arm_des[2][i] = msg->a3_des[i];   // Arm 3
    arm_des[3][i] = msg->a4_des[i];   // Arm 4
  }
}

void AllocatorWorker::publishPwmVal() {
  auto pwm_msg = allocator_interfaces::msg::PwmVal();
  // this range should be [0, 1]
  pwm_msg.pwm1 = pwm[0];
  pwm_msg.pwm2 = pwm[1];
  pwm_msg.pwm3 = pwm[2];
  pwm_msg.pwm4 = pwm[3];
  pwm_publisher_->publish(pwm_msg);
}

void AllocatorWorker::heartbeat_timer_callback() {
  // gate until handshake done
  if (!hb_enabled_) {return;}

  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = hb_state_;
  heartbeat_publisher_->publish(state_msg);

  // uint8 overflow wraps automatically
  hb_state_ = static_cast<uint8_t>(hb_state_ + 1);
}

void AllocatorWorker::debugging_timer_callback() {
  // Populate the debugging message
  allocator_interfaces::msg::AllocatorDebugVal info_msg;
  for (int i = 0; i < 4; i++) {
    info_msg.pwm[i] = pwm[i];
    info_msg.thrust[i] = u[i];
  }

  for (size_t i = 0; i < 5; ++i) {
    info_msg.a1_des[i] = arm_des[0][i];
    info_msg.a2_des[i] = arm_des[1][i];
    info_msg.a3_des[i] = arm_des[2][i];
    info_msg.a4_des[i] = arm_des[3][i];

    info_msg.a1_mea[i] = arm_mea[0][i];
    info_msg.a2_mea[i] = arm_mea[1][i];
    info_msg.a3_mea[i] = arm_mea[2][i];
    info_msg.a4_mea[i] = arm_mea[3][i];
  }

  info_msg.loop_rate = filtered_frequency_;

  // Publish
  debug_val_publisher_->publish(info_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AllocatorWorker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
