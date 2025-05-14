#include "allocator_worker.hpp"

AllocatorWorker::AllocatorWorker() : Node("allocator_node") 
{
  DH_params.resize(6,4);
  DH_params << 
  //  a     alpha    d  theta
      A_B,  0,       0, 0,    // B -> 0
      A1,   M_PI/2,  0, 0,    // 0 -> 1
      A2,   0,       0, 0,    // 1 -> 2
      A3,   0,       0, 0,    // 2 -> 3
      A4,   M_PI/2,  0, 0,    // 3 -> 4
      A5,   0,       0, 0;    // 4 -> 5
  
  CoM << xc, yc, zc;

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
  W1 << msg->moment[0], msg->moment[1], msg->moment[2], msg->force; 
  // get Wrench_1={B} frame [Tau_rall Tau_pitch Tau_yaw Fz]^T --> Under-actuated system Allocation part
  
  for (int arm_number = 1; arm_number <= 4; arm_number++) 
  {
      Matrix4d TB5 = Matrix4d::Identity();
      Eigen::Map<Eigen::VectorXd> q(arm_mea[arm_number - 1], 5);
      for (size_t i = 0; i <= 5; i++) 
      {
          double a = DH_params(i, 0);
          double alpha = DH_params(i, 1);
          double d = DH_params(i, 2);
          double theta = i==0 ? DH_params(i, 3)+q_B0[arm_number - 1] : DH_params(i, 3)+q(i-1);
          
          Matrix4d T;
          T << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
                sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
                0,          sin(alpha),              cos(alpha),               d,
                0,          0,                        0,                       1;

          TB5 *= T;
      }

      if (arm_number == 1) Transformation_a1 = TB5;
      else if (arm_number == 2) Transformation_a2 = TB5;
      else if (arm_number == 3) Transformation_a3 = TB5;
      else Transformation_a4 = TB5;
  }

  Vector3d r1 = Transformation_a1.block<3,1>(0,3) - CoM;
  Vector3d r2 = Transformation_a2.block<3,1>(0,3) - CoM;
  Vector3d r3 = Transformation_a3.block<3,1>(0,3) - CoM;
  Vector3d r4 = Transformation_a4.block<3,1>(0,3) - CoM;

  Matrix3d skew1 = (Matrix3d() <<      0, -r1(2),  r1(1),               
                                    r1(2),      0, -r1(0),
                                  -r1(1),  r1(0),     0).finished();  

  Matrix3d skew2 = (Matrix3d() <<      0, -r2(2),  r2(1),               
                                    r2(2),      0, -r2(0),
                                  -r2(1),  r2(0),     0).finished();  

  Matrix3d skew3 = (Matrix3d() <<      0, -r3(2),  r3(1),               
                                    r3(2),      0, -r3(0),
                                  -r3(1),  r3(0),     0).finished();  

  Matrix3d skew4 = (Matrix3d() <<      0, -r4(2),  r4(1),               
                                    r4(2),      0, -r4(0),
                                  -r4(1),  r4(0),     0).finished();

  Matrix3d M1 = (skew1 + zeta * Matrix3d::Identity());
  Matrix3d M2 = (skew2 - zeta * Matrix3d::Identity());
  Matrix3d M3 = (skew3 + zeta * Matrix3d::Identity());
  Matrix3d M4 = (skew4 - zeta * Matrix3d::Identity());

  A_1.block<3,3>(0,0) = M1;
  A_1.block<3,3>(0,3) = M2;
  A_1.block<3,3>(0,6) = M3;
  A_1.block<3,3>(0,9) = M4;

  A_1(3,2)=1;
  A_1(3,5)=1;
  A_1(3,8)=1;
  A_1(3,11)=1;

  A_2.block<3,1>(0,0) = Transformation_a1.block<3,1>(0,0);
  A_2.block<3,1>(3,1) = Transformation_a2.block<3,1>(0,0);
  A_2.block<3,1>(6,2) = Transformation_a3.block<3,1>(0,0);
  A_2.block<3,1>(9,3) = Transformation_a4.block<3,1>(0,0);
  
  A = A_1*A_2;
  A_inv = A.inverse();
  
  f = A_inv * W1; // compute f [f1 f2 f3 f4] in [N]
  // pwm = f.array();
  // resolve f[N] to pwm[0.0~1.0]
  for (int i = 0; i < 4; ++i) 
  {
    if (f(i) > pwm_beta_) pwm(i) = std::sqrt((f(i) - pwm_beta_) / pwm_alpha_);
    else pwm(i) = 0.0; // safe fallback
    pwm(i) = std::max(0.0, std::min(1.0, pwm(i)));
  }
  // pwm = ((f.array() - pwm_beta_).array() / pwm_alpha_).cwiseSqrt().matrix();
  // pwm = pwm.cwiseMax(0.0).cwiseMin(1.0); // this clamps to [0, 1]
  
  // RCLCPP_INFO(this->get_logger(), "W1 = [%f %f %f %f]",W1(0), W1(1), W1(2), W1(3));
  // RCLCPP_INFO(this->get_logger(), "Clamped force: [f1: %.2f, f2: %.2f, f3: %.2f, f4: %.2f], %.2f", f[0], f[1], f[2], f[3], f[0]+f[1]+f[2]+f[3]);
  // RCLCPP_INFO(this->get_logger(), "Clamped PWM: [f1: %.2f, f2: %.2f, f3: %.2f, f4: %.2f]", pwm[0], pwm[1], pwm[2], pwm[3]);
  // pwm = {0,0,0,0};
}

void AllocatorWorker::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg) 
{
  for (uint8_t i = 0; i < 5; ++i) 
  {
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

void AllocatorWorker::publishPwmVal() 
{
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

void AllocatorWorker::debugging_timer_callback() 
{
  // Populate the debugging message
  allocator_interfaces::msg::AllocatorDebugVal info_msg;
  for (int i = 0; i < 4; i++) 
  {
    info_msg.pwm[i] = pwm[i];
    info_msg.thrust[i] = f[i];
  }

  for (size_t i = 0; i < 5; ++i) 
  {
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