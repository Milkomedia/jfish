#include "allocator_worker.hpp"

AllocatorWorker::AllocatorWorker()
 : Node("allocator_node"),
  zeta_(Eigen::Vector4d( zeta, -zeta, zeta, -zeta )),
  q_B0_(Eigen::Vector4d( 0.25*M_PI, 0.75*M_PI, -0.75*M_PI, -0.25*M_PI )) {

  DH_params_ <<
    //   a      alpha     d   theta0
       0.120,   0.0,     0.0,  0.0,   // B->0
       0.134,   M_PI/2,  0.0,  0.0,   // 0->1
       0.115,   0.0,     0.0,  0.0,   // 1->2
       0.110,   0.0,     0.0,  0.0,   // 2->3
       0.024,   M_PI/2,  0.0,  0.0,   // 3->4
       0.068,   0.0,     0.0,  0.0;   // 4->5

  pwm_.setZero();
  f_.setZero();

  // Subscriber
  controller_subscriber_ = this->create_subscription<controller_interfaces::msg::ControllerOutput>("controller_output", 1, std::bind(&AllocatorWorker::controllerCallback, this, std::placeholders::_1));
  joint_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("joint_mea", 1, std::bind(&AllocatorWorker::jointValCallback, this, std::placeholders::_1));

  // Publishers
  pwm_publisher_ = this->create_publisher<allocator_interfaces::msg::PwmVal>("motor_cmd", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("allocator_state", 1);
  debug_val_publisher_ = this->create_publisher<allocator_interfaces::msg::AllocatorDebugVal>("allocator_info", 1);

  // Timers for periodic publishing
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
  Eigen::Vector4d Wrench; Eigen::Vector3d CoM;
  Wrench << msg->moment[0], msg->moment[1], msg->moment[2], msg->force;
  CoM << msg->com_bias[0], msg->com_bias[1], msg->com_bias[2];
  
  Eigen::Matrix<double,4,12> A1; A1.setZero();
  Eigen::Matrix<double,12,4> A2; A2.setZero();

  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  std::array<Eigen::Matrix4d,4> T_a;

  // FK for each arm
  for (int arm = 0; arm < 4; ++arm) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i <= 5; ++i) {
      const double q = (i == 0) ? q_B0_(arm) : arm_mea_[arm][i-1];
      T *= compute_DH(DH_params_(i,0), DH_params_(i,1), DH_params_(i,2), DH_params_(i,3) + q);
    }
    T_a[arm] = T;

    const Eigen::Vector3d r = T.block<3,1>(0,3) - CoM;
    const Eigen::Matrix3d S = skew(r);
    const Eigen::Matrix3d M = S + zeta_(arm) * I3;

    A1.block<3,3>(0, 3*arm) = M;
    A1(3, 3*arm + 2) = 1.0;  // z-component to sum Fz
    A2.block<3,1>(3*arm, arm) = T.block<3,1>(0,0);
  }

  // get thrust
  const Eigen::Matrix4d A = A1 * A2;
  Eigen::FullPivLU<Eigen::Matrix4d> lu(A);
  if (lu.isInvertible()) {f_ = lu.solve(Wrench);}
  else {f_ = (A.transpose()*A + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A.transpose()*Wrench);}

  // thrust -> pwm
  for (int i = 0; i < 4; ++i) {
    const double val = std::max(0.0, (f_(i) - pwm_beta_) / pwm_alpha_);
    pwm_(i) = std::sqrt(val);
    pwm_(i) = std::clamp(pwm_(i), 0.0, 1.0);
  }

  // send to teensy
  auto pwm_msg = allocator_interfaces::msg::PwmVal();
  pwm_msg.pwm1 = pwm_(0);
  pwm_msg.pwm2 = pwm_(1);
  pwm_msg.pwm3 = pwm_(2);
  pwm_msg.pwm4 = pwm_(3);
  pwm_publisher_->publish(pwm_msg);
}

void AllocatorWorker::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg)  {
  for (uint8_t i = 0; i < 5; ++i) 
  {
    arm_mea_[0][i] = msg->a1_mea[i];   // Arm 1
    arm_mea_[1][i] = msg->a2_mea[i];   // Arm 2
    arm_mea_[2][i] = msg->a3_mea[i];   // Arm 3
    arm_mea_[3][i] = msg->a4_mea[i];   // Arm 4

    arm_des_[0][i] = msg->a1_des[i];   // Arm 1
    arm_des_[1][i] = msg->a2_des[i];   // Arm 2
    arm_des_[2][i] = msg->a3_des[i];   // Arm 3
    arm_des_[3][i] = msg->a4_des[i];   // Arm 4
  }
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
    info_msg.pwm[i] = pwm_(i);
    info_msg.thrust[i] = f_(i);
  }

  for (size_t i = 0; i < 5; ++i) 
  {
    info_msg.a1_des[i] = arm_des_[0][i];
    info_msg.a2_des[i] = arm_des_[1][i];
    info_msg.a3_des[i] = arm_des_[2][i];
    info_msg.a4_des[i] = arm_des_[3][i];

    info_msg.a1_mea[i] = arm_mea_[0][i];
    info_msg.a2_mea[i] = arm_mea_[1][i];
    info_msg.a3_mea[i] = arm_mea_[2][i];
    info_msg.a4_mea[i] = arm_mea_[3][i];
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