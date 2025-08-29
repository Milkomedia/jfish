#include "allocator_worker.hpp"

AllocatorWorker::AllocatorWorker()
 : Node("allocator_node"),
   q_B0_(Eigen::Vector4d(0.25*M_PI, 0.75*M_PI, -0.75*M_PI, -0.25*M_PI)) {

  // -------- DH table --------
  DH_params_ <<
    //   a      alpha     d   theta0
       0.120,   0.0,     0.0,  0.0,   // B->0
       0.134,   M_PI/2,  0.0,  0.0,   // 0->1
       0.115,   0.0,     0.0,  0.0,   // 1->2
       0.110,   0.0,     0.0,  0.0,   // 2->3
       0.024,   M_PI/2,  0.0,  0.0,   // 3->4
       0.068,   0.0,     0.0,  0.0;   // 4->5

  // -------- Subscribers --------
  joint_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("/joint_mea", 1, std::bind(&AllocatorWorker::jointValCallback, this, std::placeholders::_1));

  // -------- Publishers --------
  pwm_publisher_ = this->create_publisher<allocator_interfaces::msg::PwmVal>("/motor_cmd", 1);
  tilt_angle_publisher_ = this->create_publisher<allocator_interfaces::msg::TiltAngleVal>("/tilt_cmd", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/allocator_state", 1);
  debug_val_publisher_ = this->create_publisher<allocator_interfaces::msg::AllocatorDebugVal>("/allocator_info", 1);

  // -------- Timers --------
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AllocatorWorker::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AllocatorWorker::debugging_timer_callback, this));

  // -------- Loop time avg --------
  const double nominal_dt = 1.0 / filtered_frequency_;  
  dt_buffer_.resize(buffer_size_, nominal_dt);
  dt_sum_ = nominal_dt * buffer_size_;
  last_callback_time_ = this->now();

  // initial heartbeat & flag
  hb_state_   = 42;
  hb_enabled_ = true;
  heartbeat_state_ = 0;
  allocator_run_ = false;
}

// This callback is turned on after [jointValCallback] is executed at least once.
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
  Eigen::Vector4d Wrench;
  Wrench << msg->moment[0], msg->moment[1], msg->moment[2], msg->force;
  Pc_ << msg->p_com[0], msg->p_com[1], msg->p_com[2];

  // yaw wrench conversion
  tauz_bar_ = lpf_alpha_*Wrench(2) + lpf_beta_*tauz_bar_;
  double tauz_r = Wrench(2) - tauz_bar_;
  double tauz_r_sat = std::clamp(tauz_r, tauz_min, tauz_max);
  double tauz_t = tauz_bar_ + tauz_r - tauz_r_sat;
  // RCLCPP_INFO(this->get_logger(), ">>%f\t%f\t%f\t%f<<", Wrench(2), tauz_r_sat, tauz_t, Wrench(2)-tauz_r_sat-tauz_t);
  
  Eigen::Vector4d B1(Wrench(0), Wrench(1), tauz_r_sat, Wrench(3));
  Eigen::Matrix4d A1 = calc_A1(C2_mea_);

  Eigen::FullPivLU<Eigen::Matrix4d> lu_1(A1);
  if (lu_1.isInvertible()) {C1_ = lu_1.solve(B1);}
  else {C1_ = (A1.transpose()*A1 + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A1.transpose()*B1);}

  Eigen::Vector4d B2(0.0, 0.0, tauz_t, 0.0);
  Eigen::Matrix4d A2 = calc_A2(C1_);

  Eigen::FullPivLU<Eigen::Matrix4d> lu_2(A2);
  if (lu_2.isInvertible()) {C2_des_ = lu_2.solve(B2);}
  else {C2_des_ = (A2.transpose()*A2 + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A2.transpose()*B2);}

  // for safety
  C2_des_ = C2_des_.cwiseMax(Eigen::Vector4d::Constant(-0.261799333)).cwiseMin(Eigen::Vector4d::Constant(0.261799333));

  // RCLCPP_INFO(this->get_logger(), ">>%f\t%f\t%f\t%f<<", C2_des_(0), C2_des_(1), C2_des_(2), C2_des_(3));

  // thrust -> pwm
  for (int i = 0; i < 4; ++i) {
    const double val = std::max(0.0, (C1_(i) - pwm_beta_) / pwm_alpha_);
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

  // send to armchanger
  auto tilt_msg = allocator_interfaces::msg::TiltAngleVal();
  tilt_msg.th1 = C2_des_(0);
  tilt_msg.th2 = C2_des_(1);
  tilt_msg.th3 = C2_des_(2);
  tilt_msg.th4 = C2_des_(3);
  tilt_angle_publisher_->publish(tilt_msg);
}


Eigen::Matrix4d AllocatorWorker::calc_A1(const Eigen::Vector4d& C2) {
  Eigen::Matrix4d A1;

  double s1 = std::sin(C2(0)); double s2 = std::sin(C2(1)); double s3 = std::sin(C2(2)); double s4 = std::sin(C2(3));
  double c1 = std::cos(C2(0)); double c2 = std::cos(C2(1)); double c3 = std::cos(C2(2)); double c4 = std::cos(C2(3));

  double r1x = r_mea_(0, 0); double r1y = r_mea_(1, 0); double r1z = r_mea_(2, 0);
  double r2x = r_mea_(0, 1); double r2y = r_mea_(1, 1); double r2z = r_mea_(2, 1);
  double r3x = r_mea_(0, 2); double r3y = r_mea_(1, 2); double r3z = r_mea_(2, 2);
  double r4x = r_mea_(0, 3); double r4y = r_mea_(1, 3); double r4z = r_mea_(2, 3);

  double pcx = Pc_(0); double pcy = Pc_(1); double pcz = Pc_(2);

  A1(0,0) = inv_sqrt2 * (zeta + r1z - pcz) * s1  +  (r1y - pcy) * c1;
  A1(0,1) = inv_sqrt2 * (-zeta - r2z + pcz) * s2 +  (r2y - pcy) * c2;
  A1(0,2) = inv_sqrt2 * (-zeta - r3z + pcz) * s3 +  (r3y - pcy) * c3;
  A1(0,3) = inv_sqrt2 * (zeta + r4z - pcz) * s4  +  (r4y - pcy) * c4;

  A1(1,0) = inv_sqrt2 * (-zeta + r1z - pcz) * s1 + (-r1x + pcx) * c1;
  A1(1,1) = inv_sqrt2 * (-zeta + r2z - pcz) * s2 + (-r2x + pcx) * c2;
  A1(1,2) = inv_sqrt2 * (zeta - r3z + pcz) * s3  + (-r3x + pcx) * c3;
  A1(1,3) = inv_sqrt2 * (zeta - r4z + pcz) * s4  + (-r4x + pcx) * c4;

  A1(2,0) = inv_sqrt2 * (pcx + pcy) * s1  + (zeta) * c1;
  A1(2,1) = inv_sqrt2 * (-pcx + pcy) * s2 - (zeta) * c2;
  A1(2,2) = inv_sqrt2 * (-pcx -pcy) * s3  + (zeta) * c3;
  A1(2,3) = inv_sqrt2 * (pcx  - pcy) * s4 - (zeta) * c4;

  A1(3,0) = c1;
  A1(3,1) = c2;
  A1(3,2) = c3;
  A1(3,3) = c4;

  return A1;
}

Eigen::Matrix4d AllocatorWorker::calc_A2(const Eigen::Vector4d& C1) {
  Eigen::Matrix4d A2;

  double f1 = C1(0); double f2 = C1(1); double f3 = C1(2); double f4 = C1(3);

  double r1x = r_mea_(0, 0); double r1y = r_mea_(1, 0);
  double r2x = r_mea_(0, 1); double r2y = r_mea_(1, 1);
  double r3x = r_mea_(0, 2); double r3y = r_mea_(1, 2);
  double r4x = r_mea_(0, 3); double r4y = r_mea_(1, 3);

  A2(0,0) =  inv_sqrt2 * f1;
  A2(0,1) =  inv_sqrt2 * f2;
  A2(0,2) = -inv_sqrt2 * f3;
  A2(0,3) = -inv_sqrt2 * f4;

  A2(1,0) = -inv_sqrt2 * f1;
  A2(1,1) =  inv_sqrt2 * f2;
  A2(1,2) =  inv_sqrt2 * f3;
  A2(1,3) = -inv_sqrt2 * f4;

  A2(2,0) = inv_sqrt2 * (-r1x - r1y) * f1;
  A2(2,1) = inv_sqrt2 * ( r2x - r2y) * f2;
  A2(2,2) = inv_sqrt2 * ( r3x + r3y) * f3;
  A2(2,3) = inv_sqrt2 * (-r4x + r4y) * f4;

  A2(3,0) = inv_sqrt2 * (-r1x - r1y) * f1;
  A2(3,1) = inv_sqrt2 * (-r2x + r2y) * f2;
  A2(3,2) = inv_sqrt2 * ( r3x + r3y) * f3;
  A2(3,3) = inv_sqrt2 * ( r4x - r4y) * f4;

  return A2;
}

void AllocatorWorker::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg)  {
  for (uint8_t i = 0; i < 5; ++i) {
    arm_mea_[0][i] = msg->a1_mea[i];   // Arm 1
    arm_mea_[1][i] = msg->a2_mea[i];   // Arm 2
    arm_mea_[2][i] = msg->a3_mea[i];   // Arm 3
    arm_mea_[3][i] = msg->a4_mea[i];   // Arm 4

    arm_des_[0][i] = msg->a1_des[i];   // Arm 1
    arm_des_[1][i] = msg->a2_des[i];   // Arm 2
    arm_des_[2][i] = msg->a3_des[i];   // Arm 3
    arm_des_[3][i] = msg->a4_des[i];   // Arm 4
  }

  // solve FK for each arm
  for (uint8_t arm = 0; arm < 4; ++arm) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (uint8_t i = 0; i <= 5; ++i) {
      const double q = (i == 0) ? q_B0_(arm) : arm_mea_[arm][i-1];
      T *= compute_DH(DH_params_(i,0), DH_params_(i,1), DH_params_(i,2), DH_params_(i,3) + q);
    }

    // save position vector
    r_mea_.col(arm) = T.block<3,1>(0,3);

    // save tilted angle
    const Eigen::Vector3d heading = T.block<3,3>(0,0).col(0);
    C2_mea_(arm) = std::asin(std::clamp(heading.head<2>().cwiseAbs().sum() * inv_sqrt2, -0.5, 0.5));
  }

  if (!allocator_run_) {
    allocator_run_ = true;
    start_allcation();
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
    info_msg.thrust[i] = C1_(i);
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

void AllocatorWorker::start_allcation() {
  controller_subscriber_ = this->create_subscription<controller_interfaces::msg::ControllerOutput>("/controller_output", 1, std::bind(&AllocatorWorker::controllerCallback, this, std::placeholders::_1));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AllocatorWorker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}