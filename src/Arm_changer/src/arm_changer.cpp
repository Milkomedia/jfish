#include "arm_changer.hpp"

using namespace std::chrono_literals;

inline double map_value(double input, double in_min, double in_max, double out_min, double out_max) {
  return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ArmChangerWorker::ArmChangerWorker(): Node("arm_changing_node") {
  // ROS2 Subscribers
  controller_subscription_ = this->create_subscription<controller_interfaces::msg::EstimatorOutput>("/estimator_output", 1, std::bind(&ArmChangerWorker::estimator_callback, this, std::placeholders::_1));
  // sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("/sbus_signal", 1, std::bind(&ArmChangerWorker::sbus_callback, this, std::placeholders::_1));
  killcmd_subscription_ = this->create_subscription<sbus_interfaces::msg::KillCmd>("sbus_kill", 1, std::bind(&ArmChangerWorker::killCmd_callback, this, std::placeholders::_1));
  tilt_angle_val_subscription_ = this->create_subscription<allocator_interfaces::msg::TiltAngleVal>("tilt_cmd", 1, std::bind(&ArmChangerWorker::TiltAngle_callback, this, std::placeholders::_1));

  // ROS2 Publisher
  joint_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("/joint_cmd", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/armchanger_state", 1);

  // ROS2 Timer
  joint_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmChangerWorker::joint_callback, this));
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArmChangerWorker::heartbeat_timer_callback, this));

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
}

void ArmChangerWorker::sbus_callback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg) {
  double delta_x_manual = map_value(static_cast<double>(msg->ch[10]), 352, 1696, x_min_, x_max_);
  double delta_y_manual = map_value(static_cast<double>(msg->ch[11]), 352, 1696, y_min_, y_max_);
  RCLCPP_INFO(this->get_logger(), "x: %.4f, y: %.4f", delta_x_manual, delta_y_manual);  
  
  //heading angle [rad]
  Eigen::Vector3d heading1(0.0, std::sin(C2_(0)), std::sqrt(1-std::sin(C2_(0))*std::sin(C2_(0)))); // arm1
  Eigen::Vector3d heading2(0.0, std::sin(C2_(1)), std::sqrt(1-std::sin(C2_(1))*std::sin(C2_(1)))); // arm2
  Eigen::Vector3d heading3(0.0, std::sin(C2_(2)), std::sqrt(1-std::sin(C2_(2))*std::sin(C2_(2)))); // arm3
  Eigen::Vector3d heading4(0.0, std::sin(C2_(3)), std::sqrt(1-std::sin(C2_(3))*std::sin(C2_(3)))); // arm4

  //arm postion [mm]
  Eigen::Vector3d arm_position1(-280.0 + half_sqrt2*(-delta_x_manual-delta_y_manual),   half_sqrt2*(-delta_x_manual+delta_y_manual),  150.0); //arm1
  Eigen::Vector3d arm_position2(-280.0 + half_sqrt2*(-delta_x_manual+delta_y_manual),   half_sqrt2*(+delta_x_manual+delta_y_manual),  150.0); //arm1
  Eigen::Vector3d arm_position3(-280.0 + half_sqrt2*(+delta_x_manual+delta_y_manual),   half_sqrt2*(+delta_x_manual-delta_y_manual),  150.0); //arm1
  Eigen::Vector3d arm_position4(-280.0 + half_sqrt2*(+delta_x_manual-delta_y_manual),   half_sqrt2*(-delta_x_manual-delta_y_manual),  150.0); //arm1

  //Base(J1) 2 Body(base)
  auto [arm_position1_body, heading1_body] = arm2base(arm_position1, heading1, 1);
  auto [arm_position2_body, heading2_body] = arm2base(arm_position2, heading2, 2);
  auto [arm_position3_body, heading3_body] = arm2base(arm_position3, heading3, 3);
  auto [arm_position4_body, heading4_body] = arm2base(arm_position4, heading4, 4);

  //Collision check (Body frame) 
  if (!collision_check(arm_position1_body, arm_position2_body, arm_position3_body, arm_position4_body)) {
  hb_enabled_ = false;
  RCLCPP_WARN(this->get_logger(), "Collision detected: discs overlap → heartbeat disabled!");
  return;
  }

  // path velocity check
  auto now_t = this->now();

  std::array<Eigen::Vector3d,4> cur_pos{arm_position1_body, arm_position2_body, arm_position3_body, arm_position4_body};

  for (int i = 0; i < 4; ++i) 
  {
    if (!has_last_des_[i]) 
    {
      last_des_pos_[i]  = cur_pos[i];
      last_des_time_[i] = now_t;
      has_last_des_[i]  = true;
    } 
    else 
    {
      double dt = (now_t - last_des_time_[i]).seconds();
      
      if (!path_check(last_des_pos_[i], cur_pos[i], dt)) 
      {
        hb_enabled_ = false;
        RCLCPP_WARN(this->get_logger(),"Path check failed (arm %d): speed limit exceeded → heartbeat disabled", i+1);
        return;
      }
      
      last_des_pos_[i]  = cur_pos[i];
      last_des_time_[i] = now_t;
    }
  }
  

  auto a1_radians = compute_ik(arm_position1, heading1); //[rad rad rad rad rad]
  auto a2_radians = compute_ik(arm_position2, heading2); //[rad rad rad rad rad]
  auto a3_radians = compute_ik(arm_position3, heading3); //[rad rad rad rad rad]
  auto a4_radians = compute_ik(arm_position4, heading4); //[rad rad rad rad rad]

  //IK check
  if (!ik_check(a1_radians, arm_position1, heading1) || !ik_check(a2_radians, arm_position2, heading2) || !ik_check(a3_radians, arm_position3, heading3) || !ik_check(a4_radians, arm_position4, heading4)) {
      // hb_enabled_ = false;
      // RCLCPP_WARN(this->get_logger(), "IK check failed, heartbeat disabled!");
      // return;
  }

  auto joint_msg = dynamixel_interfaces::msg::JointVal();
  joint_msg.a1_des = a1_radians;
  joint_msg.a2_des = a2_radians;
  joint_msg.a3_des = a3_radians;
  joint_msg.a4_des = a4_radians;

  joint_publisher_->publish(joint_msg);

  std::memcpy(a1_q_.data(), a1_radians.data(), a1_radians.size()*sizeof(double));
  std::memcpy(a2_q_.data(), a2_radians.data(), a2_radians.size()*sizeof(double));
  std::memcpy(a3_q_.data(), a3_radians.data(), a3_radians.size()*sizeof(double));
  std::memcpy(a4_q_.data(), a4_radians.data(), a4_radians.size()*sizeof(double));
}

void ArmChangerWorker::killCmd_callback(const sbus_interfaces::msg::KillCmd::SharedPtr msg) {
  kill_activated_ = msg->kill_activated;
  // RCLCPP_INFO(this->get_logger(), "kill_activated_: %s", kill_activated_ ? "true" : "false");
}

void ArmChangerWorker::estimator_callback(const controller_interfaces::msg::EstimatorOutput::SharedPtr msg) {
  CoM_pos_ << msg->com_pos[0], msg->com_pos[1], msg->com_pos[2];

  // delta_x_ -= lambda_*CoM_pos_(0);
  // if      (delta_x_ > x_max_){delta_x_ = x_max_;}
  // else if (delta_x_ < x_min_){delta_x_ = x_min_;}

  // delta_y_ -= lambda_*CoM_pos_(1);
  // if      (delta_y_ > y_max_){delta_y_ = y_max_;}
  // else if (delta_y_ < y_min_){delta_y_ = y_min_;}

  // // RCLCPP_INFO(this->get_logger(), "x: %.4f, y: %.4f", delta_x_, delta_y_);
  // // RCLCPP_INFO(this->get_logger(), "x: %.4f, y: %.4f|| x: %.4f, y: %.4f", CoM_pos_(0), CoM_pos_(1), delta_x_, delta_y_);

  // auto a1_radians = compute_ik(-280.0 + half_sqrt2*(-delta_x_-delta_y_), half_sqrt2*(-delta_x_+delta_y_), 140.0, heading_fixed_);
  // auto a2_radians = compute_ik(-280.0 + half_sqrt2*(-delta_x_-delta_y_), half_sqrt2*(delta_x_-delta_y_), 140.0, heading_fixed_);
  // auto a3_radians = compute_ik(-280.0 + half_sqrt2*(delta_x_+delta_y_), half_sqrt2*(delta_x_-delta_y_), 140.0, heading_fixed_);
  // auto a4_radians = compute_ik(-280.0 + half_sqrt2*(delta_x_+delta_y_), half_sqrt2*(-delta_x_+delta_y_), 140.0, heading_fixed_);

  // std::memcpy(a1_q_.data(), a1_radians.data(), a1_radians.size()*sizeof(double));
  // std::memcpy(a2_q_.data(), a2_radians.data(), a2_radians.size()*sizeof(double));
  // std::memcpy(a3_q_.data(), a3_radians.data(), a3_radians.size()*sizeof(double));
  // std::memcpy(a4_q_.data(), a4_radians.data(), a4_radians.size()*sizeof(double));
}

std::array<double,5> ArmChangerWorker::compute_ik(const Eigen::Vector3d &position, const Eigen::Vector3d &heading){
  Eigen::Vector3d p05 = position;
  Eigen::Vector3d p04 = p05 - DH_a5_ * heading;

  double th1 = -std::atan2(p04(0), p04(1)) - M_PI / 2;

  double n = p04(1) * heading(0) - p04(0) * heading(1);
  double th5 = std::acos(std::abs(n) / std::sqrt(std::pow(p04(1), 2) + std::pow(p04(0), 2)));
  if (th5 <= M_PI / 2) th5 -= M_PI / 2;
  if (p04(0) * p05(1) - p04(1) * p05(0) > 0) th5 = -th5;

  double cos_1 = std::cos(th1);
  double sin_1 = std::sin(th1);
  Eigen::Vector3d heading_projected = heading - std::sin(th5) * Eigen::Vector3d(sin_1, -cos_1, 0);
  Eigen::Vector3d p34 = DH_a4_ * heading_projected / heading_projected.norm();
  Eigen::Vector3d p03 = p04 - p34;

  Eigen::Vector3d p01(-DH_a1_ * cos_1, -DH_a1_ * sin_1, 0);
  double x_prime = std::sqrt(std::pow(p01(0) - p03(0), 2) + std::pow(p01(1), 2));
  double y_prime = p03(2);
  double xy_sqr_sum = std::pow(x_prime, 2) + std::pow(y_prime, 2);

  double cos_3 = (xy_sqr_sum - (DH_a2_ * DH_a2_ + DH_a3_ * DH_a3_)) / (2 * DH_a2_ * DH_a3_);
  double sin_3 = std::sqrt(1 - std::pow(cos_3, 2));
  double th3 = -std::acos(cos_3);

  double th2;
  if (p03(0) < p01(0)) {th2 = -std::atan2(y_prime, x_prime) + std::atan2(DH_a3_ * sin_3, DH_a2_ + DH_a3_ * cos_3);    }
  else                 {th2 = std::atan2(y_prime, x_prime) + std::atan2(DH_a3_ * sin_3, DH_a2_ + DH_a3_ * cos_3) - M_PI;}

  double cos_2 = std::cos(th2);
  Eigen::Vector3d p02 = p01 - DH_a2_ * Eigen::Vector3d(cos_2 * cos_1, cos_2 * sin_1, std::sin(th2));
  Eigen::Vector3d p32 = p02 - p03;

  double cos_4 = std::clamp(p32.dot(p34) / (DH_a3_ * DH_a4_), -1.0, 1.0);
  double th4 = M_PI - std::acos(cos_4);
  if (std::abs(cos_4) == 1) th4 = 0.0;

  double th4_ref = std::atan2(p34(2), p34(0)) - std::atan2(p32(2), p32(0));
  if (th4_ref < 0) th4_ref += 2 * M_PI;
  if (th4_ref > M_PI) th4 = -th4;

  return {th1, th2, th3, th4, th5};
}

bool ArmChangerWorker::collision_check(const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& p4) const{
  
  constexpr double R = 225.0;    // [mm]
  constexpr double T = 50.00;    // [mm]

  if (OverLapped(p1,p2,R,T)) return false;
  if (OverLapped(p1,p3,R,T)) return false;
  if (OverLapped(p1,p4,R,T)) return false;
  if (OverLapped(p2,p3,R,T)) return false;
  if (OverLapped(p2,p4,R,T)) return false;
  if (OverLapped(p3,p4,R,T)) return false;

  return true; 
}

bool ArmChangerWorker::path_check(const Eigen::Vector3d& prev_pos, const Eigen::Vector3d& curr_pos, const double dt) const{
  
  constexpr double v_max = 800.0;         // [mm/s]

  if (!prev_pos.allFinite() || !curr_pos.allFinite()) return false;

  Eigen::Vector3d v = (curr_pos - prev_pos) / dt;

  if (!v.allFinite() || v.norm() > v_max) return false;

  return true;
}

bool ArmChangerWorker::ik_check(const std::array<double,5>& q, const Eigen::Vector3d& pos_des, const Eigen::Vector3d& heading_des) const{
  
  if (q.size() != 5) return false; //vec 검사
  
  const double a[5]     = {DH_a1_, DH_a2_, DH_a3_, DH_a4_, DH_a5_};
  const double alpha[5] = {M_PI/2, 0.0, 0.0, M_PI/2, 0.0};
  const double d[5]     = {0.0, 0.0, 0.0, 0.0, 0.0};
 const double th[5]    = {q[0], q[1], q[2], q[3], q[4]};

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 5; ++i) T *= T_dh(a[i], alpha[i], d[i], th[i]);

  const Eigen::Vector3d pos_fk = T.block<3,1>(0,3);
  Eigen::Vector3d heading_fk = T.block<3,3>(0,0).col(0);

  if (!pos_fk.allFinite() || !heading_fk.allFinite()) return false; //inf나 NaN 검사

  heading_fk.normalize();
  Eigen::Vector3d h_des = heading_des.normalized();

  const double pos_err = (pos_fk - pos_des).norm();
  const double heading_product = std::clamp(heading_fk.dot(h_des), -1.0, 1.0);
  const double ang_err = std::atan2(heading_fk.cross(h_des).norm(), heading_product);
  // RCLCPP_WARN(this->get_logger(), "pos_err %f, ang_err %f", pos_err, ang_err);

  return (pos_err <= 5.0 && (ang_err <= 0.1745));  //5mm & 10 deg 
}

void ArmChangerWorker::joint_callback() {
  auto joint_msg = dynamixel_interfaces::msg::JointVal();

  std::memcpy(joint_msg.a1_des.data(), a1_q_.data(), a1_q_.size()*sizeof(double));
  std::memcpy(joint_msg.a2_des.data(), a2_q_.data(), a2_q_.size()*sizeof(double));
  std::memcpy(joint_msg.a3_des.data(), a3_q_.data(), a3_q_.size()*sizeof(double));
  std::memcpy(joint_msg.a4_des.data(), a4_q_.data(), a4_q_.size()*sizeof(double));

  joint_publisher_->publish(joint_msg);
}

void ArmChangerWorker::TiltAngle_callback(const allocator_interfaces::msg::TiltAngleVal::SharedPtr msg)  {
  C2_(0) = msg->th1;
  C2_(1) = msg->th2;
  C2_(2) = msg->th3;
  C2_(3) = msg->th4;
}

void ArmChangerWorker::watchdog_callback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
  // Watchdog update
  watchdog_state_ = msg->state;
}

void ArmChangerWorker::heartbeat_timer_callback() {
  // gate until handshake done
  if (!hb_enabled_) {return;}

  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = hb_state_;
  heartbeat_publisher_->publish(state_msg);

  // uint8 overflow wraps automatically
  hb_state_ = static_cast<uint8_t>(hb_state_ + 1);
}
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmChangerWorker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}