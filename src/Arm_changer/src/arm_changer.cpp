#include "arm_changer.hpp"
#include <cmath>     
#include <algorithm>   
#include <chrono>    

using namespace std::chrono_literals;

ArmChangerWorker::ArmChangerWorker(): Node("arm_changing_node") {
  
  // ROS2 Subscribers
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("/sbus_signal", 1, std::bind(&ArmChangerWorker::sbus_callback, this, std::placeholders::_1));
  killcmd_subscription_ = this->create_subscription<sbus_interfaces::msg::KillCmd>("sbus_kill", 1, std::bind(&ArmChangerWorker::killCmd_callback, this, std::placeholders::_1));
  tilt_angle_val_subscription_ = this->create_subscription<allocator_interfaces::msg::TiltAngleVal>("tilt_cmd", 1, std::bind(&ArmChangerWorker::TiltAngle_callback, this, std::placeholders::_1));

  // ROS2 Publisher
  joint_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("/joint_cmd", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/armchanger_state", 1);

  // ROS2 Timer
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArmChangerWorker::heartbeat_timer_callback, this));

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
}

void ArmChangerWorker::sbus_callback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg) {

  //local(J1)2global(base) cmd-------------------------------------------------------------------------------------------------------------------------------
  std::array<double, 5> a1_q, a2_q, a3_q, a4_q;

  double x = map(static_cast<double>(msg->ch[10]), 352, 1696, x_min_, x_max_); // sbus min/max: 352/1696
  double y  = 0;
  double z = map(static_cast<double>(msg->ch[11]), 352, 1696, z_min_, z_max_); // sbus min/max: 352/1696
  
  Eigen::Vector3d pos_des_local(x, y, z);
  // Eigen::Vector3d heading1(0.0, -std::sin(tilted_rad), std::sqrt(1-std::sin(tilted_rad)*std::sin(tilted_rad))); // arm1
  // Eigen::Vector3d heading2(0.0,  std::sin(tilted_rad), std::sqrt(1-std::sin(tilted_rad)*std::sin(tilted_rad))); // arm2
  // Eigen::Vector3d heading3(0.0, -std::sin(tilted_rad), std::sqrt(1-std::sin(tilted_rad)*std::sin(tilted_rad))); // arm3
  // Eigen::Vector3d heading4(0.0,  std::sin(tilted_rad), std::sqrt(1-std::sin(tilted_rad)*std::sin(tilted_rad))); // arm4
  Eigen::Vector3d heading1(0.0, std::sin(C2_(0)), std::sqrt(1-std::sin(C2_(0))*std::sin(C2_(0)))); // arm1
  Eigen::Vector3d heading2(0.0, std::sin(C2_(1)), std::sqrt(1-std::sin(C2_(1))*std::sin(C2_(1)))); // arm2
  Eigen::Vector3d heading3(0.0, std::sin(C2_(2)), std::sqrt(1-std::sin(C2_(2))*std::sin(C2_(2)))); // arm3
  Eigen::Vector3d heading4(0.0, std::sin(C2_(3)), std::sqrt(1-std::sin(C2_(3))*std::sin(C2_(3)))); // arm4

  auto [p1_des_base, heading1_base] = arm2base(pos_des_local, heading1, 1);
  auto [p2_des_base, heading2_base] = arm2base(pos_des_local, heading2, 2);
  auto [p3_des_base, heading3_base] = arm2base(pos_des_local, heading3, 3);
  auto [p4_des_base, heading4_base] = arm2base(pos_des_local, heading4, 4);

  // ---- collision check (base frame) ----------------------------------------------------------------------------------------------------------------------
  if (!collision_check(p1_des_base, p2_des_base, p3_des_base, p4_des_base)) {
  hb_enabled_ = false;
  RCLCPP_WARN(this->get_logger(), "Collision detected: discs overlap → heartbeat disabled!");
  return;
  }

  //path velocity check--------------------------------------------------------------------------------------------------------------------------------------
  auto now_t = this->now();

  if (!has_last_des_) {
    last_des_pos_  = pos_des_local;
    last_des_time_ = now_t;
    has_last_des_  = true;
  } 
  else {
    double dt = (now_t - last_des_time_).seconds();

    if (!path_check(last_des_pos_, pos_des_local, dt)) {
      hb_enabled_ = false;
      RCLCPP_WARN(this->get_logger(), "Path check failed: speed limit exceeded → heartbeat disabled");
      return;
    }
  }
  last_des_pos_  = pos_des_local;
  last_des_time_ = now_t;

  //FK2IK---------------------------------------------------------------------------------------------------------------------------------------------------
  a1_q = compute_ik(x, y, z, heading1);
  // RCLCPP_WARN(this->get_logger(), "xyz %f %f %f, a1_q %f %f %f %f %f", heading1(0),heading1(1),heading1(2),a1_q[0],a1_q[1],a1_q[2],a1_q[3],a1_q[4]);
  a2_q = compute_ik(x, y, z, heading2);
  a3_q = compute_ik(x, y, z, heading3);
  a4_q = compute_ik(x, y, z, heading4);

  //IK check------------------------------------------------------------------------------------------------------------------------------------------------
  if (!ik_check(a1_q, pos_des_local, heading1) || !ik_check(a2_q, pos_des_local, heading2) || !ik_check(a3_q, pos_des_local, heading3) || !ik_check(a4_q, pos_des_local, heading4)) {
      // hb_enabled_ = false;
      // RCLCPP_WARN(this->get_logger(), "IK check failed, heartbeat disabled!");
      // return;
  }

  auto joint_msg = dynamixel_interfaces::msg::JointVal();
  joint_msg.a1_des = a1_q;
  joint_msg.a2_des = a2_q;
  joint_msg.a3_des = a3_q;
  joint_msg.a4_des = a4_q;

  joint_publisher_->publish(joint_msg);
}

void ArmChangerWorker::killCmd_callback(const sbus_interfaces::msg::KillCmd::SharedPtr msg) {
  // currently not working
  kill_activated_ = msg->kill_activated;
  // RCLCPP_INFO(this->get_logger(), "kill_activated_: %s", kill_activated_ ? "true" : "false");
}

std::array<double, 5> ArmChangerWorker::compute_ik(const double x, const double y, const double z, const Eigen::Vector3d &heading_input){
  Eigen::Vector3d heading = heading_input.normalized();  // Normalize
  Eigen::Vector3d p05(x, y, z);
  Eigen::Vector3d p04 = p05 - a5_ * heading;
  Eigen::Vector3d p03 = p04 - a4_ * heading;

  // θ1
  double th1 = std::atan2(p03(1), p03(0));

  Eigen::Vector3d p01(a1_ * std::cos(th1), a1_ * std::sin(th1), 0);
  Eigen::Vector3d p31 = p03 - p01;
  double r = std::sqrt(std::pow(p31(0), 2) + std::pow(p31(1), 2));
  double s = p31(2);

  // θ3
  double D = (r * r + s * s - a2_ * a2_ - a3_ * a3_) / (2 * a2_ * a3_);
  D = std::clamp(D, -1.0, 1.0);
  double th3 = std::acos(D);

  // θ2
  double alpha = std::atan2(s, r);
  double beta = std::atan2(a3_ * std::sin(th3), a2_ + a3_ * std::cos(th3));
  double th2 = alpha - beta;

  // θ4
  Eigen::Matrix3d R03 = R01(th1) * R12(th2) * R23(th3);
  Eigen::Vector3d x3 = R03.col(0);
  Eigen::Vector3d x4_desired = (p04 - p03).normalized();
  double th4 = std::acos(std::clamp(x3.dot(x4_desired), -1.0, 1.0));

  // θ5
  Eigen::Matrix3d R04 = R03 * R34(th4);
  Eigen::Vector3d x4 = R04.col(0);
  Eigen::Vector3d cross = x4.cross(heading);
  double sign = std::copysign(1.0, R04.col(2).dot(cross));
  double dot = std::clamp(x4.dot(heading), -1.0, 1.0);
  double th5 = sign * std::acos(dot);

  return {th1, th2, th3, th4, th5};
}

bool ArmChangerWorker::ik_check(const std::array<double,5>& q, const Eigen::Vector3d& pos_des, const Eigen::Vector3d& heading_des) const{
  
  if (q.size() != 5) return false; //vec 검사
  
  const double a[5]     = {a1_, a2_, a3_, a4_, a5_};
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

void ArmChangerWorker::TiltAngle_callback(const allocator_interfaces::msg::TiltAngleVal::SharedPtr msg)  {
  C2_(0) = msg->th1;
  C2_(1) = msg->th2;
  C2_(2) = msg->th3;
  C2_(3) = msg->th4;
}

bool ArmChangerWorker::path_check(const Eigen::Vector3d& prev_pos, const Eigen::Vector3d& curr_pos, const double dt) const{
  
  constexpr double v_max = 800.0;         // [mm/s]

  if (!prev_pos.allFinite() || !curr_pos.allFinite()) return false;

  Eigen::Vector3d v = (curr_pos - prev_pos) / dt;

  if (!v.allFinite() || v.norm() > v_max) return false;

  return true;
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