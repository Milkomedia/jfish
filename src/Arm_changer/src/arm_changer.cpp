#include "arm_changer.hpp"

using namespace std::chrono_literals;

ArmChangerWorker::ArmChangerWorker(): Node("arm_changing_node") {
  // ROS2 Subscribers
  controller_subscription_ = this->create_subscription<controller_interfaces::msg::EstimatorOutput>("/estimator_output", 1, std::bind(&ArmChangerWorker::estimator_callback, this, std::placeholders::_1));
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("/sbus_signal", 1, std::bind(&ArmChangerWorker::sbus_callback, this, std::placeholders::_1));
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

  if(!hb_enabled_) return;
  
  if(checksum_on){
    delta_x_manual = delta_x_;
    delta_y_manual = delta_y_;
  }
  else {
    delta_x_manual = map(static_cast<double>(msg->ch[10]), 352, 1696, x_min_, x_max_);
    delta_y_manual = map(static_cast<double>(msg->ch[11]), 352, 1696, y_min_, y_max_);
    //RCLCPP_INFO(this->get_logger(), "x: %.4f, y: %.4f", delta_x_manual, delta_y_manual);  
  }
  
  //heading angle [rad]
  Eigen::Vector3d heading1(0.0, std::sin(C2_(0)), std::sqrt(1-std::sin(C2_(0))*std::sin(C2_(0)))); // arm1
  Eigen::Vector3d heading2(0.0, std::sin(C2_(1)), std::sqrt(1-std::sin(C2_(1))*std::sin(C2_(1)))); // arm2
  Eigen::Vector3d heading3(0.0, std::sin(C2_(2)), std::sqrt(1-std::sin(C2_(2))*std::sin(C2_(2)))); // arm3
  Eigen::Vector3d heading4(0.0, std::sin(C2_(3)), std::sqrt(1-std::sin(C2_(3))*std::sin(C2_(3)))); // arm4

  // //arm postion [mm]
  Eigen::Vector3d arm_position1(250.0 + half_sqrt2*(-delta_x_manual-delta_y_manual),   half_sqrt2*(+delta_x_manual-delta_y_manual),  220.0); //arm1
  Eigen::Vector3d arm_position2(250.0 + half_sqrt2*(+delta_x_manual-delta_y_manual),   half_sqrt2*(+delta_x_manual+delta_y_manual),  220.0); //arm2
  Eigen::Vector3d arm_position3(250.0 + half_sqrt2*(+delta_x_manual+delta_y_manual),   half_sqrt2*(-delta_x_manual+delta_y_manual),  220.0); //arm3
  Eigen::Vector3d arm_position4(250.0 + half_sqrt2*(-delta_x_manual+delta_y_manual),   half_sqrt2*(-delta_x_manual-delta_y_manual),  220.0); //arm4

  //Base(J1) 2 Body(base)
  auto [arm_position1_body, heading1_body] = arm2body(arm_position1, heading1, 1);
  auto [arm_position2_body, heading2_body] = arm2body(arm_position2, heading2, 2);
  auto [arm_position3_body, heading3_body] = arm2body(arm_position3, heading3, 3);
  auto [arm_position4_body, heading4_body] = arm2body(arm_position4, heading4, 4);

  //workspace check (Base frame 1 by 1 )
  if(!workspace_check(arm_position1,1)||!workspace_check(arm_position2,2)||!workspace_check(arm_position3,3)||!workspace_check(arm_position4,4)){
    checksum_on = true;
    return;
  } 
  else checksum_on = false;

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
  if (!ik_check(a1_radians, arm_position1, heading1, 1) || !ik_check(a2_radians, arm_position2, heading2, 2) || !ik_check(a3_radians, arm_position3, heading3, 3) || !ik_check(a4_radians, arm_position4, heading4, 4)) {
      // hb_enabled_ = false;
      // RCLCPP_WARN(this->get_logger(), "IK check failed, heartbeat disabled!");
      // return;
  }

  delta_x_ = delta_x_manual;
  delta_y_ = delta_y_manual;

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

bool ArmChangerWorker::workspace_check(const Eigen::Vector3d& pos, int arm_num) const {
  const double r  = std::hypot(pos.x(), pos.y());
  const double z2 = pos.z() * pos.z();
  const double z3 = z2 * pos.z();

  const double rmin = -7.02134e-05 * z3 + 0.0271478 * z2 - 3.75127 * pos.z() + 469.795;
  const double rmax = -3.69431e-06 * z3 - 9.24162e-04 * z2 + 0.255934 * pos.z() + 336.389;

  constexpr double LIM = 50.0 * M_PI / 180.0;
  const double az = std::atan2(pos.y(), pos.x()); 

  const bool radial_ok = (r >= rmin && r <= rmax);
  const bool angle_ok  = (std::abs(az) <= LIM);
  
  if(!radial_ok)RCLCPP_WARN(this->get_logger(), "out of workspace (Arm%d, r=%fmm)", arm_num, r);
  if(!angle_ok)RCLCPP_WARN(this->get_logger(), "out of workspace (Arm%d, a=%fdeg)", arm_num, az*180/M_PI);

  return radial_ok && angle_ok;
}

std::array<double, 5> ArmChangerWorker::compute_ik(const Eigen::Vector3d &p05, const Eigen::Vector3d &heading_input){
  Eigen::Vector3d heading = heading_input.normalized();  // Normalize
  const double a1_ = 134.;
  const double a2_ = 115.;
  const double a3_ = 110.;
  const double a4_ = 24.;
  const double a5_ = 68.;

  const double EPS = 1e-12;

  //Eigen::Vector3d p05(x, y, z);
  Eigen::Vector3d p04 = p05 - a5_ * heading;

  // θ1
  double th1 = std::atan2(p04(1), p04(0));

  // θ5
  const double cross_z = p04.x() * heading.y() - p04.y() * heading.x();
  const double denom_xy = std::hypot(p04.x(), p04.y()) + EPS;
  double th5 = -std::acos(std::clamp(std::abs(cross_z) / denom_xy, -1.0, 1.0));

  if (th5 <= M_PI / 2.0) th5 += M_PI / 2.0;
  if (p04.x() * p05.y() - p04.y() * p05.x() > 0.0) th5 = -th5;

  Eigen::Vector3d heading_projected = heading - std::sin(th5) * Eigen::Vector3d(std::sin(th1), -std::cos(th1), 0.0);
  if (heading_projected.norm() > EPS) heading_projected /= heading_projected.norm();

  Eigen::Vector3d p01(a1_ * std::cos(th1), a1_ * std::sin(th1), 0);
  Eigen::Vector3d p34 = a4_ * heading_projected;
  Eigen::Vector3d p03 = p04 - p34;
  Eigen::Vector3d p31 = p03 - p01;
  

  // θ3
  double r = std::sqrt(std::pow(p31(0), 2) + std::pow(p31(1), 2));
  double s = p31(2);
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
  Eigen::Vector3d z3 = R03.col(2);

  Eigen::Vector3d x4_des = p34;
  const double x4n = x4_des.norm();
  if (x4n > EPS) x4_des /= x4n;           // x4_des = p34 / ||p34||

  double c4 = std::clamp(x3.dot(x4_des), -1.0, 1.0);
  double s4 = z3.dot(x3.cross(x4_des));
  const double th4 = std::atan2(s4, c4);
  
  //RCLCPP_WARN(this->get_logger(), "%f %f %f %f %f", th1, th2, th3, th4, th5);
  return {th1, th2, th3, th4, th5};
}

bool ArmChangerWorker::collision_check(const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& p4) const{
  
  constexpr double R = 190.0;    // [mm]
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
  
  constexpr double v_max = 10000.0;         // [mm/s]

  if (!prev_pos.allFinite() || !curr_pos.allFinite()) return false;

  Eigen::Vector3d v = (curr_pos - prev_pos) / dt;

  if (!v.allFinite() || v.norm() > v_max) return false;

  return true;
}

bool ArmChangerWorker::ik_check(const std::array<double,5>& q, const Eigen::Vector3d& pos_des, const Eigen::Vector3d& heading_des, int arm_num) const{
  
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

  Eigen::Vector3d h_des = heading_des.normalized();
  Eigen::Vector3d h_cur = heading_fk.normalized();

  const double pos_err = (pos_fk - pos_des).norm();
  const double heading_product = std::clamp(h_cur.dot(h_des), -1.0, 1.0);
  const double ang_err = std::atan2(h_cur.cross(h_des).norm(), heading_product);
  bool pos_ok = pos_err <= 10.0; //10mm
  bool ang_ok = ang_err <= 0.01745*2; // 2deg

  if(!pos_ok) RCLCPP_WARN(this->get_logger(), "IK Fail (Arm%d, pos=%fmm )", arm_num, pos_err);
  //if(!ang_ok) RCLCPP_WARN(this->get_logger(), "IK Fail (Arm%d, ang=%fdeg)", arm_num, ang_err*180/M_PI);
  RCLCPP_WARN(this->get_logger(), "des %f %f %f | cur %f %f %f", h_des.x(), h_des.y(), h_des.z(), h_cur.x(), h_cur.y(), h_cur.z());
  return (pos_ok && ang_ok); 
  return true;
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