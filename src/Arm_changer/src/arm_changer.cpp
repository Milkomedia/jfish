#include "arm_changer.hpp"

using namespace std::chrono_literals;


inline Eigen::Matrix3d R01(double th1) {
  Eigen::Matrix3d R;
  R << std::cos(th1), 0, std::sin(th1),
        std::sin(th1), 0, -std::cos(th1),
        0,             1, 0;
  return R;
}  

inline Eigen::Matrix3d R12(double th2) {
  Eigen::Matrix3d R;
  R << std::cos(th2), -std::sin(th2), 0,
        std::sin(th2),  std::cos(th2), 0,
        0,              0,             1;
  return R;
}

inline Eigen::Matrix3d R23(double th3) {
  Eigen::Matrix3d R;
  R << std::cos(th3), -std::sin(th3), 0,
        std::sin(th3),  std::cos(th3), 0,
        0,              0,             1;
  return R;
}

inline Eigen::Matrix3d R34(double th4) {
  Eigen::Matrix3d R;
  R << std::cos(th4), 0, -std::sin(th4),
        std::sin(th4), 0,  std::cos(th4),
        0,            -1, 0;
  return R;
}

ArmChangerWorker::ArmChangerWorker(): Node("arm_changing_node") {
  
  // ROS2 Subscribers
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("/sbus_signal", 1, std::bind(&ArmChangerWorker::sbus_callback, this, std::placeholders::_1));
  killcmd_subscription_ = this->create_subscription<sbus_interfaces::msg::KillCmd>("sbus_kill", 1, std::bind(&ArmChangerWorker::killCmd_callback, this, std::placeholders::_1));

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
  std::array<double, 5> a1_q, a2_q, a3_q, a4_q;

  double x = map(static_cast<double>(msg->ch[10]), 352, 1696, x_min_, x_max_); // sbus min/max: 352/1696
  double y  = 0.0;
  double z = map(static_cast<double>(msg->ch[11]), 352, 1696, z_min_, z_max_); // sbus min/max: 352/1696

  double tilted_rad = 0.0872665; // 5 deg
  double sin_theta = std::sin(tilted_rad);
  double sin_theta_sqr = sin_theta*sin_theta;

  Eigen::Vector3d heading1(0.0, -sin_theta, std::sqrt(1-sin_theta_sqr)); // arm1
  Eigen::Vector3d heading2(0.0,  sin_theta, std::sqrt(1-sin_theta_sqr)); // arm2
  Eigen::Vector3d heading3(0.0, -sin_theta, std::sqrt(1-sin_theta_sqr)); // arm3
  Eigen::Vector3d heading4(0.0,  sin_theta, std::sqrt(1-sin_theta_sqr)); // arm4
  
  a1_q = compute_ik(x, y, z, heading1);
  a2_q = compute_ik(x, y, z, heading2);
  a3_q = compute_ik(x, y, z, heading3);
  a4_q = compute_ik(x, y, z, heading4);

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

void ArmChangerWorker::watchdog_callback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {// 뭐하는 새끼임?
  // Watchdog update
  //  currently not working
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