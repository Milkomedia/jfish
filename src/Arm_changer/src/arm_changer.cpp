#include "arm_changer.hpp"

using namespace std::chrono_literals;

inline double map_value(double input, double in_min, double in_max, double out_min, double out_max) {
  return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ArmChangerWorker::ArmChangerWorker(): Node("arm_changing_node") {
  // ROS2 Subscribers
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("/sbus_signal", 1, std::bind(&ArmChangerWorker::sbus_callback, this, std::placeholders::_1));
  killcmd_subscription_ = this->create_subscription<sbus_interfaces::msg::KillCmd>("sbus_kill", 1, std::bind(&ArmChangerWorker::killCmd_callback, this, std::placeholders::_1));

  // ROS2 Publisher
  joint_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("/joint_cmd", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/armchanger_state", 1);

  // ROS2 Timer
  joint_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmChangerWorker::joint_callback, this));
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArmChangerWorker::heartbeat_timer_callback, this));

  // workspace constrain
  x_min_ = 270.; 
  x_max_ = 330.;
  y_fixed_ = 0.; 
  z_min_ = 50.;
  z_max_ = 220.;

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
}

void ArmChangerWorker::sbus_callback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg) {
  double x = map_value(static_cast<double>(msg->ch[10]), 352, 1696, x_min_, x_max_);
  double z = map_value(static_cast<double>(msg->ch[11]), 352, 1696, z_min_, z_max_);

  compute_ik(x, y_fixed_, z, heading_fixed_);
}

void ArmChangerWorker::killCmd_callback(const sbus_interfaces::msg::KillCmd::SharedPtr msg) {
  kill_activated_ = msg->kill_activated;
  // RCLCPP_INFO(this->get_logger(), "kill_activated_: %s", kill_activated_ ? "true" : "false");
}


void ArmChangerWorker::compute_ik(const double x, const double y, const double z, const Eigen::Vector3d &heading_input){
  Eigen::Vector3d heading = heading_input.normalized();  // Normalize
  Eigen::Vector3d p05(x, y, z);
  Eigen::Vector3d p04 = p05 - a5_ * heading;
  Eigen::Vector3d p03 = p04 - a4_ * heading;

  // θ1
  double th1_ = std::atan2(p03(1), p03(0));

  Eigen::Vector3d p01(a1_ * std::cos(th1_), a1_ * std::sin(th1_), 0);
  Eigen::Vector3d p31 = p03 - p01;
  double r = std::sqrt(std::pow(p31(0), 2) + std::pow(p31(1), 2));
  double s = p31(2);

  // θ3
  double D = (r * r + s * s - a2_ * a2_ - a3_ * a3_) / (2 * a2_ * a3_);
  D = std::clamp(D, -1.0, 1.0);
  th3_ = std::acos(D);

  // θ2
  double alpha = std::atan2(s, r);
  double beta = std::atan2(a3_ * std::sin(th3_), a2_ + a3_ * std::cos(th3_));
  th2_ = alpha - beta;

  // θ4
  Eigen::Matrix3d R03 = R01(th1_) * R12(th2_) * R23(th3_);
  Eigen::Vector3d x3 = R03.col(0);
  Eigen::Vector3d x4_desired = (p04 - p03).normalized();
  th4_ = std::acos(std::clamp(x3.dot(x4_desired), -1.0, 1.0));

  // θ5
  // Eigen::Matrix3d R04 = R03 * R34(th4_);
  // Eigen::Vector3d x4 = R04.col(0);
  // Eigen::Vector3d cross = x4.cross(heading);
  // double sign = std::copysign(1.0, R04.col(2).dot(cross));
  // double dot = std::clamp(x4.dot(heading), -1.0, 1.0);
  // th5_ = sign * std::acos(dot);
  th5_ = 0; //fixed
}


void ArmChangerWorker::joint_callback() {
  auto joint_msg = dynamixel_interfaces::msg::JointVal();

  joint_msg.a1_des = {th1_, th2_, th3_, th4_, th5_};
  joint_msg.a2_des = {th1_, th2_, th3_, th4_, th5_};
  joint_msg.a3_des = {th1_, th2_, th3_, th4_, th5_};
  joint_msg.a4_des = {th1_, th2_, th3_, th4_, th5_};

  joint_publisher_->publish(joint_msg);
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