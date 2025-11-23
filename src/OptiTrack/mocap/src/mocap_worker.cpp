#include "mocap_worker.hpp"

using namespace std::chrono_literals;

OptiTrackNode::OptiTrackNode()
 : Node("optitrack_node"),
  gen_(std::random_device{}()),
  pos_dist_(0.0, noise_pos_std_dev),
  vel_dist_(0.0, noise_vel_std_dev),
  acc_dist_(0.0, noise_acc_std_dev),
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

  // 1) publishers
  mocap_publisher_ = this->create_publisher<mocap_interfaces::msg::MocapMeasured>("optitrack_mea", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("optitrack_state", 1);
  
  // 2) always create heartbeat timer (100ms), but _publish only when enabled
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OptiTrackNode::heartbeat_timer_callback, this));

  // 3) mode-specific init
  this->declare_parameter<std::string>("mode", "None");
  std::string mode;
  this->get_parameter("mode", mode);

  // To get the q for b_attitude_cot b_postion_cot
  joint_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("/joint_mea", 1, std::bind(&OptiTrackNode::jointValCallback, this, std::placeholders::_1));
  imu_mea_subscription_ = this->create_subscription<imu_interfaces::msg::ImuMeasured>("/imu_mea", 1, std::bind(&OptiTrackNode::imuCallback, this, std::placeholders::_1));

  if (mode == "real"){
    // Mode = real -> Reading OptiTrack
    optitrack_mea_subscription_ = this->create_subscription<mocap_interfaces::msg::NamedPoseArray>("/opti_pos", 1, std::bind(&OptiTrackNode::optitrack_callback, this, std::placeholders::_1));

    while (rclcpp::ok() && !opti_hz_check()) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&OptiTrackNode::PublishOptiTrackMeasurement, this));

    // initial handshake: immediately send 42 and enable subsequent heartbeat
    hb_state_   = 42;
    hb_enabled_ = true;
  }
  else if (mode == "sim"){
    // Subscription True Measuring value from MuJoCo
    mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("/mujoco_meas", 1, std::bind(&OptiTrackNode::mujoco_callback, this, std::placeholders::_1));
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&OptiTrackNode::PublishMuJoCoMeasurement, this));

    // initial handshake: immediately send 42 and enable subsequent heartbeat
    hb_state_   = 42;
    hb_enabled_ = true;
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s. No initialization performed.", mode.c_str());
    exit(1);
  }
}

/* for real */
void OptiTrackNode::optitrack_callback(const mocap_interfaces::msg::NamedPoseArray::SharedPtr msg) {
  // This callback must be called in 360hz
  rclcpp::Time now_time = this->now();

  for (const auto& pose : msg->poses){
    if (pose.name == "strider"){
      real_optitrack_data_.pos[0] = pose.pose.position.x;
      real_optitrack_data_.pos[1] = pose.pose.position.y;
      real_optitrack_data_.pos[2] = pose.pose.position.z;
      break;
    }
  }

  double dt = 0.0;
  if (last_time_.nanoseconds() != 0) {dt = (now_time - last_time_).seconds();}
  last_time_ = now_time;
  last_dt_   = dt; 

  if (dt > 0.0) {
    for (int i = 0; i < 3; ++i) {
      vel_raw[i] = real_optitrack_data_.vel[i] = (real_optitrack_data_.pos[i] - last_pos_[i]) / dt;
      vel_filtered_[i] = vel_lpf_alpha_ * vel_raw[i] + vel_lpf_beta_ * vel_filtered_[i];
      real_optitrack_data_.vel[i] = vel_filtered_[i];

      acc_raw[i] = real_optitrack_data_.acc[i] = (vel_filtered_[i] - last_vel_[i]) / dt;
      acc_filtered_[i] = acc_lpf_alpha_ * acc_raw[i] + acc_lpf_beta_ * acc_filtered_[i];
      real_optitrack_data_.acc[i] = acc_filtered_[i];
    }
  }
  
  last_pos_  = real_optitrack_data_.pos;
  last_vel_  = vel_filtered_;

  opti_stamp_buffer_.push_back(now_time);
}

void OptiTrackNode::PublishOptiTrackMeasurement() { // Timer callbacked as 500Hz

  //change the coordinate to {CoT} from {B}
  
  // Postion
  Eigen::Matrix3d G_R_B = G_R_cot *  B_R_cot.transpose();
  Eigen::Vector3d G_p_cot = G_R_B * B_p_cot; // bias (i.e. {B_p_cot}_G = G에서 본 B좌표계에서 VoT까지의 위치벡터값)
  for (size_t i = 0; i < 3; ++i) G_p_cot[i] += real_optitrack_data_.pos[i]; 

  // Velocity
  Eigen::Vector3d G_v_cot = G_R_B * (B_v_cot + omega_cot.cross(B_p_cot)); // G에서 본 B와 CoT의상대속도값
  for (size_t i = 0; i < 3; ++i) G_v_cot[i] += real_optitrack_data_.vel[i]; 

  // Accelation
  Eigen::Vector3d G_a_cot; // pass
  for (size_t i = 0; i < 3; ++i) G_a_cot[i] += real_optitrack_data_.acc[i]; 

  auto output_msg = mocap_interfaces::msg::MocapMeasured();
  output_msg.pos = { G_p_cot[0], G_p_cot[1], G_p_cot[2] };
  output_msg.vel = { G_v_cot[0], G_v_cot[1], G_v_cot[2] };
  output_msg.acc = { G_a_cot[0], G_a_cot[1], G_a_cot[2] };
  mocap_publisher_->publish(output_msg);

  // opti disconnect monitoring
  rclcpp::Time now = this->now();

  while (!opti_stamp_buffer_.empty() && (now - opti_stamp_buffer_.front()) > check_horizon_) {
    opti_stamp_buffer_.pop_front();
  }

  double freq_est = static_cast<double>(opti_stamp_buffer_.size()) / 0.5;

  if (freq_est < 200.0 && hb_enabled_) {
    RCLCPP_WARN(this->get_logger(), "OptiTrack callback freq dropped to %.1f Hz (<275 Hz). Disabling heartbeat.", freq_est);
    hb_enabled_ = false;
  }
}

bool OptiTrackNode::opti_hz_check() {
  rclcpp::Time now = this->now();

  while (!opti_stamp_buffer_.empty() && (now - opti_stamp_buffer_.front()) > check_horizon_) {
    opti_stamp_buffer_.pop_front();
  }
  
  double freq = static_cast<double>(opti_stamp_buffer_.size()) / check_horizon_.seconds();
  return (freq >= 200.0);
}

/* for sim */
void OptiTrackNode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  
  // Capture the current ROS time when the data is received
  rclcpp::Time now_time = this->now();

  // Push the new data into the buffer
  Delayed_OPTIdata new_data;
  new_data.stamp = now_time;
  for (size_t i = 0; i < 3; ++i) {
    new_data.pos[i] = msg->pos[i];
    new_data.vel[i] = msg->vel[i];
    new_data.acc[i] = msg->acc[i];
  }
  data_buffer_.push_back(new_data);

  // Remove older data that is no longer needed to reduce memory usage (older than 10ms)
  while (!data_buffer_.empty()) {
    if ((now_time - data_buffer_.front().stamp).nanoseconds() > 10000000LL) {
      data_buffer_.pop_front();
    }
    else { break; }
  }
}

void OptiTrackNode::PublishMuJoCoMeasurement() {
  if (data_buffer_.empty()) { return; }

  // Determine the target time: current time minus the desired delay
  rclcpp::Time target_time = this->now() - delay_;

  // Search backwards (from newest to oldest) to find the first sample with stamp <= target_time
  Delayed_OPTIdata delayed_data;
  bool found = false;

  for (auto it = data_buffer_.rbegin(); it != data_buffer_.rend(); ++it) {
    if (it->stamp <= target_time) {
      delayed_data = *it;
      found = true;
      break;
    }
  }

  if (!found) { return; }

  std::array<double, 3> noisy_pos;
  std::array<double, 3> noisy_vel;
  std::array<double, 3> noisy_acc;
  for (size_t i = 0; i < 3; ++i) {
    noisy_pos[i] = delayed_data.pos[i] + pos_dist_(gen_);
    noisy_vel[i] = delayed_data.vel[i] + vel_dist_(gen_);
    noisy_acc[i] = delayed_data.acc[i] + acc_dist_(gen_);
  }

  //change the coordinate to {CoT} from {B}
  
  // Postion
  Eigen::Matrix3d G_R_B = G_R_cot *  B_R_cot.transpose();
  Eigen::Vector3d G_p_cot = G_R_B * B_p_cot; // bias (i.e. {B_p_cot}_G = G에서 본 B좌표계에서 VoT까지의 위치벡터값)
  for (size_t i = 0; i < 3; ++i) G_p_cot[i] += noisy_pos[i]; 

  // Velocity
  
  Eigen::Vector3d G_v_cot = G_R_B * (B_v_cot + (B_R_cot*omega_cot).cross(B_p_cot)); // G에서 본 B와 CoT의상대속도값 
  for (size_t i = 0; i < 3; ++i) G_v_cot[i] += noisy_vel[i]; 
  RCLCPP_INFO(this->get_logger(), "%f %f", B_v_cot(1), G_v_cot(1));
  // Accelation
  Eigen::Vector3d G_a_cot; // pass
  for (size_t i = 0; i < 3; ++i) G_a_cot[i] += real_optitrack_data_.acc[i]; 

  // Publish data
  auto output_msg = mocap_interfaces::msg::MocapMeasured();
  output_msg.pos = { G_p_cot[0], G_p_cot[1], G_p_cot[2] };
  output_msg.vel = { noisy_vel[0], noisy_vel[1], noisy_vel[2] };
  output_msg.acc = { G_a_cot[0], G_a_cot[1], G_a_cot[2] };

  mocap_publisher_->publish(output_msg);
}

/* for Both */
void OptiTrackNode::heartbeat_timer_callback() {
  // gate until handshake done
  if (!hb_enabled_) {return;}

  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = hb_state_;
  heartbeat_publisher_->publish(state_msg);

  // uint8 overflow wraps automatically
  hb_state_ = static_cast<uint8_t>(hb_state_ + 1);
}

/* for Coordinate Define */
void OptiTrackNode::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg)  {

  for (uint8_t i = 0; i < 5; ++i) {
    arm_des_[0][i] = msg->a1_des[i];   // Arm 1
    arm_des_[1][i] = msg->a2_des[i];   // Arm 2
    arm_des_[2][i] = msg->a3_des[i];   // Arm 3
    arm_des_[3][i] = msg->a4_des[i];   // Arm 4
  }

  for (uint8_t arm = 0; arm < 4; ++arm) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (uint8_t i = 0; i <= 5; ++i) {
      const double q = (i == 0) ? q_B0_(arm) : arm_des_[arm][i-1];
      T *= compute_DH(DH_params_(i,0), DH_params_(i,1), DH_params_(i,2), DH_params_(i,3) + q);
    }
    // save position vector
    B_p_arm.col(arm) = T.block<3,1>(0,3);
    B_h_arm.col(arm) = T.block<3,3>(0,0).col(0);
  }

  // get the CoT coordinate => B_p_cot / B_R_cot
  B_p_cot = B_p_arm.rowwise().mean();

  std::string mode;
  if (!this->get_parameter("mode", mode)) mode = "sim";
  last_dt_ = (mode == "real") ? last_dt_ : 1.0/400.0;

  if (last_dt_ > 0.0) B_v_cot = (B_p_cot - B_p_cot_prev) / last_dt_;
  B_p_cot_prev = B_p_cot;

  Eigen::Vector3d z_unit = B_h_arm.rowwise().sum().normalized();
  Eigen::Vector3d x_unit = ((B_p_arm.col(0) + B_p_arm.col(3)) - (B_p_arm.col(1) + B_p_arm.col(2))).normalized();
  Eigen::Vector3d y_unit = ((B_p_arm.col(0) + B_p_arm.col(1)) - (B_p_arm.col(2) + B_p_arm.col(3))).normalized();

  const double ortho_eps = 1e-3; //if not othogonal => believe the X-axis than Y-axis (not perpect yet)
  if ( std::abs(x_unit.dot(y_unit)) > ortho_eps || std::abs(y_unit.dot(z_unit)) > ortho_eps || std::abs(x_unit.dot(z_unit)) > ortho_eps ) y_unit = z_unit.cross(x_unit).normalized();

  B_R_cot.col(0) = x_unit; B_R_cot.col(1) = y_unit; B_R_cot.col(2) = z_unit;

}

/* for Coordinate Define */
void OptiTrackNode::imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg) {

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

  G_R_cot(0,0) = 1.0 - 2.0 * (yy + zz);
  G_R_cot(0,1) = 2.0 * (xy - wz);
  G_R_cot(0,2) = 2.0 * (xz + wy);

  G_R_cot(1,0) = 2.0 * (xy + wz);
  G_R_cot(1,1) = 1.0 - 2.0 * (xx + zz);
  G_R_cot(1,2) = 2.0 * (yz - wx);

  G_R_cot(2,0) = 2.0 * (xz - wy);
  G_R_cot(2,1) = 2.0 * (yz + wx);
  G_R_cot(2,2) = 1.0 - 2.0 * (xx + yy);

  omega_cot << msg->w[0], msg->w[1], msg->w[2];
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptiTrackNode>());
  rclcpp::shutdown();
  return 0;
}
