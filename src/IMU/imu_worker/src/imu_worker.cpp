#include "imu_worker.hpp"

constexpr inline std::array<double, 4> quaternion_multiply(const std::array<double, 4>& q1, const std::array<double, 4>& q2) noexcept {
  // Cache components to minimize repeated array accesses
  const double a = q1[0], b = q1[1], c = q1[2], d = q1[3];
  const double e = q2[0], f = q2[1], g = q2[2], h = q2[3];

return { a * e - b * f - c * g - d * h,    // Real part
         a * f + b * e + c * h - d * g,    // i component
         a * g - b * h + c * e + d * f,    // j component
         a * h + b * g - c * f + d * e };  // k component
}

IMUnode::IMUnode()
: Node("imu_node"),
  gen_(std::random_device{}()),
  angle_dist_(0.0, noise_quat_std_dev),
  axis_dist_(0.0, 1.0),
  noise_dist_(0.0, noise_gyro_std_dev),
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
  imu_publisher_ = this->create_publisher<imu_interfaces::msg::ImuMeasured>("imu_mea", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/imu_state", 1);

  // 2) always create heartbeat timer (100ms), but _publish only when enabled
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IMUnode::heartbeat_timer_callback, this));

  // To get the q for b_attitude_cot b_postion_cot
  joint_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("/joint_mea", 1, std::bind(&IMUnode::jointValCallback, this, std::placeholders::_1));

  // 3) mode-specific init
  this->declare_parameter<std::string>("mode", "None");
  std::string mode;
  this->get_parameter("mode", mode);

  if (mode == "real"){
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    // auto sensor_qos = rclcpp::SensorDataQoS{};
    microstrain_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", sensor_qos, std::bind(&IMUnode::microstrain_callback, this, std::placeholders::_1));

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(this->get_node_base_interface());

    while (rclcpp::ok() && !imu_hz_check()) {
      exec.spin_once(std::chrono::milliseconds(1));
    }

    // RCLCPP_INFO(this->get_logger(), " IMU STARTED");

    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&IMUnode::PublishMicroStrainMeasurement, this));

    // initial handshake: immediately send 42 and enable subsequent heartbeat
    hb_state_   = 42;
    hb_enabled_ = true;
  }
  else if (mode == "sim"){
    // Subscription True Measuring value from MuJoCo
    mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("/mujoco_meas", 1, std::bind(&IMUnode::mujoco_callback, this, std::placeholders::_1));
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&IMUnode::PublishMuJoCoMeasurement, this));
    
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
void IMUnode::microstrain_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // This callback must be called in 500hz
  rclcpp::Time now_time = this->now();
  
  real_imu_data_.q[0] = msg->orientation.w;
  real_imu_data_.q[1] = msg->orientation.x;
  real_imu_data_.q[2] = msg->orientation.y;
  real_imu_data_.q[3] = msg->orientation.z;

  real_imu_data_.w[0] = msg->angular_velocity.x;
  real_imu_data_.w[1] = msg->angular_velocity.y;
  real_imu_data_.w[2] = msg->angular_velocity.z;

  // Store timestamp for later frequency estimation
  imu_stamp_buffer_.push_back(now_time);
}

void IMUnode::PublishMicroStrainMeasurement() { // Timer callbacked as 1kHz

  //change the coordinate to {CoT} from {B}

  G_R_B = q2rot(real_imu_data_.q.data());

  Eigen::Matrix3d G_R_cot = G_R_B * B_R_cot;
  Eigen::Vector3d w_B(real_imu_data_.w[0], real_imu_data_.w[1], real_imu_data_.w[2]);

  double q_cot[4]; rot2q(G_R_cot, q_cot);
  Eigen::Vector3d w_cot = B_R_cot.transpose() * w_B;

  auto output_msg = imu_interfaces::msg::ImuMeasured();
  output_msg.q = { q_cot[0], q_cot[1], q_cot[2], q_cot[3] };
  output_msg.w = { w_cot[0], w_cot[1], w_cot[2] };
  imu_publisher_->publish(output_msg);

  // imu disconnect(or hz dropping) monitoring
  rclcpp::Time now = this->now();

  while (!imu_stamp_buffer_.empty() && (now - imu_stamp_buffer_.front()) > check_horizon_) {
    imu_stamp_buffer_.pop_front();
  }

  double freq_est = static_cast<double>(imu_stamp_buffer_.size()) / check_horizon_.seconds();

  if (freq_est < 165.0 && hb_enabled_) {
    RCLCPP_WARN(this->get_logger(), "IMU callback freq dropped to %.1f Hz (<100 Hz). Disabling heartbeat.", freq_est);
    hb_enabled_ = false;
  }

  // RCLCPP_INFO(this->get_logger(), "IMU is %.1f Hz", freq_est);
}

bool IMUnode::imu_hz_check() {
  rclcpp::Time now = this->now();

  while (!imu_stamp_buffer_.empty() && (now - imu_stamp_buffer_.front()) > check_horizon_) {
    imu_stamp_buffer_.pop_front();
  }
  
  double freq = static_cast<double>(imu_stamp_buffer_.size()) / check_horizon_.seconds();
  // RCLCPP_INFO(this->get_logger(), "IMU is %.1f Hz", freq);

  return (freq >= 165.);
}

/* for sim */
void IMUnode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  // Capture the current ROS time when the data is received
  rclcpp::Time now_time = this->now();

  // Push the new data into the buffer
  Delayed_IMUdata new_data;
  new_data.stamp = now_time;
  new_data.q[0] = msg->q[0]; new_data.q[1] = msg->q[1]; new_data.q[2] = msg->q[2]; new_data.q[3] = msg->q[3];
  new_data.w[0] = msg->w[0]; new_data.w[1] = msg->w[1]; new_data.w[2] = msg->w[2];

  data_buffer_.push_back(new_data);

  // Remove older data that is no longer needed to reduce memory usage (older than 10ms)
  while (!data_buffer_.empty()) {
    if ((now_time - data_buffer_.front().stamp).nanoseconds() > 10000000LL) {
      data_buffer_.pop_front();
    }
    else { break; }
  }
}

void IMUnode::PublishMuJoCoMeasurement() {
  if (data_buffer_.empty()) { return; }

  // Determine the target time: current time minus the desired delay
  rclcpp::Time target_time = this->now() - delay_;

  // Search backwards (from newest to oldest) to find the first sample with stamp <= target_time
  Delayed_IMUdata delayed_data;
  bool found = false;

  for (auto it = data_buffer_.rbegin(); it != data_buffer_.rend(); ++it) {
    if (it->stamp <= target_time) {
      delayed_data = *it;
      found = true;
      break;
    }
  }

  if (!found) { return; }

  // Generate small rotation noise as a quaternion using member distributions
  double delta_angle = angle_dist_(gen_);
  double ax = axis_dist_(gen_);
  double ay = axis_dist_(gen_);
  double az = axis_dist_(gen_);
  double norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (norm < 1e-6) { norm = 1.0; }
  ax /= norm; ay /= norm; az /= norm;

  double half_angle = delta_angle / 2.0;
  double cos_half = std::cos(half_angle);
  double sin_half = std::sin(half_angle);
  std::array<double, 4> noise_q = { cos_half, sin_half * ax, sin_half * ay, sin_half * az };

  // Retrieve simulation quaternion (assumed [w, x, y, z] order)
  std::array<double, 4> sim_q = delayed_data.q;

  // Combine noise quaternion with simulation quaternion (noise applied first)
  std::array<double, 4> noisy_q = quaternion_multiply(noise_q, sim_q);

  // Normalize resulting quaternion
  double norm_noisy = std::sqrt(noisy_q[0]*noisy_q[0] + noisy_q[1]*noisy_q[1] +
                                noisy_q[2]*noisy_q[2] + noisy_q[3]*noisy_q[3]);
  for (int i = 0; i < 4; ++i) {
    noisy_q[i] /= norm_noisy;
  }

  // Add white noise to angular velocity measurements using pre-initialized distribution
  double noisy_w[3] = {
    delayed_data.w[0] + noise_dist_(gen_),
    delayed_data.w[1] + noise_dist_(gen_),
    delayed_data.w[2] + noise_dist_(gen_)
  };

  //change the coordinate to {CoT} from {B}

  G_R_B = q2rot(noisy_q.data());

  Eigen::Matrix3d G_R_cot = G_R_B * B_R_cot;
  Eigen::Vector3d w_B(noisy_w[0], noisy_w[1], noisy_w[2]);

  double q_cot[4]; rot2q(G_R_cot, q_cot);
  Eigen::Vector3d w_cot = B_R_cot.transpose() * w_B;

  // Construct and publish the IMU measurement message
  auto output_msg = imu_interfaces::msg::ImuMeasured();
  output_msg.q = { q_cot[0], q_cot[1], q_cot[2], q_cot[3] };
  output_msg.w = { w_cot[0], w_cot[1], w_cot[2] };
  imu_publisher_->publish(output_msg);
}

/* for Both */
void IMUnode::heartbeat_timer_callback() {
  // gate until handshake done
  if (!hb_enabled_) {return;}

  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = hb_state_;
  heartbeat_publisher_->publish(state_msg);

  // uint8 overflow wraps automatically
  hb_state_ = static_cast<uint8_t>(hb_state_ + 1);
}

/* for Coordinate Define */
void IMUnode::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg) {

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

  Eigen::Vector3d z_unit = B_h_arm.rowwise().sum().normalized();
  Eigen::Vector3d x_unit = ((B_p_arm.col(0) + B_p_arm.col(3)) - (B_p_arm.col(1) + B_p_arm.col(2))).normalized();
  Eigen::Vector3d y_unit = ((B_p_arm.col(0) + B_p_arm.col(1)) - (B_p_arm.col(2) + B_p_arm.col(3))).normalized();

  const double ortho_eps = 1e-3; //if not othogonal => believe the X-axis than Y-axis (not perpect yet)
  if ( std::abs(x_unit.dot(y_unit)) > ortho_eps || std::abs(y_unit.dot(z_unit)) > ortho_eps || std::abs(x_unit.dot(z_unit)) > ortho_eps ) y_unit = z_unit.cross(x_unit).normalized();

  B_R_cot.col(0) = x_unit; B_R_cot.col(1) = y_unit; B_R_cot.col(2) = z_unit;

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUnode>());
  rclcpp::shutdown();
  return 0;
}