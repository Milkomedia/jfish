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

constexpr inline std::array<double, 3> rotate_accel_to_world(const std::array<double, 4>& q, const std::array<double, 3>& a_local) noexcept {
  double w = q[0], x = q[1], y = q[2], z = q[3];
  double R[3][3] = {
    {1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w},
    {    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w},
    {    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y}
  };
  return {
    R[0][0]*a_local[0] + R[0][1]*a_local[1] + R[0][2]*a_local[2],
    R[1][0]*a_local[0] + R[1][1]*a_local[1] + R[1][2]*a_local[2],
    R[2][0]*a_local[0] + R[2][1]*a_local[1] + R[2][2]*a_local[2]
  };
}

IMUnode::IMUnode()
: Node("imu_node"),
  gen_(std::random_device{}()),
  angle_dist_(0.0, noise_quat_std_dev),
  axis_dist_(0.0, 1.0),
  noise_dist_(0.0, noise_gyro_std_dev),
  accel_dist_(0.0, noise_accel_std_dev)
{
  // 1) publishers
  imu_publisher_ = this->create_publisher<imu_interfaces::msg::ImuMeasured>("imu_mea", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/imu_state", 1);

  // 2) always create heartbeat timer (100ms), but _publish only when enabled
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IMUnode::heartbeat_timer_callback, this));

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
  // This callback must be called in 900hz
  rclcpp::Time now_time = this->now();
  
  real_imu_data.q[0] = msg->orientation.w;
  real_imu_data.q[1] = msg->orientation.x;
  real_imu_data.q[2] = msg->orientation.y;
  real_imu_data.q[3] = msg->orientation.z;

  real_imu_data.w[0] = msg->angular_velocity.x;
  real_imu_data.w[1] = msg->angular_velocity.y;
  real_imu_data.w[2] = msg->angular_velocity.z;

  std::array<double, 3> local_a = {
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z
  };
  auto global_a = rotate_accel_to_world(real_imu_data.q, local_a);
  //auto global_a = local_a;

  real_imu_data.a[0] = global_a[0];
  real_imu_data.a[1] = global_a[1];
  real_imu_data.a[2] = global_a[2];

  // Store timestamp for later frequency estimation
  imu_stamp_buffer_.push_back(now_time);
}

void IMUnode::PublishMicroStrainMeasurement() { // Timer callbacked as 1kHz
  auto output_msg = imu_interfaces::msg::ImuMeasured();
  output_msg.q = { real_imu_data.q[0], real_imu_data.q[1], real_imu_data.q[2], real_imu_data.q[3] };
  output_msg.w = { real_imu_data.w[0], real_imu_data.w[1], real_imu_data.w[2] };
  output_msg.a = { real_imu_data.a[0], real_imu_data.a[1], real_imu_data.a[2] };
  imu_publisher_->publish(output_msg);

  // imu disconnect(or hz dropping) monitoring
  rclcpp::Time now = this->now();

  while (!imu_stamp_buffer_.empty() && (now - imu_stamp_buffer_.front()) > check_horizon_) {
    imu_stamp_buffer_.pop_front();
  }

  double freq_est = static_cast<double>(imu_stamp_buffer_.size()) / check_horizon_.seconds();

  if (freq_est < 830.0 && hb_enabled_) {
    RCLCPP_WARN(this->get_logger(), "IMU callback freq dropped to %.1f Hz (<830 Hz). Disabling heartbeat.", freq_est);
    hb_enabled_ = false;
  }
  else{RCLCPP_WARN(this->get_logger(), "%.5f", real_imu_data.q[2]);}
}

bool IMUnode::imu_hz_check() {
  rclcpp::Time now = this->now();

  while (!imu_stamp_buffer_.empty() && (now - imu_stamp_buffer_.front()) > check_horizon_) {
    imu_stamp_buffer_.pop_front();
  }
  
  double freq = static_cast<double>(imu_stamp_buffer_.size()) / check_horizon_.seconds();
  return (freq >= 800.0);
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

  // Construct and publish the IMU measurement message
  auto output_msg = imu_interfaces::msg::ImuMeasured();
  output_msg.q = { noisy_q[0], noisy_q[1], noisy_q[2], noisy_q[3] };
  output_msg.w = { noisy_w[0], noisy_w[1], noisy_w[2] };

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUnode>());
  rclcpp::shutdown();
  return 0;
}
