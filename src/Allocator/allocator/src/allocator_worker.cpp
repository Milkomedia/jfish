#include "allocator_worker.hpp"

AllocatorWorker::AllocatorWorker()
 : Node("allocator_node"),
   zeta_(Eigen::Vector4d(zeta, -zeta, zeta, -zeta)),
   q_B0_(Eigen::Vector4d(0.25*M_PI, 0.75*M_PI, -0.75*M_PI, -0.25*M_PI)) {

  // -------- ROS parameters (override 가능) --------
  // Tilt assist
  tilt_limit_rad_        = this->declare_parameter<double>("tilt_limit_rad",        tilt_limit_rad_);
  tilt_rate_limit_rad_s_ = this->declare_parameter<double>("tilt_rate_limit_rad_s", tilt_rate_limit_rad_s_);
  yaw_eps_               = this->declare_parameter<double>("yaw_eps",               yaw_eps_);
  w_fx_                  = this->declare_parameter<double>("w_fx",                  w_fx_);
  w_fy_                  = this->declare_parameter<double>("w_fy",                  w_fy_);
  w_mz_                  = this->declare_parameter<double>("w_mz",                  w_mz_);
  lambda_tilt_           = this->declare_parameter<double>("lambda_tilt",           lambda_tilt_);
  tilt_joint_index_      = this->declare_parameter<int>("tilt_joint_index",         tilt_joint_index_);
  tilt_axis_index_       = this->declare_parameter<int>("tilt_axis_index",          tilt_axis_index_);

  // 초기 EE 틸트 각 (deg) 파라미터: 기본값 A1:-5, A2:+5, A3:-5, A4:+5
  const double base_deg_a1 = this->declare_parameter<double>("tilt_base_deg.a1", -5.0);
  const double base_deg_a2 = this->declare_parameter<double>("tilt_base_deg.a2",  5.0);
  const double base_deg_a3 = this->declare_parameter<double>("tilt_base_deg.a3", -5.0);
  const double base_deg_a4 = this->declare_parameter<double>("tilt_base_deg.a4",  5.0);

  // -------- DH table --------
  DH_params_ <<
    //   a      alpha     d   theta0
       0.120,   0.0,     0.0,  0.0,   // B->0
       0.134,   M_PI/2,  0.0,  0.0,   // 0->1
       0.115,   0.0,     0.0,  0.0,   // 1->2
       0.110,   0.0,     0.0,  0.0,   // 2->3
       0.024,   M_PI/2,  0.0,  0.0,   // 3->4
       0.068,   0.0,     0.0,  0.0;   // 4->5

  pwm_.setZero();
  C1_.setZero();
  C2_.setZero();

  // -------- Subscribers --------
  controller_subscriber_ = this->create_subscription<controller_interfaces::msg::ControllerOutput>("/controller_output", 1, std::bind(&AllocatorWorker::controllerCallback, this, std::placeholders::_1));
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

  // initial heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
  heartbeat_state_ = 0;
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

  // yaw wrench conversion
  tauz_bar_ = lpf_alpha_*Wrench(2) + lpf_beta_*tauz_bar_;
  double tauz_r = Wrench(2) - tauz_bar_;
  double tauz_r_saturated = std::clamp(tauz_r, tauz_min, tauz_max);
  double tauz_t = tauz_bar_ + tauz_r - tauz_r_saturated;
  RCLCPP_INFO(this->get_logger(), ">> %f\t%f<<", tauz_r_saturated, tauz_t);
  // for now, tauz_r_saturated and tauz_t are not using yet.(영준이형이 해줄거야)

  // calculate Allocation Matrix A 
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
  if (lu.isInvertible()) {C1_ = lu.solve(Wrench);}
  else {C1_ = (A.transpose()*A + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A.transpose()*Wrench);}

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
  tilt_msg.th1 = C2_(0);
  tilt_msg.th2 = C2_(1);
  tilt_msg.th3 = C2_(2);
  tilt_msg.th4 = C2_(3);
  tilt_angle_publisher_->publish(tilt_msg);
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AllocatorWorker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



// void AllocatorWorker::controllerCallback(const controller_interfaces::msg::ControllerOutput::SharedPtr msg) {

//   // ===== Loop Time (moving average) =====
//   rclcpp::Time current_callback_time = this->now();
//   double dt = (current_callback_time - last_callback_time_).seconds();
//   last_callback_time_ = current_callback_time;
//   dt_sum_ = dt_sum_ - dt_buffer_[buffer_index_] + dt;
//   dt_buffer_[buffer_index_] = dt;
//   buffer_index_ = (buffer_index_ + 1) % buffer_size_;
//   double avg_dt = dt_sum_ / static_cast<double>(buffer_size_);
//   if (avg_dt > 1e-6) filtered_frequency_ = 1.0 / avg_dt;

//   // ===== Inputs =====
//   Eigen::Vector4d Wrench; Eigen::Vector3d CoM;
//   Wrench << msg->moment[0], msg->moment[1], msg->moment[2], msg->force;   // [Mx My Mz Fz]
//   CoM    << msg->com_bias[0], msg->com_bias[1], msg->com_bias[2];

//   // ===== yaw wrench LPF & residual =====
//   tauz_bar_ = lpf_alpha_ * Wrench(2) + lpf_beta_ * tauz_bar_;
//   const double tauz_r        = Wrench(2) - tauz_bar_;
//   const double tauz_r_sat    = std::clamp(tauz_r, tauz_min, tauz_max);
//   const double tauz_t = tauz_r - tauz_r_sat;

//   // ====== 1) 현 자세(FK)로 A 구성, 1차 할당 ======
//   Eigen::Matrix<double,4,12> A1; A1.setZero();
//   Eigen::Matrix<double,12,4> A2; A2.setZero();
//   const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

//   std::array<Eigen::Vector3d,4> r_arr, e_arr, u_arr;

//   for (int arm = 0; arm < 4; ++arm) {
//     // --- 전체 FK (end-effector) ---
//     Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//     for (int i = 0; i <= 5; ++i) {
//       const double q = (i == 0) ? q_B0_(arm) : arm_mea_[arm][i-1];
//       T *= compute_DH(DH_params_(i,0), DH_params_(i,1), DH_params_(i,2), DH_params_(i,3) + q);
//     }
//     const Eigen::Vector3d r = T.block<3,1>(0,3) - CoM;  // 위치 (월드)
//     const Eigen::Vector3d e = T.block<3,1>(0,0);        // 추력 방향 (월드, end-effector x축)

//     // --- 틸트 조인트 회전축(world) ---
//     Eigen::Matrix4d T_pre = Eigen::Matrix4d::Identity();
//     for (int i = 0; i < tilt_joint_index_; ++i) {
//       const double q = (i == 0) ? q_B0_(arm) : arm_mea_[arm][i-1];
//       T_pre *= compute_DH(DH_params_(i,0), DH_params_(i,1), DH_params_(i,2), DH_params_(i,3) + q);
//     }
//     Eigen::Vector3d axis_local = Eigen::Vector3d::UnitZ();
//     if (tilt_axis_index_ == 0) axis_local = Eigen::Vector3d::UnitX();
//     else if (tilt_axis_index_ == 1) axis_local = Eigen::Vector3d::UnitY();
//     const Eigen::Vector3d u = T_pre.block<3,3>(0,0) * axis_local;

//     const Eigen::Matrix3d S = skew(r);
//     const Eigen::Matrix3d M = S + zeta_(arm) * I3;

//     A1.block<3,3>(0, 3*arm) = M;     // 모멘트 기여
//     A1(3, 3*arm + 2) = 1.0;          // Fz 합산 (z성분)
//     A2.block<3,1>(3*arm, arm) = e;   // thrust 방향

//     r_arr[arm] = r; e_arr[arm] = e; u_arr[arm] = u;
//   }

//   const Eigen::Matrix4d A = A1 * A2;

//   // --- 1차 thrust 해 ---
//   Eigen::Vector4d thrust_raw = Eigen::Vector4d::Zero();
//   {
//     Eigen::FullPivLU<Eigen::Matrix4d> lu(A);
//     if (lu.isInvertible())
//       thrust_raw = lu.solve(Wrench);
//     else
//       thrust_raw = (A.transpose()*A + 1e-8*Eigen::Matrix4d::Identity()).ldlt()
//                     .solve(A.transpose()*Wrench);
//   }

//   // --- pwm 포화 적용 후 달성 렌치 평가 ---
//   Eigen::Vector4d pwm_tmp, Fsat;
//   for (int i=0;i<4;++i) {
//     const double val = std::max(0.0, (thrust_raw(i) - pwm_beta_) / pwm_alpha_);
//     pwm_tmp(i) = std::clamp(std::sqrt(val), 0.0, 1.0);
//     Fsat(i)    = pwm_alpha_ * pwm_tmp(i) * pwm_tmp(i) + pwm_beta_;
//   }
//   const Eigen::Vector4d Wrench_hat = A * Fsat;
//   const double yaw_shortfall = Wrench(2) - Wrench_hat(2);

//   // ====== 2) yaw 부족 시, 틸트 보조 (민감도 G) ======
//   bool did_tilt_assist = false;
//   if (std::fabs(yaw_shortfall) > yaw_eps_) {
//     // G(3x4): [dFx/dth; dFy/dth; dMz/dth]
//     Eigen::Matrix<double,3,4> G; G.setZero();

//     for (int i=0;i<4;++i) {
//       const Eigen::Vector3d dfi = Fsat(i) * (u_arr[i].cross(e_arr[i]));                    // δf_i
//       const Eigen::Vector3d dMi = (skew(r_arr[i]) + zeta_(i)*Eigen::Matrix3d::Identity())  // δM_i
//                                 * dfi;
//       G(0,i) = dfi.x();
//       G(1,i) = dfi.y();
//       G(2,i) = dMi.z();
//     }

//     // 목표: Fx,Fy는 0 유지, Mz 부족만 메움
//     Eigen::Vector3d b; b << 0.0, 0.0, yaw_shortfall;

//     Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
//     W(0,0)=w_fx_; W(1,1)=w_fy_; W(2,2)=w_mz_;

//     const Eigen::Matrix<double,4,4> H = G.transpose() * W * W * G
//                                        + lambda_tilt_ * Eigen::Matrix4d::Identity();
//     const Eigen::Matrix<double,4,1> g = G.transpose() * W * W * b;

//     Eigen::Matrix<double,4,1> dtheta = H.ldlt().solve(g);

//     // 각/속도 제한 (절대각 setpoint C2_에 누적)
//     const double step_lim = tilt_rate_limit_rad_s_ * std::max(1e-3, avg_dt);
//     for (int i=0;i<4;++i){
//       dtheta(i) = std::clamp(dtheta(i), -step_lim, step_lim);
//       C2_(i) = std::clamp(C2_(i) + dtheta(i), -tilt_limit_rad_, tilt_limit_rad_);
//     }
//     did_tilt_assist = true;
//   }

//   // ====== 3) 틸트 반영한 예측 자세로 재할당 ======
//   Eigen::Vector4d thrust_final = thrust_raw;
//   Eigen::Vector4d pwm_final    = pwm_tmp;

//   if (did_tilt_assist) {
//     Eigen::Matrix<double,4,12> A1p; A1p.setZero();
//     Eigen::Matrix<double,12,4> A2p; A2p.setZero();

//     for (int arm=0; arm<4; ++arm) {
//       Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//       for (int i=0;i<=5;++i) {
//         double q;
//         if (i==0) q = q_B0_(arm);
//         else {
//           if (i==tilt_joint_index_) q = C2_(arm); // 틸트는 내부 목표(절대각)
//           else                      q = arm_mea_[arm][i-1];
//         }
//         T *= compute_DH(DH_params_(i,0), DH_params_(i,1), DH_params_(i,2), DH_params_(i,3)+q);
//       }
//       const Eigen::Vector3d r = T.block<3,1>(0,3) - CoM;
//       const Eigen::Vector3d e = T.block<3,1>(0,0);
//       const Eigen::Matrix3d M = skew(r) + zeta_(arm)*Eigen::Matrix3d::Identity();

//       A1p.block<3,3>(0,3*arm) = M;
//       A1p(3, 3*arm + 2) = 1.0;
//       A2p.block<3,1>(3*arm, arm) = e;
//     }

//     const Eigen::Matrix4d Ap = A1p*A2p;

//     Eigen::FullPivLU<Eigen::Matrix4d> lu(Ap);
//     if (lu.isInvertible())
//       thrust_final = lu.solve(Wrench);
//     else
//       thrust_final = (Ap.transpose()*Ap + 1e-8*Eigen::Matrix4d::Identity()).ldlt()
//                       .solve(Ap.transpose()*Wrench);

//     for (int i=0;i<4;++i) {
//       const double val = std::max(0.0, (thrust_final(i) - pwm_beta_) / pwm_alpha_);
//       pwm_final(i) = std::clamp(std::sqrt(val), 0.0, 1.0);
//     }
//   }

//   pwm_ = pwm_final;
//   C1_  = thrust_final;

//   // ====== 4) Publish ======
//   auto pwm_msg = allocator_interfaces::msg::PwmVal();
//   pwm_msg.pwm1 = pwm_(0);
//   pwm_msg.pwm2 = pwm_(1);
//   pwm_msg.pwm3 = pwm_(2);
//   pwm_msg.pwm4 = pwm_(3);
//   pwm_publisher_->publish(pwm_msg);

//   auto tilt_msg = allocator_interfaces::msg::TiltAngleVal();
//   tilt_msg.th1 = C2_(0);
//   tilt_msg.th2 = C2_(1);
//   tilt_msg.th3 = C2_(2);
//   tilt_msg.th4 = C2_(3);
//   tilt_angle_publisher_->publish(tilt_msg);
// }

// void AllocatorWorker::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg)  {
//   for (uint8_t i = 0; i < 5; ++i)
//   {
//     arm_mea_[0][i] = msg->a1_mea[i];
//     arm_mea_[1][i] = msg->a2_mea[i];
//     arm_mea_[2][i] = msg->a3_mea[i];
//     arm_mea_[3][i] = msg->a4_mea[i];

//     arm_des_[0][i] = msg->a1_des[i];
//     arm_des_[1][i] = msg->a2_des[i];
//     arm_des_[2][i] = msg->a3_des[i];
//     arm_des_[3][i] = msg->a4_des[i];
//   }
// }

// void AllocatorWorker::heartbeat_timer_callback() {
//   if (!hb_enabled_) {return;}
//   watchdog_interfaces::msg::NodeState state_msg;
//   state_msg.state = hb_state_;
//   heartbeat_publisher_->publish(state_msg);
//   hb_state_ = static_cast<uint8_t>(hb_state_ + 1);
// }

// void AllocatorWorker::debugging_timer_callback()
// {
//   allocator_interfaces::msg::AllocatorDebugVal info_msg;
//   for (int i = 0; i < 4; i++)
//   {
//     info_msg.pwm[i]    = pwm_(i);
//     info_msg.thrust[i] = C1_(i);
//   }

//   for (size_t i = 0; i < 5; ++i)
//   {
//     info_msg.a1_des[i] = arm_des_[0][i];
//     info_msg.a2_des[i] = arm_des_[1][i];
//     info_msg.a3_des[i] = arm_des_[2][i];
//     info_msg.a4_des[i] = arm_des_[3][i];

//     info_msg.a1_mea[i] = arm_mea_[0][i];
//     info_msg.a2_mea[i] = arm_mea_[1][i];
//     info_msg.a3_mea[i] = arm_mea_[2][i];
//     info_msg.a4_mea[i] = arm_mea_[3][i];
//   }

//   info_msg.loop_rate = filtered_frequency_;
//   debug_val_publisher_->publish(info_msg);
// }

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<AllocatorWorker>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

