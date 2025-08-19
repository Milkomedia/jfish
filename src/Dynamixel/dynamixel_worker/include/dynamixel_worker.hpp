#ifndef DYNAMIXEL_WORKER_HPP
#define DYNAMIXEL_WORKER_HPP

#include <cstdio>
#include <memory>
#include <string>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "mujoco_interfaces/msg/mu_jo_co_meas.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <iostream>
#include <chrono>
#include <stdexcept>

// Dynamixel Addresses
#define ADDR_OPERATING_MODE      11
#define ADDR_TORQUE_ENABLE       64
#define ADDR_GOAL_POSITION       116
#define ADDR_PRESENT_POSITION    132
#define ADDR_POSITION_P_GAIN     84
#define ADDR_POSITION_I_GAIN     82
#define ADDR_POSITION_D_GAIN     80
#define ADDR_VELOCITY_P_GAIN     78
#define ADDR_VELOCITY_I_GAIN     76

#define PROTOCOL_VERSION 2.0
#define BAUDRATE         4000000
#define DEVICE_NAME      "/dev/ttyUSB1"

#define ARM_NUM   4
#define JOINT_NUM 5

constexpr double PI = 3.1415926535897932384626433832706;
constexpr double tilted_rad = 5 / 180.0 * PI; // [rad]

constexpr double LPF_ALPHA = 0.20;
constexpr double LPF_BETA  = 1.0 - LPF_ALPHA;

constexpr double rad2ppr_J1 = 6.25 * 2048.0 / PI;
constexpr double ppr2rad_J1 = PI / 2048.0 / 6.25;
constexpr double rad2ppr = 2048.0 / PI;
constexpr double ppr2rad = PI / 2048.0;

constexpr std::array<std::array<uint8_t, JOINT_NUM>, ARM_NUM> DXL_IDS = {{
  { 1,  2,  3,  4,  5}, // Arm 1
  { 6,  7,  8,  9, 10}, // Arm 2
  {11, 12, 13, 14, 15}, // Arm 3
  {16, 17, 18, 19, 20}  // Arm 4
}};

constexpr std::array<int, JOINT_NUM> JointSign = {
  +1,  // J1
  -1,  // J2
  -1,  // J3
  +1,  // J4
  +1   // J5
};

struct Gains {
  uint16_t pos_P;
  uint16_t pos_I;
  uint16_t pos_D;
  uint16_t vel_P;
  uint16_t vel_I;
};

constexpr std::array<Gains, JOINT_NUM> setGains = {{
  {  800,    0,    0,  100,  20  },  // J1 P I D (pos) / P I (vel)
  { 2562, 1348,  809, 1079, 3843 },  // J2
  { 2500, 1341, 3843, 1314, 9102 },  // J3
  { 2700,  390,  100, 2023, 2023 },  // J4
  {  700,    0,    0,  100, 1920 },  // J5
}};

inline int32_t rad_2_ppr(int j, double rad) {
  if (j == 0) return static_cast<int32_t>( JointSign[j] * rad * rad2ppr_J1 + 2048.0 );
  return static_cast<int32_t>( JointSign[j] * rad * rad2ppr + 2048.0 );
}

inline double ppr_2_rad(int j, int32_t ppr) {
  if (j == 0) return static_cast<double>( JointSign[j] * (ppr - 2048) * ppr2rad_J1 );
  return static_cast<double>( JointSign[j] * (ppr - 2048) * ppr2rad );
}

inline void syncread_set (dynamixel::GroupSyncRead* read_add_param) {
  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {
      read_add_param->addParam(static_cast<uint8_t>(DXL_IDS[i][j]));
    }
  }
}


class DynamixelNode : public rclcpp::Node {
public:
  DynamixelNode(const std::string &device_name);
  virtual ~DynamixelNode();
  bool init_Dynamixel();

private:
  void Dynamixel_Write_Read();
  void Mujoco_Pub();
  void armchanger_callback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg);
  void mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg);
  void change_position_gain(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);
  void change_velocity_gain(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain);
  void heartbeat_timer_callback();
  void align_dynamixel();

  // ROS2 communication
  rclcpp::Subscription<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_val_subscriber_;
  rclcpp::Subscription<mujoco_interfaces::msg::MuJoCoMeas>::SharedPtr mujoco_subscriber_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<dynamixel_interfaces::msg::JointVal>::SharedPtr pos_write_publisher_;
  rclcpp::Publisher<dynamixel_interfaces::msg::JointVal>::SharedPtr pos_mea_publisher_;
  rclcpp::TimerBase::SharedPtr motor_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Dynamixel SDK objects as class member variables
  dynamixel::PortHandler* portHandler_;
  dynamixel::PacketHandler* packetHandler_;
  dynamixel::GroupSyncWrite* groupSyncWrite_;
  dynamixel::GroupSyncRead*  groupSyncRead_;

  const size_t init_count_max_{500};
  size_t init_count_{0};
  bool init_read_{false}; 
  bool read_setting_{false};

  const double arm_init_rad_[4][5] = { // [rad]
    {0., 0.095993089, 0.67544228, 0.806341947, -tilted_rad},
    {0., 0.095993089, 0.67544228, 0.806341947,  tilted_rad},
    {0., 0.095993089, 0.67544228, 0.806341947, -tilted_rad},
    {0., 0.095993089, 0.67544228, 0.806341947,  tilted_rad}
  };

  double init_read_ppr[ARM_NUM][JOINT_NUM] = {};
  double arm_des_rad[ARM_NUM][JOINT_NUM] = {};
  double arm_des_ppr[ARM_NUM][JOINT_NUM] = {};
  double lpf_des_ppr[ARM_NUM][JOINT_NUM] = {};
  double init_des_ppr[ARM_NUM][JOINT_NUM] = {};

  // mujoco or dynamixel read
  double arm_mea[4][5] = { // [rad]
    {0., 0.095993089, 0.67544228, 0.806341947, -tilted_rad},
    {0., 0.095993089, 0.67544228, 0.806341947,  tilted_rad},
    {0., 0.095993089, 0.67544228, 0.806341947, -tilted_rad},
    {0., 0.095993089, 0.67544228, 0.806341947,  tilted_rad}
  };

  uint16_t dnmxl_err_cnt_ = 0;
  
  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag
};

#endif // DYNAMIXEL_WORKER_HPP