#include "dynamixel_worker.hpp"

// using namespace std::chrono_literals;
using namespace dynamixel;

// Constructor: perform port and packet handler initialization here
DynamixelNode::DynamixelNode(const std::string &device_name): Node("dynamixel_node"),
    portHandler_(nullptr),
    packetHandler_(nullptr),
    groupSyncWrite_(nullptr),
    groupSyncRead_(nullptr),
    last_pub_time_(std::chrono::steady_clock::now()) 
{
  // Create ROS2 subscriber for joint values
  joint_val_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("joint_cmd", 1, std::bind(&DynamixelNode::armchanger_callback, this, std::placeholders::_1));

  // Create ROS2 publishers for heartbeat and motor positions
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/dynamixel_state", 1);
  pos_write_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("joint_write", 1);
  pos_mea_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("joint_mea", 1);

  // Create timer for heartbeat
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamixelNode::heartbeat_timer_callback, this));

  // mode-specific init
  this->declare_parameter<std::string>("mode", "None");
  std::string mode;
  this->get_parameter("mode", mode);
  
  if (mode == "real"){
    // Initialize PortHandler and PacketHandler using provided device name and protocol version
    portHandler_ = PortHandler::getPortHandler(device_name.c_str());
    packetHandler_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port and set baudrate
    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the port: %s", device_name.c_str());
      rclcpp::shutdown();
      exit(1);
    }
    if (!portHandler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
      portHandler_->closePort();
      rclcpp::shutdown();
      exit(1);
    }
    
    // Initialize Dynamixel motors and create GroupSync objects
    if (!init_Dynamixel()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel motors");
      portHandler_->closePort();
      rclcpp::shutdown();
      exit(1);
    }

    // Create timer for Write/Read motor positions
    motor_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DynamixelNode::Dynamixel_Write_Read, this));
  }
  else if (mode == "sim"){
    // Create ROS2 subscriber for joint values
    mujoco_subscriber_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("mujoco_meas", 1, std::bind(&DynamixelNode::mujoco_callback, this, std::placeholders::_1));

    // Create timer for Write/Read motor positions
    motor_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DynamixelNode::Mujoco_Pub, this));
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s. No initialization performed.", mode.c_str());
    exit(1);
  }

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
}

/* for sim */
void DynamixelNode::Mujoco_Pub() {
  /*  Publish to mujoco  */
  dynamixel_interfaces::msg::JointVal msg1;

  for (size_t i = 0; i < 5; ++i) {
    msg1.a1_des[i] = arm_des_rad[0][i];  // Arm 1
    msg1.a2_des[i] = arm_des_rad[1][i];  // Arm 2
    msg1.a3_des[i] = arm_des_rad[2][i];  // Arm 3
    msg1.a4_des[i] = arm_des_rad[3][i];  // Arm 4
  }
  pos_write_publisher_->publish(msg1);

  /*  Publish to allocator  */
  dynamixel_interfaces::msg::JointVal msg2;
  for (size_t i = 0; i < 5; ++i) {
    msg2.a1_des[i] = arm_des_rad[0][i];
    msg2.a2_des[i] = arm_des_rad[1][i];
    msg2.a3_des[i] = arm_des_rad[2][i];
    msg2.a4_des[i] = arm_des_rad[3][i];
    msg2.a1_mea[i] = arm_mea[0][i];
    msg2.a2_mea[i] = arm_mea[1][i];
    msg2.a3_mea[i] = arm_mea[2][i];
    msg2.a4_mea[i] = arm_mea[3][i];
  }
  pos_mea_publisher_->publish(msg2);
}

void DynamixelNode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  for (uint8_t i = 0; i < 5; ++i) {
    arm_mea[0][i] = msg->a1_q[i];   // Arm 1
    arm_mea[1][i] = msg->a2_q[i];   // Arm 2
    arm_mea[2][i] = msg->a3_q[i];   // Arm 3
    arm_mea[3][i] = msg->a4_q[i];   // Arm 4
  }
}

/* for real */
void DynamixelNode::Dynamixel_Write_Read() {

  if (!init_read_) {
    align_dynamixel();
    return;
  }
  
  /*  Write  */
  groupSyncWrite_->clearParam();
  
  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      // Apply LPF in PPR domain
      filtered_des_ppr[i][j] = static_cast<int>(
        0.2 * arm_des_ppr[i][j] + 0.8 * filtered_des_ppr[i][j]
      );
  
      uint8_t param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(filtered_des_ppr[i][j])),
        DXL_HIBYTE(DXL_LOWORD(filtered_des_ppr[i][j])),
        DXL_LOBYTE(DXL_HIWORD(filtered_des_ppr[i][j])),
        DXL_HIBYTE(DXL_HIWORD(filtered_des_ppr[i][j]))
      };
  
      if (!groupSyncWrite_->addParam(DXL_IDS[i][j], param_goal_position)){dnmxl_err_cnt_++;}
    }
  }

  if (groupSyncWrite_->txPacket() != COMM_SUCCESS){dnmxl_err_cnt_++;}

  /*  Read  */
  groupSyncRead_->clearParam();

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j)
      if (!groupSyncRead_->addParam(DXL_IDS[i][j]))
        dnmxl_err_cnt_++;
  }

  if (groupSyncRead_->txRxPacket() != COMM_SUCCESS) {dnmxl_err_cnt_++;}

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      uint8_t id = DXL_IDS[i][j];

      if (!groupSyncRead_->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
        dnmxl_err_cnt_++;
        continue;
      }

      int ppr = groupSyncRead_->getData(id, ADDR_PRESENT_POSITION, 4);

      if (j == 0) {
        arm_mea[i][j] = static_cast<double>(ppr-2048) * ppr2rad_J1;
      } 
      else {
        double rad = static_cast<double>(ppr-2048) * ppr2rad;
        arm_mea[i][j] = (j == 3 || j ==4) ? rad : -rad;
      }
    }
  }

  /*  Publish  */
  dynamixel_interfaces::msg::JointVal msg;
  for (size_t j = 0; j < 5; ++j) {
    msg.a1_mea[j] = arm_mea[0][j];  // Arm 1
    msg.a2_mea[j] = arm_mea[1][j];  // Arm 2
    msg.a3_mea[j] = arm_mea[2][j];  // Arm 3
    msg.a4_mea[j] = arm_mea[3][j];  // Arm 4
    msg.a1_des[j] = arm_des_rad[0][j];  // Arm 1
    msg.a2_des[j] = arm_des_rad[1][j];  // Arm 2
    msg.a3_des[j] = arm_des_rad[2][j];  // Arm 3
    msg.a4_des[j] = arm_des_rad[3][j];  // Arm 4
  }

  pos_mea_publisher_->publish(msg);
}

void DynamixelNode::align_dynamixel() {
  if (init_count_ == 0) {
    groupSyncRead_->clearParam();
    for (size_t i = 0; i < ARM_NUM; ++i) {
      for (size_t j = 0; j < 5; ++j) {
        groupSyncRead_->addParam(static_cast<uint8_t>(DXL_IDS[i][j]));
      }
    }

    if (groupSyncRead_->txRxPacket() != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Initial read failed");
      return;
    }

    for (size_t i = 0; i < ARM_NUM; ++i) {
      for (size_t j = 0; j < 5; ++j) {
        uint8_t id = DXL_IDS[i][j];
        if (groupSyncRead_->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
          int32_t ppr = groupSyncRead_->getData(id, ADDR_PRESENT_POSITION, 4);
          init_read_ppr_[i][j] = ppr;
        } else {
          RCLCPP_WARN(get_logger(), "Motor ID %d not available for initial read", id);
        }
      }
    }

    for (size_t i = 0; i < ARM_NUM; ++i) {
      for (size_t j = 0; j < 5; ++j) {
        double rad = arm_init_rad_[i][j];
        if (j == 0) {
          arm_des_ppr[i][j] = static_cast<int32_t>(rad * rad2ppr_J1 + 2048.0);
        } else if (j == 3) {
          arm_des_ppr[i][j] = static_cast<int32_t>(rad * rad2ppr + 2048.0);
        } else {
          arm_des_ppr[i][j] = static_cast<int32_t>(-rad * rad2ppr + 2048.0);
        }
      }
    }
  }

  double alpha = static_cast<double>(init_count_) / init_count_max_;
  if (alpha > 1.0) alpha = 1.0;

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      filtered_des_ppr[i][j] = static_cast<int32_t>(init_read_ppr_[i][j] * (1.0 - alpha) + arm_des_ppr[i][j] * alpha);
    }
  }

  groupSyncWrite_->clearParam();
  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      uint8_t param_goal[4] = {
        DXL_LOBYTE(DXL_LOWORD(filtered_des_ppr[i][j])),
        DXL_HIBYTE(DXL_LOWORD(filtered_des_ppr[i][j])),
        DXL_LOBYTE(DXL_HIWORD(filtered_des_ppr[i][j])),
        DXL_HIBYTE(DXL_HIWORD(filtered_des_ppr[i][j]))
      };
      groupSyncWrite_->addParam(DXL_IDS[i][j], param_goal);
    }
  }
  groupSyncWrite_->txPacket();

  if (++init_count_ >= init_count_max_) {
    init_read_ = true;
    // RCLCPP_INFO(get_logger(), "Dynamixel initial alignment done.");
  }
}

bool DynamixelNode::init_Dynamixel() {
  uint8_t dxl_error = 0;

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      uint8_t id = DXL_IDS[i][j];

      // Set operating mode
      uint8_t mode = (j == 0) ? 4 : 3;  // J1 - Extended Position & Jn - Position
      if (packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, mode, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to set operating mode for motor ID " << static_cast<int>(id) << std::endl;
        return false;
      }

      // Enable torque
      if (packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to enable torque for motor ID " << static_cast<int>(id) << std::endl;
        return false;
      }

      // Set PID gains
      if (j == 0) { // J1
        change_position_gain(id, 800, 0, 0);
        change_velocity_gain(id, 100, 20);
      }
      else if (j == 1) { // J2
        change_position_gain(id, 2562, 1348, 809);
        change_velocity_gain(id, 1079, 3843);
      }
      else if (j == 2) { // J3
        change_position_gain(id, 2500, 1341, 3843);
        change_velocity_gain(id, 1314, 9102);
      }
      else if (j == 3) { // J4
        change_position_gain(id, 2700, 390, 100);
        change_velocity_gain(id, 2023, 2023);
      }
      else if (j == 4) { // J5
        change_position_gain(id, 700, 0, 0);
        change_velocity_gain(id, 100, 1920);
      }
    }
  }

  groupSyncWrite_ = new GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION, 4);
  groupSyncRead_  = new GroupSyncRead(portHandler_, packetHandler_, ADDR_PRESENT_POSITION, 4);
  return true;
}

void DynamixelNode::change_position_gain(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) {
  uint8_t dxl_error = 0;
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_POSITION_P_GAIN, p_gain, &dxl_error);
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_POSITION_I_GAIN, i_gain, &dxl_error);
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_POSITION_D_GAIN, d_gain, &dxl_error);
}

void DynamixelNode::change_velocity_gain(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain) {
  uint8_t dxl_error = 0;
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_VELOCITY_P_GAIN, p_gain, &dxl_error);
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_VELOCITY_I_GAIN, i_gain, &dxl_error);
}

/* for Both */
void DynamixelNode::armchanger_callback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg) {
  
  for (uint8_t i = 0; i < 5; ++i) {
    arm_des_rad[0][i] = msg->a1_des[i];   // Arm 1
    arm_des_rad[1][i] = msg->a2_des[i];   // Arm 2
    arm_des_rad[2][i] = msg->a3_des[i];   // Arm 3
    arm_des_rad[3][i] = msg->a4_des[i];   // Arm 4
  }

  arm_des_ppr[0][0] = msg->a1_des[0] * rad2ppr_J1 + 2048.0;  // Arm 1
  arm_des_ppr[1][0] = msg->a2_des[0] * rad2ppr_J1 + 2048.0;  // Arm 2
  arm_des_ppr[2][0] = msg->a3_des[0] * rad2ppr_J1 + 2048.0;  // Arm 3
  arm_des_ppr[3][0] = msg->a4_des[0] * rad2ppr_J1 + 2048.0;  // Arm 4
    
  for (uint8_t i = 1; i < 5; ++i) {
    if (i == 3 || i == 4){
      arm_des_ppr[0][i] = msg->a1_des[i] * rad2ppr + 2048.0;  // Arm 1
      arm_des_ppr[1][i] = msg->a2_des[i] * rad2ppr + 2048.0;  // Arm 2
      arm_des_ppr[2][i] = msg->a3_des[i] * rad2ppr + 2048.0;  // Arm 3
      arm_des_ppr[3][i] = msg->a4_des[i] * rad2ppr + 2048.0;  // Arm 4
    }
    else {
      arm_des_ppr[0][i] = -msg->a1_des[i] * rad2ppr + 2048.0;   // Arm 1
      arm_des_ppr[1][i] = -msg->a2_des[i] * rad2ppr + 2048.0;   // Arm 2
      arm_des_ppr[2][i] = -msg->a3_des[i] * rad2ppr + 2048.0;   // Arm 3
      arm_des_ppr[3][i] = -msg->a4_des[i] * rad2ppr + 2048.0;   // Arm 4
    }
  }

  // for (uint8_t i = 0; i < 4; ++i) {
  //   arm_des_rad[i][0] = 0.;
  //   arm_des_rad[i][1] = -0.2114;
  //   arm_des_rad[i][2] = 1.4717;
  //   arm_des_rad[i][3] = 0.3105;
  //   arm_des_rad[i][4] = 0.;
  // }

  // arm_des_ppr[0][0] = 0. * rad2ppr_J1 + 2048.0;  // Arm 1
  // arm_des_ppr[1][0] = 0. * rad2ppr_J1 + 2048.0;  // Arm 2
  // arm_des_ppr[2][0] = 0. * rad2ppr_J1 + 2048.0;  // Arm 3
  // arm_des_ppr[3][0] = 0. * rad2ppr_J1 + 2048.0;  // Arm 4

  // arm_des_ppr[0][1] = 0.2114 * rad2ppr + 2048.0;   // Arm 1
  // arm_des_ppr[1][1] = 0.2114 * rad2ppr + 2048.0;   // Arm 2
  // arm_des_ppr[2][1] = 0.2114 * rad2ppr + 2048.0;   // Arm 3
  // arm_des_ppr[3][1] = 0.2114 * rad2ppr + 2048.0;   // Arm 4

  // arm_des_ppr[0][2] = -1.4717 * rad2ppr + 2048.0;   // Arm 1
  // arm_des_ppr[1][2] = -1.4717 * rad2ppr + 2048.0;   // Arm 2
  // arm_des_ppr[2][2] = -1.4717 * rad2ppr + 2048.0;   // Arm 3
  // arm_des_ppr[3][2] = -1.4717 * rad2ppr + 2048.0;   // Arm 4

  // arm_des_ppr[0][3] = 0.3105 * rad2ppr + 2048.0;   // Arm 1
  // arm_des_ppr[1][3] = 0.3105 * rad2ppr + 2048.0;   // Arm 2
  // arm_des_ppr[2][3] = 0.3105 * rad2ppr + 2048.0;   // Arm 3
  // arm_des_ppr[3][3] = 0.3105 * rad2ppr + 2048.0;   // Arm 4

  // arm_des_ppr[0][4] = -0.042 * rad2ppr + 2048.0;   // Arm 1
  // arm_des_ppr[1][4] = 0.023 * rad2ppr + 2048.0;   // Arm 2
  // arm_des_ppr[2][4] = -0.049 * rad2ppr + 2048.0;   // Arm 3
  // arm_des_ppr[3][4] = 0.058 * rad2ppr + 2048.0;   // Arm 4

}

void DynamixelNode::heartbeat_timer_callback() {
  // gate until handshake done
  if (!hb_enabled_) {return;}

  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = hb_state_;
  heartbeat_publisher_->publish(state_msg);

  // uint8 overflow wraps automatically
  hb_state_ = static_cast<uint8_t>(hb_state_ + 1);
}
  
DynamixelNode::~DynamixelNode() {
  if (groupSyncWrite_){delete groupSyncWrite_;}
  if (groupSyncRead_){delete groupSyncRead_;}
  if (portHandler_){portHandler_->closePort();}
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DynamixelNode>(DEVICE_NAME);
    rclcpp::spin(node);
  }
  catch (const std::exception &e) {std::cerr << "Exception caught in main: " << e.what() << std::endl;}
  rclcpp::shutdown();
  return 0;
}