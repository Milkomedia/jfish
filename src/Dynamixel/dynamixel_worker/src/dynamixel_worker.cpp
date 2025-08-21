#include "dynamixel_worker.hpp"

// using namespace std::chrono_literals;
using namespace dynamixel;

// Constructor: perform port and packet handler initialization here
DynamixelNode::DynamixelNode(const std::string &device_name): Node("dynamixel_node"),
    portHandler_(nullptr),
    packetHandler_(nullptr),
    groupSyncWrite_(nullptr),
    groupSyncRead_(nullptr)
{
  // Create ROS2 subscriber for joint values
  joint_val_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("/joint_cmd", 1, std::bind(&DynamixelNode::armchanger_callback, this, std::placeholders::_1));
  watchdog_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("watchdog_state", 1, std::bind(&DynamixelNode::watchdogCallback, this, std::placeholders::_1));

  // Create ROS2 publishers for heartbeat and motor positions
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/dynamixel_state", 1);
  pos_write_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("/joint_write", 1);
  pos_mea_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("/joint_mea", 1);

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
    
    init_read_ = true;
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s. No initialization performed.", mode.c_str());
    exit(1);
  }

  // initial handshake: immediately send 42 and enable subsequent heartbeat
  hb_state_   = 42;
  hb_enabled_ = true;
  shit_ = false;
}

/* for sim */
void DynamixelNode::Mujoco_Pub() {
  /*  Publish to mujoco  */
  dynamixel_interfaces::msg::JointVal msg1;

  for (size_t j = 0; j < JOINT_NUM; ++j) {
    msg1.a1_des[j] = arm_des_rad[0][j];
    msg1.a2_des[j] = arm_des_rad[1][j];
    msg1.a3_des[j] = arm_des_rad[2][j];
    msg1.a4_des[j] = arm_des_rad[3][j];
  }
  pos_write_publisher_->publish(msg1);

  /*  Publish to allocator  */
  dynamixel_interfaces::msg::JointVal msg2;
  for (size_t j = 0; j < JOINT_NUM; ++j) {
    msg2.a1_des[j] = arm_des_rad[0][j];
    msg2.a2_des[j] = arm_des_rad[1][j];
    msg2.a3_des[j] = arm_des_rad[2][j];
    msg2.a4_des[j] = arm_des_rad[3][j];

    msg2.a1_mea[j] = arm_mea[0][j];
    msg2.a2_mea[j] = arm_mea[1][j];
    msg2.a3_mea[j] = arm_mea[2][j];
    msg2.a4_mea[j] = arm_mea[3][j];
  }
  pos_mea_publisher_->publish(msg2);
}

void DynamixelNode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  
  for (uint8_t j = 0; j < JOINT_NUM; ++j) {
    arm_mea[0][j] = msg->a1_q[j];
    arm_mea[1][j] = msg->a2_q[j];
    arm_mea[2][j] = msg->a3_q[j];
    arm_mea[3][j] = msg->a4_q[j];
  }

}


/* for real */
void DynamixelNode::Dynamixel_Write_Read() {

  if (!init_read_) {
    align_dynamixel();
    return;
  }

  // if (check_shutdown()) return;
  /*  Write  */
  groupSyncWrite_->clearParam();
  
  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {
      
      arm_cmd[i][j] = LPF_ALPHA * arm_des_rad[i][j] + LPF_BETA * arm_cmd[i][j];

      int32_t ppr_goal = rad_2_ppr(static_cast<int>(j), arm_cmd[i][j]);
      uint8_t param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(ppr_goal)),
        DXL_HIBYTE(DXL_LOWORD(ppr_goal)),
        DXL_LOBYTE(DXL_HIWORD(ppr_goal)),
        DXL_HIBYTE(DXL_HIWORD(ppr_goal))
      };
  
      if (!groupSyncWrite_->addParam(DXL_IDS[i][j], param_goal_position)){dnmxl_err_cnt_++;}
    }
  }

  if (groupSyncWrite_->txPacket() != COMM_SUCCESS){dnmxl_err_cnt_++;}


  /*  Read  */
  groupSyncRead_->clearParam();

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j)
      if (!groupSyncRead_->addParam(DXL_IDS[i][j])) dnmxl_err_cnt_++;
  }

  if (groupSyncRead_->txRxPacket() != COMM_SUCCESS) {dnmxl_err_cnt_++;}

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {

      if (!groupSyncRead_->isAvailable(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4)) {
        dnmxl_err_cnt_++;
        continue;
      }

      int ppr = groupSyncRead_->getData(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4);
      arm_mea[i][j] = ppr_2_rad(static_cast<int>(j), ppr);
    }
  }

  /*  Publish  */
  dynamixel_interfaces::msg::JointVal msg;
  
  for (size_t j = 0; j < JOINT_NUM; ++j) {
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

  // if (dnmxl_err_cnt_ > 0) {
  //   RCLCPP_WARN(this->get_logger(), "Dynamixel comm errors: %u", dnmxl_err_cnt_);
  // }
}

void DynamixelNode::align_dynamixel() {
  
  if (!first_cmd_) return;

  if (!align_completed_) {

    groupSyncRead_->clearParam();

    for (size_t i = 0; i < ARM_NUM; ++i)
      for (size_t j = 0; j < JOINT_NUM; ++j)
        groupSyncRead_->addParam(DXL_IDS[i][j]);

    if (groupSyncRead_->txRxPacket() == COMM_SUCCESS) {
      for (size_t i = 0; i < ARM_NUM; ++i) {
        for (size_t j = 0; j < JOINT_NUM; ++j) {
          if (groupSyncRead_->isAvailable(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4)) {
            int32_t ppr = groupSyncRead_->getData(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4);
            arm_mea[i][j] = ppr_2_rad(static_cast<int>(j), ppr);  // power-on first read ppr
          } 
          first_cmd[i][j] = arm_mea[i][j];     
          arm_cmd[i][j]   = first_cmd[i][j]; 
        }
      }
      align_completed_ = true;
      init_count_ = 0;
    } 
    else {return;}
  }

  // linear interpolation
  double alpha = static_cast<double>(init_count_) / init_count_max_;
  if (alpha > 1.0) alpha = 1.0;

  groupSyncWrite_->clearParam();

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {
      double initial_cmd = (1.0 - alpha) * first_cmd[i][j] + alpha * arm_des_rad[i][j];
      arm_cmd[i][j] = initial_cmd;

      int32_t ppr_goal = rad_2_ppr(static_cast<int>(j), initial_cmd);
      uint8_t param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(ppr_goal)),
        DXL_HIBYTE(DXL_LOWORD(ppr_goal)),
        DXL_LOBYTE(DXL_HIWORD(ppr_goal)),
        DXL_HIBYTE(DXL_HIWORD(ppr_goal))
      };
      if (!groupSyncWrite_->addParam(DXL_IDS[i][j], param_goal_position)) dnmxl_err_cnt_++;
    }
  }
  if (groupSyncWrite_->txPacket() != COMM_SUCCESS) dnmxl_err_cnt_++;

  if (++init_count_ >= init_count_max_) {
    init_read_ = true;
  }
}

bool DynamixelNode::init_Dynamixel() {
  uint8_t dxl_error = 0;

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {

      // Set operating mode
      uint8_t mode = (j == 0) ? 4 : 3;  // J1 - Extended Position & Jn - Position
      if (packetHandler_->write1ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_OPERATING_MODE, mode, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to set operating mode for motor ID " << static_cast<int>(DXL_IDS[i][j]) << std::endl;
        return false;
      }

      // Enable torque
      if (packetHandler_->write1ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to enable torque for motor ID " << static_cast<int>(DXL_IDS[i][j]) << std::endl;
        return false;
      }

      // Set gain tune
      const Gains& g = setGains[j];
      change_position_gain(DXL_IDS[i][j], g.pos_P, g.pos_I, g.pos_D);
      change_velocity_gain(DXL_IDS[i][j], g.vel_P, g.vel_I);
    }
  }

  groupSyncWrite_ = new GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION,    4);
  groupSyncRead_  = new GroupSyncRead (portHandler_, packetHandler_, ADDR_PRESENT_POSITION, 4);
  
  syncread_set(groupSyncRead_);
  read_setting_ = true;

  // (void)check_shutdown();
  
  return true;
}

bool DynamixelNode::check_shutdown() { // not running... comming soon //
  
  if (!portHandler_ || !packetHandler_) return false;

  bool shutdown = false;
  uint8_t dxl_error = 0;

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {

      uint8_t hw_error = 0;
      const int comm = packetHandler_->read1ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_HARDWARE_ERROR_STATUS, &hw_error, &dxl_error);
      
      if (comm == COMM_SUCCESS) {
        if (hw_error != 0){
          shutdown = true;
          RCLCPP_ERROR(this->get_logger(), "[DXL ID %u] Hardware Error = 0x%02X", DXL_IDS[i][j], hw_error);
        }
     }
    }
  }

  if (shutdown) {
    hb_enabled_ = false;
    RCLCPP_WARN(this->get_logger(), "DXL failed, heartbeat disabled!");
  }
  return shutdown;
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

  if (!init_read_ && first_cmd_) return;

  for (uint8_t j = 0; j < JOINT_NUM; ++j) {
    arm_des_rad[0][j] = msg->a1_des[j];   // Arm 1
    arm_des_rad[1][j] = msg->a2_des[j];   // Arm 2
    arm_des_rad[2][j] = msg->a3_des[j];   // Arm 3
    arm_des_rad[3][j] = msg->a4_des[j];   // Arm 4
  }

  if (!first_cmd_) first_cmd_ = true;

}

void DynamixelNode::watchdogCallback(watchdog_interfaces::msg::NodeState::ConstSharedPtr msg){
  bool is_ok = msg->state==13; // Watchdog update (state must be 13.)
  // currently do nothing
  if (!is_ok  && !shit_){
    RCLCPP_INFO(this->get_logger(), "\n >> KILL ACTIVATED BY WATCHDOG. [DYNAMIXEL] <<\n");
    shit_ = true;
  }
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