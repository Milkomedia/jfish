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

  // Create ROS2 publishers for heartbeat and motor positions
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/dynamixel_state", 1);
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
    mujoco_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("joint_write", 1);

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
  mujoco_publisher_->publish(msg1);

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
  /*  Write  */
  groupSyncWrite_->clearParam();
  
  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      uint8_t param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(arm_des_ppr[i][j])),
        DXL_HIBYTE(DXL_LOWORD(arm_des_ppr[i][j])),
        DXL_LOBYTE(DXL_HIWORD(arm_des_ppr[i][j])),
        DXL_HIBYTE(DXL_HIWORD(arm_des_ppr[i][j]))
      };
      if (!groupSyncWrite_->addParam(DXL_IDS[i][j], param_goal_position)) {
        dnmxl_err_cnt_++;
      }
    }
  }

  if (groupSyncWrite_->txPacket() != COMM_SUCCESS){dnmxl_err_cnt_++;}

  /*  Read  */
  groupSyncRead_->clearParam();

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      if (!groupSyncRead_->addParam(DXL_IDS[i][j])) {dnmxl_err_cnt_++;}
    }
  }

  if (groupSyncRead_->txRxPacket() != COMM_SUCCESS){dnmxl_err_cnt_++;}

  // Retrieve the present position for each motor
  for (size_t i = 0; i < ARM_NUM; ++i) {

    uint8_t id = DXL_IDS[i][0];
    if (groupSyncRead_->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
      int ppr = groupSyncRead_->getData(id, ADDR_PRESENT_POSITION, 4);
      arm_mea[i][0] = static_cast<double>(ppr) * ppr2rad_J1;
    }
    else {dnmxl_err_cnt_++;}

    for (size_t j = 1; j < 5; ++j) {
      uint8_t id = DXL_IDS[i][j];
      if (groupSyncRead_->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
        int ppr = groupSyncRead_->getData(id, ADDR_PRESENT_POSITION, 4);
        arm_mea[i][j] = static_cast<double>(ppr) * ppr2rad;
      }
      else {dnmxl_err_cnt_++;}
    }
  }
  
  /*  Publish  */
  dynamixel_interfaces::msg::JointVal msg;
  for (size_t j = 0; j < 5; ++j) {
    msg.a1_mea[j] = arm_mea[0][j];
    msg.a2_mea[j] = arm_mea[1][j];
    msg.a3_mea[j] = arm_mea[2][j];
    msg.a4_mea[j] = arm_mea[3][j];
  }
  pos_mea_publisher_->publish(msg);
}

bool DynamixelNode::init_Dynamixel() {
  uint8_t dxl_error = 0;

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
        uint8_t id = DXL_IDS[i][j];
        if (packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, 3, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to set operating mode for motor ID " << static_cast<int>(id) << std::endl;
        return false;
      }
      if (packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to enable torque for motor ID " << static_cast<int>(id) << std::endl;
        return false;
      }
    }
  }

  // Set PID gains
  change_position_gain(1, 100, 30, 0);
  change_velocity_gain(1, 100, 20);
  change_position_gain(2, 2500, 270, 0);
  change_velocity_gain(2, 2100, 220);

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
    if (i != 3){
      arm_des_rad[0][i] = -msg->a1_des[i];  // Arm 1
      arm_des_rad[1][i] = -msg->a2_des[i];  // Arm 2
      arm_des_rad[2][i] = -msg->a3_des[i];  // Arm 3
      arm_des_rad[3][i] = -msg->a4_des[i];  // Arm 4
    }
    else{
      arm_des_rad[0][i] = msg->a1_des[i];   // Arm 1
      arm_des_rad[1][i] = msg->a2_des[i];   // Arm 2
      arm_des_rad[2][i] = msg->a3_des[i];   // Arm 3
      arm_des_rad[3][i] = msg->a4_des[i];   // Arm 4
    }
  }

  arm_des_ppr[0][0] = msg->a1_des[0] * rad2ppr_J1 + 2048.0;  // Arm 1
  arm_des_ppr[1][0] = msg->a2_des[0] * rad2ppr_J1 + 2048.0;  // Arm 2
  arm_des_ppr[2][0] = msg->a3_des[0] * rad2ppr_J1 + 2048.0;  // Arm 3
  arm_des_ppr[3][0] = msg->a4_des[0] * rad2ppr_J1 + 2048.0;  // Arm 4
    
  for (uint8_t i = 1; i < 5; ++i) {
    arm_des_ppr[0][i] = msg->a1_des[i] * rad2ppr + 2048.0;   // Arm 1
    arm_des_ppr[1][i] = msg->a2_des[i] * rad2ppr + 2048.0;   // Arm 2
    arm_des_ppr[2][i] = msg->a3_des[i] * rad2ppr + 2048.0;   // Arm 3
    arm_des_ppr[3][i] = msg->a4_des[i] * rad2ppr + 2048.0;   // Arm 4
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