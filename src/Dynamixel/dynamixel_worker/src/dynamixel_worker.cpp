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
  publisher_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DynamixelNode::Dynamixel_Pub, this));

  // mode-specific init
  this->declare_parameter<std::string>("mode", "None");
  std::string mode;
  this->get_parameter("mode", mode);
  
  if (mode == "real"){
    // Initialize PortHandler and PacketHandler using provided device name and protocol version
    portHandler_ = PortHandler::getPortHandler(device_name.c_str());
    packetHandler_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    real_mode_ = true;

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

  }
  else if (mode == "sim"){
    // Create ROS2 subscriber for joint values
    mujoco_subscriber_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("mujoco_meas", 1, std::bind(&DynamixelNode::mujoco_callback, this, std::placeholders::_1));

    init_dxl_ = true;
    real_mode_ = false;
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

/* for Both */
void DynamixelNode::Dynamixel_Pub() {
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

  if (!init_dxl_) { return; } // before init, not start

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

      groupSyncWrite_->addParam(DXL_IDS[i][j], param_goal_position);
    }
  }

  // groupSyncWrite_->txPacket();
  if (groupSyncWrite_->txPacket() != COMM_SUCCESS) {
    dnmxl_err_cnt_++;
    RCLCPP_WARN(this->get_logger(), "DXL failed2222");
  }


  /*  Read  */
  groupSyncRead_->clearParam();

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j)
      groupSyncRead_->addParam(DXL_IDS[i][j]);
  }

  groupSyncRead_->txRxPacket();

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {
      if (groupSyncRead_->isAvailable(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4)) {
        int ppr = groupSyncRead_->getData(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4);
        arm_mea[i][j] = ppr_2_rad(static_cast<int>(j), ppr);
      }
    }
  }

  /*  Publish  */
  // dynamixel_interfaces::msg::JointVal msg;
  
  // for (size_t j = 0; j < JOINT_NUM; ++j) {
  //   msg.a1_mea[j] = arm_mea[0][j];  // Arm 1
  //   msg.a2_mea[j] = arm_mea[1][j];  // Arm 2
  //   msg.a3_mea[j] = arm_mea[2][j];  // Arm 3
  //   msg.a4_mea[j] = arm_mea[3][j];  // Arm 4

  //   msg.a1_des[j] = arm_des_rad[0][j];  // Arm 1
  //   msg.a2_des[j] = arm_des_rad[1][j];  // Arm 2
  //   msg.a3_des[j] = arm_des_rad[2][j];  // Arm 3
  //   msg.a4_des[j] = arm_des_rad[3][j];  // Arm 4
  // }
  // pos_mea_publisher_->publish(msg);

  if (dnmxl_err_cnt_ > 0) { RCLCPP_WARN(this->get_logger(), "Dynamixel comm errors: %u", dnmxl_err_cnt_); }
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
      if (packetHandler_->write2ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_POSITION_P_GAIN, g.pos_P, &dxl_error) != COMM_SUCCESS) return false;
      if (packetHandler_->write2ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_POSITION_I_GAIN, g.pos_I, &dxl_error) != COMM_SUCCESS) return false;
      if (packetHandler_->write2ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_POSITION_D_GAIN, g.pos_D, &dxl_error) != COMM_SUCCESS) return false;
      if (packetHandler_->write2ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_VELOCITY_P_GAIN, g.vel_P, &dxl_error) != COMM_SUCCESS) return false;
      if (packetHandler_->write2ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_VELOCITY_I_GAIN, g.vel_I, &dxl_error) != COMM_SUCCESS) return false;
    }
  }

  groupSyncWrite_ = new GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION,    4);
  groupSyncRead_  = new GroupSyncRead (portHandler_, packetHandler_, ADDR_PRESENT_POSITION, 4);

  (void)check_shutdown();

  /*  Align Read */
  groupSyncRead_->clearParam();  // clear buffer, ID register

  for (size_t i = 0; i < ARM_NUM; ++i)
    for (size_t j = 0; j < JOINT_NUM; ++j)
      groupSyncRead_->addParam(DXL_IDS[i][j]);

  if (groupSyncRead_->txRxPacket() != COMM_SUCCESS) {  // send read request
    dnmxl_err_cnt_++; 
    RCLCPP_WARN(this->get_logger(), "DXL failed!!");
  }

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {

      if (!groupSyncRead_->isAvailable(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4)) {  // validate read data
        dnmxl_err_cnt_++; 
        RCLCPP_WARN(this->get_logger(), "DXL is not AVAILABLE!!");
      }
      
      int ppr = groupSyncRead_->getData(DXL_IDS[i][j], ADDR_PRESENT_POSITION, 4);

      arm_mea[i][j] = ppr_2_rad(static_cast<int>(j), ppr);
      arm_cmd[i][j] = arm_mea[i][j];
      arm_des_rad[i][j] = arm_mea[i][j];

    }
  }

  /*  Align Write  */
  init_count_ = 0;

  align_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {  // only align timer 10ms 500 step -> 5s

      // LERP
      double alpha = static_cast<double>(init_count_) / static_cast<double>(init_count_max_);  
      // alpha=0 : current pos -> alpha=1 : goal pos
      if (alpha > 1.0) alpha = 1.0; 

      groupSyncWrite_->clearParam();

      for (size_t i = 0; i < ARM_NUM; ++i) {
        for (size_t j = 0; j < JOINT_NUM; ++j) {
          double align_cmd = (1.0 - alpha) * arm_cmd[i][j] + alpha * arm_des_rad[i][j];
          arm_cmd[i][j] = align_cmd;

          int32_t ppr_goal = rad_2_ppr(static_cast<int>(j), align_cmd);
          uint8_t init_goal_position[4] = {
            DXL_LOBYTE(DXL_LOWORD(ppr_goal)),
            DXL_HIBYTE(DXL_LOWORD(ppr_goal)),
            DXL_LOBYTE(DXL_HIWORD(ppr_goal)),
            DXL_HIBYTE(DXL_HIWORD(ppr_goal))
          };

          groupSyncWrite_->addParam(DXL_IDS[i][j], init_goal_position);

          }
        }

      groupSyncWrite_->txPacket();

      ++init_count_;
      if (init_count_ >= init_count_max_) {
        align_timer_.reset();
        init_dxl_ = true;
      }
    }
  );

  return true;

}

bool DynamixelNode::check_shutdown() {
  
  if (!portHandler_ || !packetHandler_) return false;

  uint8_t dxl_error = 0;
  uint8_t hw_error  = 0;  // not error -> status 0

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < JOINT_NUM; ++j) {

      const int comm = packetHandler_->read1ByteTxRx(portHandler_, DXL_IDS[i][j], ADDR_HARDWARE_ERROR_STATUS, &hw_error, &dxl_error);
      
      if (comm == COMM_SUCCESS && hw_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "[DXL ID %u] Hardware Error = 0x%02X", DXL_IDS[i][j], hw_error);

        hb_enabled_ = false;
        RCLCPP_WARN(this->get_logger(), "DXL failed, heartbeat disabled!");
        return true;
      }
    }
  }

  return false;
}

/* for Both */
void DynamixelNode::armchanger_callback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg) {

  for (uint8_t j = 0; j < JOINT_NUM; ++j) {
    arm_des_rad[0][j] = msg->a1_des[j];   // Arm 1
    arm_des_rad[1][j] = msg->a2_des[j];   // Arm 2
    arm_des_rad[2][j] = msg->a3_des[j];   // Arm 3
    arm_des_rad[3][j] = msg->a4_des[j];   // Arm 4
  }

  if (real_mode_) { Dynamixel_Write_Read(); }

}

void DynamixelNode::watchdogCallback(watchdog_interfaces::msg::NodeState::ConstSharedPtr msg){
  bool is_ok = msg->state==13; // Watchdog update (state must be 13.)
  // currently do nothing
  // what i do ? T_T
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