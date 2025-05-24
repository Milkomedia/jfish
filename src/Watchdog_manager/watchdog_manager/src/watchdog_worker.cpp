#include "watchdog_worker.hpp"

using namespace std::chrono_literals;

WatchDogNode::WatchDogNode(): Node("watchdog_node"),
  handshake_checked_(false),failure_detected_(false) {

  start_time_ = this->now();

  std::vector<std::string> node_names = {
    // "optitrack_node",
    "imu_node",
    "sbus_node",
    "arm_changing_node",
    "controller_node",
    "allocator_node",
    "dynamixel_node"
  };

  for (const auto & name : node_names) {
    is_node_initiated_[name]   = false;
    last_state_[name]   = 0;
    last_time_[name]    = start_time_;
  }

  // subscribers
  optitrack_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("/optitrack_state", 1, std::bind(&WatchDogNode::optitrackCallback, this, std::placeholders::_1));
  imu_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("/imu_state", 1, std::bind(&WatchDogNode::imuCallback, this, std::placeholders::_1));
  sbus_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("/sbus_state", 1, std::bind(&WatchDogNode::sbusCallback, this, std::placeholders::_1));
  arm_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("/armchanger_state", 1, std::bind(&WatchDogNode::armCallback, this, std::placeholders::_1));
  controller_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("/controller_state", 1, std::bind(&WatchDogNode::controllerCallback, this, std::placeholders::_1));
  allocator_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("/allocator_state", 1, std::bind(&WatchDogNode::allocatorCallback, this, std::placeholders::_1));
  dynamixel_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("/dynamixel_state", 1, std::bind(&WatchDogNode::dynamixelCallback, this, std::placeholders::_1));

  // publisher
  watchdog_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("/watchdog_state", 1);

  // timer at 20 Hz to enforce handshake & heartbeat timeouts
  pub_timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&WatchDogNode::publishKill, this));

  // timer at 50 Hz to check abnormality
  thinking_timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&WatchDogNode::watchdog_Thinking, this));
}

void WatchDogNode::watchdog_Thinking() {
  if (!handshake_checked_ || failure_detected_){return;}
  
  rclcpp::Time now = this->now();

  // iterate over all nodes and check heartbeat timeout
  for (const auto & entry : last_time_) {
    const std::string & node_name = entry.first;
    const rclcpp::Time & last_time = entry.second;

    double dt = (now - last_time).seconds();
    if (dt > HEARTBEAT_TIMEOUT_SEC) {
      RCLCPP_WARN(this->get_logger(), "[%s]: heartbeat timeout (dt=%.3f > %.3f)", node_name.c_str(), dt, HEARTBEAT_TIMEOUT_SEC);
      failure_detected_ = true;
      
      break;
    }
  }
}

void WatchDogNode::commonCallback(const std::string & node_name, const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
  if (failure_detected_){return;}

  auto now = this->now();

  // initial handshake: expect exactly state==42 ---
  if (!is_node_initiated_[node_name]) {
    if (msg->state == 42) {
      is_node_initiated_[node_name] = true;
      last_state_[node_name] = msg->state;
      last_time_[node_name]  = now;
    }
    else{
      handshake_checked_ = true; // pass handshake timeout check
      failure_detected_ = true;
      RCLCPP_ERROR(this->get_logger(), " >> SHUT DOWN << : [%s] started badly.(get %u)", node_name.c_str(), msg->state);
    }
    return;
  }

  // after handshake: check sequence increment & inter-message timeout ---
  uint8_t expected = static_cast<uint8_t>(last_state_[node_name] + 1);
  if (msg->state != expected) {
    RCLCPP_WARN(this->get_logger(), "[%s]: heartbeat sequence error(expected %u but got %u).", node_name.c_str(), expected, msg->state);
    failure_detected_ = true;
  }

  // update for next check
  last_state_[node_name] = msg->state;
  last_time_[node_name]  = now;
}

// Heartbeat Callback
void WatchDogNode::sbusCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg)
{ commonCallback("sbus_node", msg); }

void WatchDogNode::controllerCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg)
{ commonCallback("controller_node", msg); }

void WatchDogNode::allocatorCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg)
{ commonCallback("allocator_node", msg); }

void WatchDogNode::dynamixelCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg)
{ commonCallback("dynamixel_node", msg); }

void WatchDogNode::imuCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg)
{ commonCallback("imu_node", msg); }

void WatchDogNode::optitrackCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg)
{ commonCallback("optitrack_node", msg); }

void WatchDogNode::armCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg)
{ commonCallback("arm_changing_node", msg); }

void WatchDogNode::publishKill() {
  auto now = this->now();

  // --- handshake timeout enforcement (1 s) --- 핸드셰이크 활성화하는놈
  if (!handshake_checked_ && !failure_detected_) {
    if ((now - start_time_).seconds() > HANDSHAKE_TIMEOUT_SEC) {
      // if any node never sent 42, immediately flag failure
      for (auto & it : is_node_initiated_) {
        if (!it.second) {
          RCLCPP_ERROR(this->get_logger(), "[%s]: not started.", it.first.c_str());
          failure_detected_ = true;
        }
      }

      handshake_checked_ = true;
    }
  }
  
  // --- publish final watchdog_state: 255=kill, 13=ok ---
  watchdog_interfaces::msg::NodeState out;
  out.state = failure_detected_ ? 255 : 13;
  watchdog_publisher_->publish(out);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WatchDogNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}