import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}]: {message}'

import rclpy
from rclpy.node import Node
from sbus_interfaces.msg import KillCmd, SbusSignal
from watchdog_interfaces.msg import NodeState

import asyncio
import threading

from .sbus_receiver import SBUSReceiver

class SbusNode(Node):
  def __init__(self):
    super().__init__('sbus_node')

    # Publisher for SBUS node
    self.channel_publisher_ = self.create_publisher(SbusSignal, '/sbus_signal', 1)
    self.killcmd_publisher_ = self.create_publisher(KillCmd, '/sbus_kill', 1)
    self.heartbeat_publisher_ = self.create_publisher(NodeState, '/sbus_state', 1)

    self._hb_state = 0                   # initial value
    self._hb_enabled = False             # heartbeat gate

    # start timer but gate execution until port-connected
    self._hb_timer = self.create_timer(0.1, self._publish_heartbeat)
    
    # Create a new event loop for asyncio and run it on a separate thread
    self.loop = asyncio.new_event_loop()
    self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
    self.thread.start()

    # Run the SBUS receiver in the asyncio event loop
    asyncio.run_coroutine_threadsafe(self.sbus_loop(), self.loop)

  # heartbeat publish method
  def _publish_heartbeat(self):
    if not self._hb_enabled: return

    msg = NodeState()
    msg.state = self._hb_state
    self.heartbeat_publisher_.publish(msg)
    self._hb_state = (self._hb_state + 1) % 256

  async def sbus_loop(self):
    """
    1. Open the serial port.
    2. Wait for SBUS frames (await sbus.get_frame()).
    3. Convert each frame to a ROS2 message and publish it.
    """

    port_name = "/dev/ttyUSB0"
    
    try:
      sbus = await SBUSReceiver.create(port_name)
    except Exception as e:
      self.get_logger().error(f"!! SBUS port Failed : >> {port_name} << !!")
      return
    
    # dip-just one sbus packet (for connection-check)
    first_frame = await sbus.get_frame()
    first_channels = first_frame.get_rx_channels()
    fist_failsafe_status = first_frame.get_failsafe_status()
    if first_channels[9] == 352 and fist_failsafe_status == 0: # not killed && signal is good
      # Only after SBUS is successfully connected, send initial handshake (42)
      self._hb_state = 42
      self._hb_enabled = True
      kill_state = False
    else:
      kill_state = True
      
    while rclpy.ok():
      # Wait for the next SBUS frame from the queue
      frame = await sbus.get_frame()
      channels = frame.get_rx_channels()
      failsafe_status = frame.get_failsafe_status()

      # Log the received channels and failsafe status
      # self.get_logger().info(f"Received channels: {channels}, Failsafe status: {failsafe_status}")

      # Create and populate the SbusSignal&KillCmd message
      msg_channels = SbusSignal()
      msg_channels.ch = channels
      msg_channels.sbus_signal = failsafe_status

      # kill logic
      if channels[9]!=352 or failsafe_status!=0:
        kill_state = True
        if self._hb_enabled: self._hb_enabled = False

      msg_kill = KillCmd()
      msg_kill._kill_activated = kill_state

      # Publish
      self.killcmd_publisher_.publish(msg_kill)
      self.channel_publisher_.publish(msg_channels)

def main(args=None):
  rclpy.init(args=args)
  node = SbusNode()
  try:
    rclpy.spin(node)  # Keep the node alive to process callbacks and publish messages
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

    node.loop.call_soon_threadsafe(node.loop.stop)
    node.thread.join()