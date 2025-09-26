import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from mujoco_interfaces.msg import MotorThrust, MuJoCoMeas, MujocoHz, MujocoState
from dynamixel_interfaces.msg import JointVal

import mujoco
import mujoco.viewer
from mujoco import mjtObj
from rclpy.executors import SingleThreadedExecutor
import threading
import time
from collections import deque

import signal
import numpy as np

class MuJoCoSimulatorNode(Node):
    def __init__(self, executor):
        super().__init__('mujoco_node')
        self.executor = executor

        # Get the package directory and initialize MuJoCo model
        package_share_dir = get_package_share_directory('mujoco_sim')
        scene_file_path = os.path.join(package_share_dir, 'xml', 'scene.xml')
        self.model = mujoco.MjModel.from_xml_path(scene_file_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.001  # 1kHz simulation frequency

        # xml body&site ids..
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b"BODY")
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE,  b"imu")

        # — thread‐safe access to self.data
        self.data_lock = threading.Lock()

        # hz measurements
        self._sim_times = deque(maxlen=1200)
        now = time.monotonic()
        self._sim_times.append(now)

        # Initialize motor thrust and moments
        self.motor_thrusts = [0.0, 0.0, 0.0, 0.0]
        self.motor_moments = [0.0, 0.0, 0.0, 0.0]
        self.a1_des = [0., 0.095993089, 0.67544228, 0.806341947, 0.0]
        self.a2_des = [0., 0.095993089, 0.67544228, 0.806341947, 0.0]
        self.a3_des = [0., 0.095993089, 0.67544228, 0.806341947, 0.0]
        self.a4_des = [0., 0.095993089, 0.67544228, 0.806341947, 0.0]

        # Start MuJoCo viewer
        self.viewer_thread = threading.Thread(target=self.run_viewer, daemon=True)
        self.data_lock = threading.Lock()
        self.viewer_thread.start()

        # Publish /mujoco_state (Hz)
        self.create_timer(0.1, self.publish_mujoco_state)

        # publishers
        self.mujoco_meas_publisher = self.create_publisher(MuJoCoMeas, '/mujoco_meas', 1)
        self.mujoco_hz_publisher = self.create_publisher(MujocoHz, '/mujoco_hz', 1)
        self.mujoco_state_publisher = self.create_publisher(MujocoState, '/mujoco_state', 1)

        # Start MuJoCo SIM
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.sim_thread.start()

        # Start Subscriber after some delay
        self._delayed_timer = self.create_timer(3.0, self._start_sub)

        self.get_logger().info("\n\n")

    def _start_sub(self):
        self._delayed_timer.cancel()
        
        self.create_subscription(MotorThrust, '/motor_write', self.motor_thrust_callback, 1)
        self.create_subscription(JointVal, '/joint_write', self.joint_arm_callback, 1)

    def motor_thrust_callback(self, msg: MotorThrust):
        self.motor_thrusts = msg.force
        self.motor_moments = msg.moment
    
    def joint_arm_callback(self, msg: JointVal):
        self.a1_des = msg.a1_des
        self.a2_des = msg.a2_des
        self.a3_des = msg.a3_des
        self.a4_des = msg.a4_des

    def sim_loop(self):

        dt = self.model.opt.timestep

        while rclpy.ok():
            now = time.monotonic()
            # — measure actual sim Hz over 1 s window
            self._sim_times.append(now)
            cutoff = now - 1.0
            while self._sim_times and self._sim_times[0] < cutoff:
                self._sim_times.popleft()

            with self.data_lock:
                # — set control inputs
                self.data.ctrl[0:4]  = self.motor_thrusts
                self.data.ctrl[4:8]  = self.motor_moments
                self.data.ctrl[8:13] = self.a1_des
                self.data.ctrl[13:18]= self.a2_des
                self.data.ctrl[18:23]= self.a3_des
                self.data.ctrl[23:28]= self.a4_des

                # — perform one simulation step (1 kHz)
                mujoco.mj_step(self.model, self.data)

                # — publish current state (same as before)
                self.publish_mujoco_meas()

            # — maintain steady 1 kHz loop
            elapsed = time.monotonic() - now
            if elapsed < dt: time.sleep(dt - elapsed)
            # else: self.get_logger().warn(f"Step overrun: {elapsed:.6f}s")

    def run_viewer(self):
        # Launch MuJoCo viewer in passive mode
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and rclpy.ok():
                time.sleep(0.02)
                with self.data_lock:
                    viewer.sync()  # Sync the viewer to the current simulation state

        if rclpy.ok(): rclpy.shutdown()

    def publish_mujoco_meas(self):
        # Extract state information
        pos = self.data.qpos[:3]
        q = self.data.qpos[3:7]
        vel = self.data.qvel[:3]
        w = self.data.qvel[3:6]
        acc = self.data.qacc[:3]

        # --- Joint slices based on XML ordering ---
        base_offset    = 7
        joints_per_arm = 5

        # Arm3 joints are qpos[7:12]
        a3_q = list(self.data.qpos[ base_offset                    : base_offset + joints_per_arm ])
        # Arm2 joints are qpos[12:17]
        a2_q = list(self.data.qpos[ base_offset + 1*joints_per_arm : base_offset + 2*joints_per_arm ])
        # Arm1 joints are qpos[17:22]
        a1_q = list(self.data.qpos[ base_offset + 2*joints_per_arm : base_offset + 3*joints_per_arm ])
        # Arm4 joints are qpos[22:27]
        a4_q = list(self.data.qpos[ base_offset + 3*joints_per_arm : base_offset + 4*joints_per_arm ])
        # ------------------------------------------------

        # Create and publish the message
        msg = MuJoCoMeas(
            q=[q[0], q[1], q[2], q[3]],
            w=[w[0], w[1], w[2]],
            pos=[pos[0], pos[1], pos[2]],
            vel=[vel[0], vel[1], vel[2]],
            acc=[acc[0], acc[1], acc[2]],
            a1_q = a1_q,
            a2_q = a2_q,
            a3_q = a3_q,
            a4_q = a4_q
        )
        self.mujoco_meas_publisher.publish(msg)

        # ==================  CoM_bias (about imu site)  ==================

        # a) global CoM
        x_coms   = self.data.xipos[1:]
        masses   = self.model.body_mass[1:]
        com_world = np.average(x_coms, axis=0, weights=masses)

        # b) IMU pos·attitude
        imu_pos = self.data.site_xpos[self.site_id]
        imu_R   = self.data.site_xmat[self.site_id].reshape(3, 3)

        # c) CoM bias wrt. imu site
        bias_world = com_world - imu_pos
        bias_imu   = imu_R.T @ bias_world

        # ==================  J (3×3 inertia about global COM)  ==================
        # 1) Compute global center of mass (world frame)
        masses     = self.model.body_mass[1:]          # exclude body 0 (world)
        x_coms     = self.data.xipos[1:]               # each body CoM in world frame
        com_world  = np.average(x_coms, axis=0, weights=masses)

        # 2) Compute system inertia about CoM (world frame)
        J_world = np.zeros((3, 3))
        eye3    = np.eye(3)

        for i in range(1, self.model.nbody):
            # a) Rotate local principal inertia to world frame
            m_i   = self.model.body_mass[i]
            R_i   = self.data.xmat[i].reshape(3, 3)         # body-to-world rotation
            I_i   = np.diag(self.model.body_inertia[i])     # principal inertia at CoM
            I_w   = R_i @ I_i @ R_i.T                       # inertia in world axes

            # b) Parallel-axis term: distance from global CoM to link CoM
            r_i   = self.data.xipos[i] - com_world          # vector from system CoM to link CoM
            J_pa  = m_i * ((np.dot(r_i, r_i) * eye3) - np.outer(r_i, r_i))

            # c) Sum up
            J_world += I_w + J_pa

        # 3) Transform J into IMU‐aligned frame with origin at CoM
        #    - imu_R_world: rotation from IMU frame to world frame
        #    - so J_imu = R^T * J_world * R 
        imu_R_world = self.data.site_xmat[self.site_id].reshape(3, 3)
        J_imu       = imu_R_world.T @ J_world @ imu_R_world

        # publish to controller Node
        state_msg = MujocoState()
        state_msg.inertia = J_imu.flatten().tolist()
        state_msg.com_bias = bias_imu.tolist()
        self.mujoco_state_publisher.publish(state_msg)

    def publish_mujoco_state(self):
        # measured_hz = len(self._sim_times) / (self._sim_times[-1] - self._sim_times[0])

        if len(self._sim_times) < 2 or self._sim_times[-1] == self._sim_times[0]:
            measured_hz = 0.0
        else:
            duration = self._sim_times[-1] - self._sim_times[0]
            measured_hz = len(self._sim_times) / duration

        msg = MujocoHz()
        msg.hz = measured_hz
        self.mujoco_hz_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = MuJoCoSimulatorNode(executor)

    signal.signal(signal.SIGINT, signal.default_int_handler)

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        if 'Unable to convert call argument to Python object' in str(e): pass
        else: raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()