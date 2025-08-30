import sys
import rclpy
from rclpy.node import Node
import threading
import signal

from PyQt5.QtWidgets import QApplication, QWidget, QLineEdit, QLabel, QVBoxLayout, QHBoxLayout, QProgressBar, QRadioButton, QGroupBox
from PyQt5.QtCore import pyqtSignal, QTimer
import pyqtgraph as pg

from controller_interfaces.msg import ControllerDebugVal
from allocator_interfaces.msg import AllocatorDebugVal
from mujoco_interfaces.msg import MujocoHz
from math import pi


class GUI_NODE(Node):
    def __init__(self, gui):
        super().__init__('pid_debugger')
        self.gui = gui

        # Subscription
        self.controller_sub = self.create_subscription(ControllerDebugVal, '/controller_info', self.controller_callback, 1)
        self.allocator_sub = self.create_subscription(AllocatorDebugVal, '/allocator_info', self.allocator_callback, 1)
        self.mujoco_sub = self.create_subscription(MujocoHz, '/mujoco_hz', self.mujoco_callback, 1)

    def controller_callback(self, msg):
        self.gui.controller_update_signal.emit(msg)

    def allocator_callback(self, msg):
        self.gui.allocator_update_signal.emit(msg)

    def mujoco_callback(self, msg):
        self.gui.mujoco_update_signal.emit(msg)

class DebugGUI(QWidget):
    controller_update_signal = pyqtSignal(ControllerDebugVal)
    allocator_update_signal = pyqtSignal(AllocatorDebugVal)
    mujoco_update_signal = pyqtSignal(MujocoHz)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.controller_update_signal.connect(self.controller_update)
        self.allocator_update_signal.connect(self.allocator_update)
        self.mujoco_update_signal.connect(self.mujoco_update)

        self.setWindowTitle("jFish Debugger")
        self.setGeometry(100, 100, 100, 100)
        layout = QVBoxLayout()

        # Create groups and add them to the layout
        top_hbox = QHBoxLayout()
        top_hbox.addWidget(self.create_sbus_group())
        top_hbox.addWidget(self.create_fc_group())
        top_hbox.addWidget(self.create_nodestate_group())
        layout.addLayout(top_hbox)

        # Add the remaining groups to the VBox
        mid_hbox = QHBoxLayout()
        mid_hbox.addWidget(self.create_thruster_group())
        mid_hbox.addWidget(self.create_imu_group())
        mid_hbox.addWidget(self.create_opti_group())
        layout.addLayout(mid_hbox)

        bot_hbox = QHBoxLayout()
        bot_hbox.addWidget(self.create_dynmxl_group())
        layout.addLayout(bot_hbox)

        plot_box = QHBoxLayout()
        plot_box.addWidget(self.create_plot_group())
        layout.addLayout(plot_box)

        self.setLayout(layout)
        
        self.controller_data = {
            "sbus_chnl": [0.0] * 9,
            "pos_cmd": [0.0, 0.0, 0.0, 0.0],
            "wrench_des": [0.0, 0.0, 0.0, 0.0],
            "imu_roll": [0.0, 0.0, 0.0],
            "imu_pitch": [0.0, 0.0, 0.0],
            "imu_yaw": [0.0, 0.0, 0.0],
            "opti_x": [0.0, 0.0, 0.0],
            "opti_y": [0.0, 0.0, 0.0],
            "opti_z": [0.0, 0.0, 0.0]
        }

        self.allocator_data = {
            "pwm": [0.0, 0.0, 0.0, 0.0],
            "thrust": [0.0, 0.0, 0.0, 0.0],
            "a1_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a2_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a3_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a4_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a1_mea": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a2_mea": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a3_mea": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a4_mea": [0.0, 0.0, 0.0, 0.0, 0.0],
            "loop_rate": 0.0
        }

        self.mujoco_data = {
            "loop_rate": 0.0
        }

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # 100ms (10Hz)

    def create_sbus_group(self):
        self.cmd_vals = [] # x,y,z,yaw
        self.sbus_dials_bar = [] # LD, RD
        self.sbus_dials_label = [] # LD, RD
        self.sbus_toggles = [] # SE, SG
        self.sbus_kill = None # KILL

        sbus_group = QGroupBox("SBUS")
        sbus_group.setFixedSize(310, 180)
        sbus_layout = QHBoxLayout()

        # Channel display
        sbus_cmds_layout = QVBoxLayout()
        sbus_cmds = [" x ", " y ", " z ", " ψ"]
        for sbus_cmd in sbus_cmds:  # Channels 1~4
            sbus_cmd_layout = QHBoxLayout()
            cmd_label = QLabel(sbus_cmd)
            sbus_cmd_layout.addWidget(cmd_label)

            cmd_val = QLineEdit()
            cmd_val.setText("  ?")
            cmd_val.setReadOnly(True)
            cmd_val.setFixedWidth(70)
            sbus_cmd_layout.addWidget(cmd_val)
            self.cmd_vals.append(cmd_val)

            cmd_label = QLabel("° ") if sbus_cmd == " ψ" else QLabel("m   ")
            sbus_cmd_layout.addWidget(cmd_label)

            sbus_cmds_layout.addLayout(sbus_cmd_layout)
        sbus_layout.addLayout(sbus_cmds_layout)

        # Toggle switches
        toggle_layout = QVBoxLayout()

        dials = ["LD", "RD"]
        for dial in dials:
            dial_hbox = QHBoxLayout()
            dial_label = QLabel(dial)
            dial_hbox.addWidget(dial_label)

            dial_bar = QProgressBar()
            dial_bar.setFixedWidth(60)
            dial_bar.setRange(0, 100)
            dial_bar.setValue(0)
            dial_hbox.addWidget(dial_bar)
            self.sbus_dials_bar.append(dial_bar)

            dial_val = QLineEdit()
            dial_val.setText("  ?")
            dial_val.setReadOnly(True)
            dial_val.setFixedWidth(30)
            dial_hbox.addWidget(dial_val)
            self.sbus_dials_label.append(dial_val)

            toggle_layout.addLayout(dial_hbox)

        sticks = [" SE ", "  SG "]
        stick_layout = QHBoxLayout()
        for stick in sticks:
            stick_hbox = QHBoxLayout()
            stick_label = QLabel(stick)
            stick_hbox.addWidget(stick_label)

            stick_val = QLineEdit()
            stick_val.setText(" ?")
            stick_val.setReadOnly(True)
            stick_val.setFixedWidth(15)
            stick_hbox.addWidget(stick_val)
            self.sbus_toggles.append(stick_val)

            stick_layout.addLayout(stick_hbox)
        toggle_layout.addLayout(stick_layout)

        kill_layout = QHBoxLayout()
        kill_idx_label =  QLabel("      KILL")
        kill_layout.addWidget(kill_idx_label)

        kill_radio = QRadioButton()
        kill_radio.setAutoExclusive(False)
        kill_layout.addWidget(kill_radio)
        self.sbus_kill = kill_radio

        toggle_layout.addLayout(kill_layout)
        
        sbus_layout.addLayout(toggle_layout)

        sbus_group.setLayout(sbus_layout)
        return sbus_group

    def create_fc_group(self):
        self.fc_wrench_bar = []
        self.fc_wrench_label = []
        
        geom_group = QGroupBox("FC")
        geom_group.setFixedSize(390, 180)
        geom_layout = QVBoxLayout()

        wrench_labels = [" F ", "Mx", "My", "Mz"]
        for label in wrench_labels:
            label_layout = QHBoxLayout()
            
            label_layout.addWidget(QLabel(label))
            
            wrench_bar = QProgressBar()
            wrench_bar.setRange(0, 1000)
            wrench_bar.setValue(0)
            wrench_bar.setFixedWidth(200)
            label_layout.addWidget(wrench_bar)
            self.fc_wrench_bar.append(wrench_bar)

            wrench_val = QLineEdit()
            wrench_val.setText("    ?")
            wrench_val.setReadOnly(True)
            wrench_val.setFixedWidth(60)
            label_layout.addWidget(wrench_val)
            self.fc_wrench_label.append(wrench_val)

            unit_label = QLabel("N") if label == " F " else QLabel("Nm")
            label_layout.addWidget(unit_label)

            geom_layout.addLayout(label_layout)

        geom_group.setLayout(geom_layout)
        return geom_group

    def create_nodestate_group(self):
        self.node_states = []

        nodestate_group_box = QGroupBox("Control Hz")
        nodestate_group_box.setFixedSize(120, 180)
        joint_layout = QVBoxLayout()

        nodes = [" Allocator", "MuJoCo"]
        for node in nodes:
            q_label = QLabel(node)
            q_label.setFixedHeight(50)
            joint_layout.addWidget(q_label)

            row_layout = QHBoxLayout()

            value_label = QLineEdit()
            value_label.setText("0")
            value_label.setReadOnly(True)
            value_label.setFixedWidth(50)
            value_label.setFixedHeight(20)
            self.node_states.append(value_label)
            dummy = QLabel()
            dummy.setFixedWidth(1)
            row_layout.addWidget(dummy)
            row_layout.addWidget(value_label)

            unit_label = QLabel("Hz")
            row_layout.addWidget(unit_label)

            joint_layout.addLayout(row_layout)

            if node == "Controller": joint_layout.addWidget(QLabel()) # dummy

        nodestate_group_box.setLayout(joint_layout)
        return nodestate_group_box

    def create_thruster_group(self):
        self.thrusters_bar = []
        self.thrusters_label = []
        thruster_group_box = QGroupBox("Thrusters")
        thruster_group_box.setFixedSize(220, 180)
        thruster_layout = QVBoxLayout()

        motors = ["f1", "f2", "f3", "f4"]
        for motor in motors:
            motor_layout = QHBoxLayout()
            motor_label = QLabel(motor)
            motor_layout.addWidget(motor_label)

            motor_progress = QProgressBar()
            motor_progress.setRange(0, 100)
            motor_progress.setValue(0)
            self.thrusters_bar.append(motor_progress)
            motor_layout.addWidget(motor_progress)

            motor_val = QLineEdit()
            motor_val.setText("  ?")
            motor_val.setReadOnly(True)
            motor_val.setFixedWidth(60)
            motor_layout.addWidget(motor_val)
            self.thrusters_label.append(motor_val)

            unit_label = QLabel("N")
            unit_label.setFixedWidth(20)
            motor_layout.addWidget(unit_label)

            thruster_layout.addLayout(motor_layout)

        thruster_group_box.setLayout(thruster_layout)
        return thruster_group_box

    def create_sensor_group(self):
        sensor_group = QGroupBox("Sensing")
        sensor_layout = QHBoxLayout()

        sensor_layout.addWidget(self.create_imu_group())
        sensor_layout.addWidget(self.create_opti_group())

        sensor_group.setLayout(sensor_layout)
        return sensor_group
    
    def create_imu_group(self):
        self.imu_meas = []

        imu_group_box = QGroupBox("IMU")
        imu_group_box.setFixedSize(290, 180)
        imu_group_layout = QVBoxLayout()

        imu_label_layout = QHBoxLayout()
        padding_label = QLabel()
        imu_label_layout.addWidget(padding_label)
        imu_labels = ["deg", "deg/s", "deg/s^2"]
        for label in imu_labels:
            rpy_label = QLabel(label)
            rpy_label.setFixedHeight(25)
            imu_label_layout.addWidget(rpy_label)
        imu_group_layout.addLayout(imu_label_layout)

        imu_headers = ["roll", "pitch", "yaw"]
        for header in imu_headers:
            header_layout = QHBoxLayout()
            header_label = QLabel(header)
            header_label.setFixedWidth(40)
            header_layout.addWidget(header_label)

            imu_mea= []
            for _ in imu_labels:
                value_label = QLineEdit()
                value_label.setText("  ?")
                value_label.setReadOnly(True)
                value_label.setFixedWidth(70)
                header_layout.addWidget(value_label)

                imu_mea.append(value_label)

            self.imu_meas.append(imu_mea)
            imu_group_layout.addLayout(header_layout)

        imu_group_box.setLayout(imu_group_layout)
        return imu_group_box

    def create_opti_group(self):
        self.opti_meas = []

        opti_group_box = QGroupBox("Opti")
        opti_group_box.setFixedSize(290, 180)
        opti_group_layout = QVBoxLayout()

        opti_label_layout = QHBoxLayout()
        padding_label = QLabel()
        opti_label_layout.addWidget(padding_label)
        opti_labels = [" m ", "m/s", "m/s^2"]
        for label in opti_labels:
            opti_label = QLabel(label)
            opti_label.setFixedHeight(35)
            opti_label_layout.addWidget(opti_label)
        opti_group_layout.addLayout(opti_label_layout)

        opti_headers = ["x", "y", "z"]
        for header in opti_headers:
            header_layout = QHBoxLayout()
            header_label = QLabel(header)
            header_label.setFixedWidth(40)
            header_layout.addWidget(header_label)

            opti_mea = []
            for _ in opti_labels:
                value_label = QLineEdit()
                value_label.setText("  ?")
                value_label.setReadOnly(True)
                value_label.setFixedWidth(70)
                header_layout.addWidget(value_label)
                opti_mea.append(value_label)

            self.opti_meas.append(opti_mea)
            opti_group_layout.addLayout(header_layout)

        opti_group_box.setLayout(opti_group_layout)
        return opti_group_box

    def create_dynmxl_group(self):
        self.dynmxl_label = []

        joint_group_box = QGroupBox("Dynmxl R/W")
        joint_group_box.setFixedSize(800, 180)
        joint_layout = QVBoxLayout()

        arms = ["A1", "A2", "A3", "A4"]
        for arm in arms:
            arm_des = []
            arm_i_layout = QHBoxLayout()
            arm_i_layout.addWidget(QLabel(arm))
            for i in range(5):
                value_label = QLineEdit()
                value_label.setText("  ?")
                value_label.setReadOnly(True)
                if i == 2: value_label.setFixedWidth(155)
                else: value_label.setFixedWidth(136)
                arm_des.append(value_label)
                arm_i_layout.addWidget(value_label)
            joint_layout.addLayout(arm_i_layout)
            self.dynmxl_label.append(arm_des)

        joint_group_box.setLayout(joint_layout)
        return joint_group_box

    def create_plot_group(self):
        plot_group = QGroupBox()
        plot_group.setFixedSize(800, 300)
        vbox = QVBoxLayout()

        self.plot_widgets = []
        self.plot_curves_ref = []
        self.plot_curves_mea = []

        # plots
        idxs = ["x", "y", "z", "ψ"]
        for idx in idxs:
            hbox = QHBoxLayout()
            idx_label = QLabel(idx)
            idx_label.setFixedHeight(50)
            hbox.addWidget(idx_label)

            plot_widget = pg.PlotWidget()
            plot_widget.setBackground('w')

            plot_item = plot_widget.getPlotItem()
            # Hide x-axis
            plot_item.showAxis('bottom', show=False)

            # Set left axis color to black
            plot_item.getAxis('left').setPen(pg.mkPen(color='k'))
            plot_item.getAxis('left').setTextPen(pg.mkPen(color='k'))

            # Set y-axis range
            if idx == "z": plot_item.setYRange(-0.1, 1.5)
            elif idx == "ψ": plot_item.setYRange(-45., 45.)
            else: plot_item.setYRange(-1.0, 1.0)

            # Two curves: reference (blue) and measurement (red), each with thicker lines
            curve_ref = plot_widget.plot(name="Ref", pen=pg.mkPen(color='b', width=1))
            curve_mea = plot_widget.plot(name="Mea", pen=pg.mkPen(color='r', width=1))

            self.plot_widgets.append(plot_widget)
            self.plot_curves_ref.append(curve_ref)
            self.plot_curves_mea.append(curve_mea)

            hbox.addWidget(plot_widget)
            vbox.addLayout(hbox)

        plot_group.setLayout(vbox)

        # 20 seconds of buffered data at 10 Hz = 200 samples
        self.buffer_size = 200

        self.x_data = list(range(self.buffer_size))
        self.plot_ref_data = [[0.0] * self.buffer_size for _ in range(4)]
        self.plot_mea_data = [[0.0] * self.buffer_size for _ in range(4)]

        return plot_group

    def controller_update(self, msg):
        self.controller_data["sbus_chnl"] = msg.sbus_chnl
        self.controller_data["pos_cmd"] = msg.pos_cmd
        self.controller_data["wrench_des"] = msg.wrench_des
        self.controller_data["imu_roll"] = msg.imu_roll*180/pi
        self.controller_data["imu_pitch"] = msg.imu_pitch*180/pi
        self.controller_data["imu_yaw"] = msg.imu_yaw*180/pi
        self.controller_data["opti_x"] = msg.opti_x
        self.controller_data["opti_y"] = msg.opti_y
        self.controller_data["opti_z"] = msg.opti_z

    def allocator_update(self, msg):
        self.allocator_data["pwm"] = msg.pwm
        self.allocator_data["thrust"] = msg.thrust        
        self.allocator_data["a1_des"] = msg.a1_des / pi * 180
        self.allocator_data["a2_des"] = msg.a2_des / pi * 180
        self.allocator_data["a3_des"] = msg.a3_des / pi * 180
        self.allocator_data["a4_des"] = msg.a4_des / pi * 180
        self.allocator_data["a1_mea"] = msg.a1_mea / pi * 180
        self.allocator_data["a2_mea"] = msg.a2_mea / pi * 180
        self.allocator_data["a3_mea"] = msg.a3_mea / pi * 180
        self.allocator_data["a4_mea"] = msg.a4_mea / pi * 180
        self.allocator_data["loop_rate"] = msg.loop_rate

    def mujoco_update(self, msg):
        self.mujoco_data["loop_rate"] = msg.hz

    def update_gui(self):
        # Update SBUS channel values
        for i, val in enumerate(self.cmd_vals):
            if i != 3: val.setText(f'{self.controller_data["pos_cmd"][i]:.2f}')
            else: val.setText(f'{self.controller_data["pos_cmd"][i]*180/pi:.1f}')

        for idx, toggle_index in enumerate([0, 1]):
            ch_val = self.controller_data["sbus_chnl"][5 + idx]
            if ch_val == 352:
                self.sbus_toggles[toggle_index].setText(" 0")
            elif ch_val == 1024:
                self.sbus_toggles[toggle_index].setText(" 1")
            elif ch_val == 1696:
                self.sbus_toggles[toggle_index].setText(" 2")
            else:
                self.sbus_toggles[toggle_index].setText("  ?")

        for idx, dial_index in enumerate([0, 1]):
            ch_val = self.controller_data["sbus_chnl"][7 + idx]
            mapped_val = (ch_val - 352) / 1344.0
            if mapped_val < 0: break
            self.sbus_dials_label[dial_index].setText(f"{mapped_val:.2f}")
            self.sbus_dials_bar[dial_index].setValue(int(mapped_val * 100))

        kill_val = self.controller_data["sbus_chnl"][4]
        if kill_val == 1696: self.sbus_kill.setChecked(True)
        else: self.sbus_kill.setChecked(False)

        for i, bar in enumerate(self.fc_wrench_bar):
            if i==0:
                bar.setValue(int(self.controller_data["wrench_des"][i]*5))
            else:
                bar.setValue(int(abs(self.controller_data["wrench_des"][i]*100)))

        for i, label in enumerate(self.fc_wrench_label):
            label.setText(f' {self.controller_data["wrench_des"][i]:.2f}')

        # Update thruster values using allocator PWM
        for i, thruster in enumerate(self.thrusters_bar):
            thruster.setValue(int(self.allocator_data['pwm'][i] * 100))

        for i, label in enumerate(self.thrusters_label):
            label.setText(f" {(self.allocator_data['thrust'][i]):.2f}")

        # Update Dynamixel Read/Write values
        for i, row in enumerate(self.dynmxl_label):
            for j, joint in enumerate(row):
                mea = self.allocator_data[f'a{i+1}_mea'][j]
                des = self.allocator_data[f'a{i+1}_des'][j]
                # :6.1f -> width=6, precision=1
                joint.setText(f"{mea:+6.1f}/{des:+6.1f}")

        # Update IMU measurements
        for i, imu_data in enumerate([self.controller_data['imu_roll'], self.controller_data['imu_pitch'], self.controller_data['imu_yaw']]):
            for j, value in enumerate(self.imu_meas[i]):
                value.setText(f"{imu_data[j]:.3f}")

        # Update Opti measurements
        for i, opti_data in enumerate([self.controller_data['opti_x'], self.controller_data['opti_y'], self.controller_data['opti_z']]):
            for j, value in enumerate(self.opti_meas[i]):
                value.setText(f"{opti_data[j]:.3f}")

        # Update node states
        self.node_states[0].setText(f"{int(self.allocator_data['loop_rate'])}")  # Allocator loop rate
        self.node_states[1].setText(f"{ int(self.mujoco_data['loop_rate'])}")  # MuJoCo loop rate

        # Update plots
        for i in range(4):
            if len(self.plot_ref_data[i]) >= self.buffer_size:
                self.plot_ref_data[i].pop(0)
                self.plot_mea_data[i].pop(0)
                
        self.plot_ref_data[0].append(self.controller_data['pos_cmd'][0])
        self.plot_ref_data[1].append(self.controller_data['pos_cmd'][1])
        self.plot_ref_data[2].append(self.controller_data['pos_cmd'][2])
        self.plot_ref_data[3].append(self.controller_data['pos_cmd'][3]*180/pi)

        self.plot_mea_data[0].append(self.controller_data['opti_x'][0])
        self.plot_mea_data[1].append(self.controller_data['opti_y'][0])
        self.plot_mea_data[2].append(self.controller_data['opti_z'][0])
        self.plot_mea_data[3].append(self.controller_data['imu_yaw'][0])
        
        for i in range(4):
            length = len(self.plot_ref_data[i])
            self.plot_curves_ref[i].setData(self.x_data[:length], self.plot_ref_data[i])
            self.plot_curves_mea[i].setData(self.x_data[:length], self.plot_mea_data[i])

def run_ros2_node(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = DebugGUI(None)  # generate GUI first

    # ROS 2 Node generate && connet to GUI
    node = GUI_NODE(gui)
    gui.node = node
    gui.show()

    # gui close when ctrl+C pressed
    def signal_handler(sig, frame):
        rclpy.shutdown()
        app.quit()
    signal.signal(signal.SIGINT, signal_handler)

    ros_thread = threading.Thread(target=run_ros2_node, args=(node,))
    ros_thread.start()

    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()