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

class PIDDebugger(Node):
    def __init__(self, gui):
        super().__init__('pid_debugger')
        self.gui = gui

        # Subscription
        self.controller_sub = self.create_subscription(ControllerDebugVal, '/controller_info', self.controller_callback, 1)
        self.allocator_sub = self.create_subscription(AllocatorDebugVal, '/allocator_info', self.allocator_callback, 1)

    def controller_callback(self, msg):
        self.gui.controller_update_signal.emit(msg)

    def allocator_callback(self, msg):
        self.gui.allocator_update_signal.emit(msg)

class DebugGUI(QWidget):
    controller_update_signal = pyqtSignal(ControllerDebugVal)
    allocator_update_signal = pyqtSignal(AllocatorDebugVal)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.controller_update_signal.connect(self.controller_update)
        self.allocator_update_signal.connect(self.allocator_update)

        self.setWindowTitle("jFish Debugger")
        self.setGeometry(100, 100, 100, 100)
        layout = QVBoxLayout()

        # Create groups and add them to the layout
        top_hbox = QHBoxLayout()
        top_hbox.addWidget(self.create_sbus_group())
        top_hbox.addWidget(self.create_pid_group())
        layout.addLayout(top_hbox)

        # Add the remaining groups to the VBox
        mid_hbox = QHBoxLayout()
        mid_hbox.addWidget(self.create_allocation_group())
        mid_hbox.addWidget(self.create_nodestate_group())
        layout.addLayout(mid_hbox)

        bot_hbox = QHBoxLayout()
        bot_hbox.addWidget(self.create_imu_group())
        bot_hbox.addWidget(self.create_opti_group())
        bot_hbox.addWidget(self.create_dmxl_group())
        layout.addLayout(bot_hbox)

        plot_box = QHBoxLayout()
        plot_box.addWidget(self.create_plot_group())
        layout.addLayout(plot_box)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # 100ms (10Hz)
        
        self.controller_data = {
            "sbus_chnl": [0] * 9,
            "des_pos": [0] * 4,
            "pid_mx": [0.0, 0.0, 0.0, 0.0],
            "pid_my": [0.0, 0.0, 0.0, 0.0],
            "pid_mz": [0.0, 0.0],
            "pid_f": [0.0, 0.0],
            "imu_roll": [0.0, 0.0],
            "imu_pitch": [0.0, 0.0],
            "imu_yaw": [0.0, 0.0],
            "opti_x": [0.0, 0.0],
            "opti_y": [0.0, 0.0],
            "opti_z": [0.0, 0.0],
            "a1_mea": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a2_mea": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a3_mea": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a4_mea": [0.0, 0.0, 0.0, 0.0, 0.0]
        }

        self.allocator_data = {
            "pwm": [0.0] * 4,
            "a1_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a2_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a3_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "a4_des": [0.0, 0.0, 0.0, 0.0, 0.0],
            "loop_rate": 0.0
        }

    def create_sbus_group(self):
        self.channel_vals = []
        self.sbus_channels = []
        self.sbus_toggles = []

        sbus_group = QGroupBox("SBUS")
        sbus_layout = QVBoxLayout()

        # Channel display
        channel_outer_layout = QVBoxLayout()
        channels = ["cmd_r ", "cmd_p ", "cmd_y ", "cmd_z "]
        for channel in channels:  # Channels 1~4
            channel_layout = QHBoxLayout()
            channel_label = QLabel(channel)
            channel_progress = QProgressBar()
            channel_progress.setRange(0, 1000)
            channel_progress.setValue(0)
            self.sbus_channels.append(channel_progress)
            channel_val = QLineEdit()
            channel_val.setText("  ?")
            channel_val.setReadOnly(True)
            channel_val.setFixedWidth(90)
            self.channel_vals.append(channel_val)

            channel_layout.addWidget(channel_label)
            channel_layout.addWidget(channel_progress)
            channel_layout.addWidget(channel_val)

            channel_outer_layout.addLayout(channel_layout)
        sbus_layout.addLayout(channel_outer_layout)

        # Toggle switches (circular style)
        toggle_layout = QVBoxLayout()
        left_toggle_vbox = QHBoxLayout()

        toggles = ["SE", "SG", "LD", "RD", "KILL"]
        for toggle in toggles:
            toggle_hbox = QHBoxLayout()
            toggle_index_label = QLabel(toggle)

            if toggle == "KILL":
                toggle_val = QRadioButton()
                toggle_val.setAutoExclusive(False)
                self.sbus_toggles.append(toggle_val)

            elif toggle == "SE" or toggle == "SG":
                toggle_val = QLineEdit()
                toggle_val.setText("  ?")
                toggle_val.setReadOnly(True)
                toggle_val.setFixedWidth(40)
                self.sbus_toggles.append(toggle_val)

            else:
                toggle_val = QLineEdit()
                toggle_val.setText("  ?")
                toggle_val.setReadOnly(True)
                toggle_val.setFixedWidth(70)
                self.sbus_toggles.append(toggle_val)

            toggle_hbox.addWidget(toggle_index_label)
            toggle_hbox.addWidget(toggle_val)
            left_toggle_vbox.addLayout(toggle_hbox)

        # Layout configuration for toggle switches
        toggle_layout.addLayout(left_toggle_vbox)
        sbus_layout.addLayout(toggle_layout)
        sbus_group.setLayout(sbus_layout)
        return sbus_group

    def create_pid_group(self):
        self.pid_midval_r = []
        self.pid_midval_p = []
        self.pid_midval_y = []
        self.pid_midval_z = []
        
        pid_group = QGroupBox("PID")
        pid_layout = QVBoxLayout()
        
        labels_layout = QHBoxLayout()
        padding_label = QLabel()
        padding_label.setFixedWidth(65)
        labels_layout.addWidget(padding_label)

        pid_labels = ["r", "p", "y", "z"]
        for label in pid_labels:
            label_layout = QHBoxLayout()
            idx_label = QLabel(label)
            idx_label.setFixedHeight(50)
            label_layout.addWidget(idx_label)
            
            if label == "r":
                for _ in range(4):
                    pid_label = QLineEdit("0.0")
                    pid_label.setReadOnly(True)
                    pid_label.setFixedHeight(50)
                    pid_label.setFixedWidth(170)
                    label_layout.addWidget(pid_label)
                    self.pid_midval_r.append(pid_label)
            elif label == "p":
                for _ in range(4):
                    pid_label = QLineEdit("0.0")
                    pid_label.setReadOnly(True)
                    pid_label.setFixedHeight(50)
                    pid_label.setFixedWidth(170)
                    label_layout.addWidget(pid_label)
                    self.pid_midval_p.append(pid_label)
            elif label == "y":
                padding_label = QLabel()
                padding_label.setFixedWidth(340)
                label_layout.addWidget(padding_label)
                for _ in range(2):
                    pid_label = QLineEdit("0.0")
                    pid_label.setReadOnly(True)
                    pid_label.setFixedHeight(50)
                    pid_label.setFixedWidth(170)
                    label_layout.addWidget(pid_label)
                    self.pid_midval_y.append(pid_label)
            else:
                for _ in range(2):
                    pid_label = QLineEdit("0.0")
                    pid_label.setReadOnly(True)
                    pid_label.setFixedHeight(50)
                    pid_label.setFixedWidth(170)
                    label_layout.addWidget(pid_label)
                    self.pid_midval_z.append(pid_label)
                padding_label = QLabel()
                padding_label.setFixedWidth(350)
                label_layout.addWidget(padding_label)

            pid_layout.addLayout(label_layout)

        pid_group.setLayout(pid_layout)
        return pid_group
    
    def create_allocation_group(self):
        alloc_group = QGroupBox("Allocation")
        alloc_layout = QHBoxLayout()

        # Add Thruster and Joint groups
        alloc_layout.addWidget(self.create_thruster_group())
        alloc_layout.addWidget(self.create_joint_group())

        alloc_group.setLayout(alloc_layout)
        return alloc_group

    def create_thruster_group(self):
        self.thrusters = []
        thruster_group_box = QGroupBox("Thrusters")
        thruster_layout = QVBoxLayout()

        motors = ["f1", "f2", "f3", "f4"]
        for motor in motors:
            motor_layout = QHBoxLayout()
            motor_label = QLabel(motor)
            motor_progress = QProgressBar()
            motor_progress.setRange(0, 100)
            motor_progress.setValue(0)
            self.thrusters.append(motor_progress)

            motor_layout.addWidget(motor_label)
            motor_layout.addWidget(motor_progress)
            thruster_layout.addLayout(motor_layout)

        thruster_group_box.setLayout(thruster_layout)
        return thruster_group_box

    def create_joint_group(self):
        self.joint_des = []

        joint_group_box = QGroupBox("Joint Write")
        joint_layout = QVBoxLayout()

        arms = ["A1", "A2", "A3", "A4"]
        for arm in arms:
            arm_des = []
            arm_i_layout = QHBoxLayout()
            arm_i_layout.addWidget(QLabel(arm))
            for _ in range(5):
                value_label = QLineEdit()
                value_label.setText("  ?")
                value_label.setReadOnly(True)
                value_label.setFixedWidth(100)
                arm_des.append(value_label)
                arm_i_layout.addWidget(value_label)
            joint_layout.addLayout(arm_i_layout)
            self.joint_des.append(arm_des)

        joint_group_box.setLayout(joint_layout)
        return joint_group_box

    def create_nodestate_group(self):
        self.node_states = []

        nodestate_group_box = QGroupBox("Control Hz")
        nodestate_group_box.setFixedWidth(180)
        joint_layout = QVBoxLayout()

        nodes = [" Allocator"]
        for node in nodes:
            q_label = QLabel(node)
            q_label.setFixedHeight(35)
            joint_layout.addWidget(q_label)

            row_layout = QHBoxLayout()

            value_label = QLineEdit()
            value_label.setText("0")
            value_label.setReadOnly(True)
            value_label.setFixedWidth(85)
            value_label.setFixedHeight(45)
            self.node_states.append(value_label)
            dummy = QLabel()
            dummy.setFixedWidth(1)
            row_layout.addWidget(dummy)
            row_layout.addWidget(value_label)

            unit_label = QLabel("Hz")
            unit_label.setFixedHeight(20)
            row_layout.addWidget(unit_label)

            joint_layout.addLayout(row_layout)

            if node == "Controller": joint_layout.addWidget(QLabel()) # dummy

        nodestate_group_box.setLayout(joint_layout)
        return nodestate_group_box

    def create_sensor_group(self):
        sensor_group = QGroupBox("Sensing")
        sensor_layout = QHBoxLayout()

        sensor_layout.addWidget(self.create_imu_group())
        sensor_layout.addWidget(self.create_opti_group())
        sensor_layout.addWidget(self.create_dmxl_group())

        sensor_group.setLayout(sensor_layout)
        return sensor_group
    
    def create_imu_group(self):
        self.imu_meas = []

        imu_group_box = QGroupBox("IMU")
        imu_group_layout = QVBoxLayout()

        imu_label_layout = QHBoxLayout()
        padding_label = QLabel()
        padding_label.setFixedWidth(85)
        imu_label_layout.addWidget(padding_label)
        imu_labels = ["rad", "rad/s"]
        for label in imu_labels:
            rpy_label = QLabel(label)
            rpy_label.setFixedHeight(35)
            imu_label_layout.addWidget(rpy_label)
        imu_group_layout.addLayout(imu_label_layout)

        imu_headers = ["roll", "pitch", "yaw"]
        for header in imu_headers:
            header_layout = QHBoxLayout()
            header_label = QLabel(header)
            header_label.setFixedWidth(70)
            header_layout.addWidget(header_label)

            imu_mea= []
            for _ in imu_labels:
                value_label = QLineEdit()
                value_label.setText("  ?")
                value_label.setReadOnly(True)
                value_label.setFixedWidth(110)
                header_layout.addWidget(value_label)

                imu_mea.append(value_label)

            self.imu_meas.append(imu_mea)
            imu_group_layout.addLayout(header_layout)

        imu_group_box.setLayout(imu_group_layout)
        imu_group_box.setFixedWidth(350)
        return imu_group_box

    def create_opti_group(self):
        self.opti_meas = []

        opti_group_box = QGroupBox("Opti")
        opti_group_layout = QVBoxLayout()

        opti_label_layout = QHBoxLayout()
        padding_label = QLabel()
        padding_label.setFixedWidth(85)
        opti_label_layout.addWidget(padding_label)
        opti_labels = [" m ", "m/s"]
        for label in opti_labels:
            opti_label = QLabel(label)
            opti_label.setFixedHeight(35)
            opti_label_layout.addWidget(opti_label)
        opti_group_layout.addLayout(opti_label_layout)

        opti_headers = ["x", "y", "z"]
        for header in opti_headers:
            header_layout = QHBoxLayout()
            header_label = QLabel(header)
            header_label.setFixedWidth(50)
            header_layout.addWidget(header_label)

            opti_mea = []
            for _ in opti_labels:
                value_label = QLineEdit()
                value_label.setText("  ?")
                value_label.setReadOnly(True)
                value_label.setFixedWidth(110)
                header_layout.addWidget(value_label)
                opti_mea.append(value_label)

            self.opti_meas.append(opti_mea)
            opti_group_layout.addLayout(header_layout)


        opti_group_box.setLayout(opti_group_layout)
        opti_group_box.setFixedWidth(350)
        return opti_group_box

    def create_dmxl_group(self):
        self.joint_meas = []

        dmxl_group_box = QGroupBox("Joint Read")
        dmxl_group_layout = QVBoxLayout()

        num_layout = QHBoxLayout()
        padding_label = QLabel()
        padding_label.setFixedWidth(1)
        num_layout.addWidget(padding_label)
        
        arms = ["A1", "A2", "A3", "A4"]
        for arm in arms:
            arm_mea = []
            arm_i_layout = QHBoxLayout()
            arm_i_layout.addWidget(QLabel(arm))
            for i in range(5):
                value_label = QLineEdit()
                value_label.setText("  ?")
                value_label.setReadOnly(True)
                value_label.setFixedWidth(100)
                arm_mea.append(value_label)
                arm_i_layout.addWidget(value_label)
            dmxl_group_layout.addLayout(arm_i_layout)
            self.joint_meas.append(arm_mea)

        dmxl_group_box.setLayout(dmxl_group_layout)
        return dmxl_group_box

    def controller_update(self, msg):
        self.controller_data["sbus_chnl"] = msg.sbus_chnl
        self.controller_data["des_pos"] = msg.des_pos
        self.controller_data["pid_mx"] = msg.pid_mx
        self.controller_data["pid_my"] = msg.pid_my
        self.controller_data["pid_mz"] = msg.pid_mz
        self.controller_data["pid_f"] = msg.pid_f
        self.controller_data["imu_roll"] = msg.imu_roll
        self.controller_data["imu_pitch"] = msg.imu_pitch
        self.controller_data["imu_yaw"] = msg.imu_yaw
        self.controller_data["opti_x"] = msg.opti_x
        self.controller_data["opti_y"] = msg.opti_y
        self.controller_data["opti_z"] = msg.opti_z
        self.controller_data["a1_mea"] = msg.a1_mea
        self.controller_data["a2_mea"] = msg.a2_mea
        self.controller_data["a3_mea"] = msg.a3_mea
        self.controller_data["a4_mea"] = msg.a4_mea

    def allocator_update(self, msg):
        self.allocator_data["pwm"] = msg.pwm
        self.allocator_data["a1_des"] = msg.a1_des
        self.allocator_data["a2_des"] = msg.a2_des
        self.allocator_data["a3_des"] = msg.a3_des
        self.allocator_data["a4_des"] = msg.a4_des
        self.allocator_data["loop_rate"] = msg.loop_rate

    def create_plot_group(self):
        plot_group = QGroupBox()
        vbox = QVBoxLayout()

        self.plot_widgets = []
        self.plot_curves_ref = []
        self.plot_curves_mea = []

        # r, p, y, z plots
        idxs = ["r", "p", "y", "z"]
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
            if idx == "z": plot_item.setYRange(0.0, 1.5)
            elif idx == "y": plot_item.setYRange(-3.14, 3.14)
            else: plot_item.setYRange(-0.06, 0.06)

            # Two curves: reference (blue) and measurement (red), each with thicker lines
            curve_ref = plot_widget.plot(name="Ref", pen=pg.mkPen(color='b', width=3))
            curve_mea = plot_widget.plot(name="Mea", pen=pg.mkPen(color='r', width=3))

            self.plot_widgets.append(plot_widget)
            self.plot_curves_ref.append(curve_ref)
            self.plot_curves_mea.append(curve_mea)

            hbox.addWidget(plot_widget)
            vbox.addLayout(hbox)

        plot_group.setLayout(vbox)

        # 5 seconds of buffered data at 10 Hz = 50 samples
        self.plot_ref_data = [[] for _ in range(4)]  # [r, p, y, z] desired values
        self.plot_mea_data = [[] for _ in range(4)]  # [r, p, y, z] measured values
        self.x_data = list(range(50))

        return plot_group

    def update_gui(self):
        # Update SBUS channel values
        for i, (channel, val) in enumerate(zip(self.sbus_channels, self.channel_vals)):
            if i==3: channel.setValue(round((self.controller_data['sbus_chnl'][i] - 352.) / 1.344))
            else: channel.setValue(500 + round((self.controller_data['sbus_chnl'][i] - 1024.) / 1.344))
                
            val.setText(f"{self.controller_data['des_pos'][i]:.2f}")

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

        for idx, toggle_index in enumerate([2, 3]):
            ch_val = self.controller_data["sbus_chnl"][7 + idx]
            mapped_val = (ch_val - 352) / 1344.0
            self.sbus_toggles[toggle_index].setText(f"{mapped_val:.2f}")

        kill_val = self.controller_data["sbus_chnl"][4]
        if kill_val == 1696: self.sbus_toggles[4].setChecked(True)
        else: self.sbus_toggles[4].setChecked(False)
        
        # Update PID vals
        for i, label in enumerate(self.pid_midval_r):
            if i == 0: label.setText(self.format_pid_value(self.controller_data['pid_mx'][i], " m/s"))
            elif i == 1: label.setText(self.format_pid_value(self.controller_data['pid_mx'][i], " rad"))
            elif i == 2: label.setText(self.format_pid_value(self.controller_data['pid_mx'][i], " rad/s"))
            else: label.setText(self.format_pid_value(self.controller_data['pid_mx'][i], " N.m"))

        for i, label in enumerate(self.pid_midval_p):
            if i == 0: label.setText(self.format_pid_value(self.controller_data['pid_my'][i], " m/s"))
            elif i == 1: label.setText(self.format_pid_value(self.controller_data['pid_my'][i], " rad"))
            elif i == 2: label.setText(self.format_pid_value(self.controller_data['pid_my'][i], " rad/s"))
            else: label.setText(self.format_pid_value(self.controller_data['pid_my'][i], " N.m"))

        for i, label in enumerate(self.pid_midval_y):
            if i == 0: label.setText(self.format_pid_value(self.controller_data['pid_mz'][i], " rad/s"))
            elif i == 1: label.setText(self.format_pid_value(self.controller_data['pid_mz'][i], " N.m"))

        for i, label in enumerate(self.pid_midval_z):
            if i == 0: label.setText(self.format_pid_value(self.controller_data['pid_f'][i], " m/s"))
            elif i == 1: label.setText(self.format_pid_value(self.controller_data['pid_f'][i], " rad"))

        # Update thruster values using allocator PWM
        for i, thruster in enumerate(self.thrusters):
            thruster.setValue(int(self.allocator_data['pwm'][i] * 100))

        # Update Joint Desired values
        for i, row in enumerate(self.joint_des):
            for j, joint in enumerate(row):
                joint.setText(f"{self.allocator_data[f'a{i+1}_des'][j]:.2f}")

        # Update Joint Measured values
        for i, row in enumerate(self.joint_meas):
            for j, joint in enumerate(row):
                joint.setText(f"{self.controller_data[f'a{i+1}_mea'][j]:.2f}")

        # Update IMU measurements
        for i, imu_data in enumerate([self.controller_data['imu_roll'], self.controller_data['imu_pitch'], self.controller_data['imu_yaw']]):
            for j, value in enumerate(self.imu_meas[i]):
                value.setText(f"{imu_data[j]:.3f}")

        # Update Opti measurements
        for i, opti_data in enumerate([self.controller_data['opti_x'], self.controller_data['opti_y'], self.controller_data['opti_z']]):
            for j, value in enumerate(self.opti_meas[i]):
                value.setText(f"{opti_data[j]:.3f}")

        # Update node states
        self.node_states[0].setText(str(round(self.allocator_data['loop_rate'])))  # Controller loop rate

        # Update plots
        for i in range(4):
            if len(self.plot_ref_data[i]) >= 50:
                self.plot_ref_data[i].pop(0)
                self.plot_mea_data[i].pop(0)
                
        self.plot_ref_data[0].append(self.controller_data['des_pos'][0])
        self.plot_ref_data[1].append(self.controller_data['des_pos'][1])
        self.plot_ref_data[2].append(self.controller_data['des_pos'][2])
        self.plot_ref_data[3].append(self.controller_data['des_pos'][3])

        self.plot_mea_data[0].append(self.controller_data['imu_roll'][0])
        self.plot_mea_data[1].append(self.controller_data['imu_pitch'][0])
        self.plot_mea_data[2].append(self.controller_data['imu_yaw'][0])
        self.plot_mea_data[3].append(self.controller_data['opti_z'][0])
        
        for i in range(4):
            length = len(self.plot_ref_data[i])
            self.plot_curves_ref[i].setData(self.x_data[:length], self.plot_ref_data[i])
            self.plot_curves_mea[i].setData(self.x_data[:length], self.plot_mea_data[i])

    def format_pid_value(self, value, unit, total_width=13):
        sign = '-' if value < 0 else '+'
        num_str = f"{abs(value):.3f}"
        available = total_width - len(sign) - len(unit)
        centered_num = num_str.center(available)
        return f"{sign}{centered_num}{unit}"

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
    node = PIDDebugger(gui)
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