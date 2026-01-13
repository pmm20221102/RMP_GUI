import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
# QoS Imports
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

# ROS Messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from nav2_msgs.action import NavigateToPose

# CV & Bridge
from cv_bridge import CvBridge
import cv2 

# PyQt Imports
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QSplitter
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap

# --- UI Modules ---
# Ensure you have compiled these files using pyuic5
from .rmp_main_ui import Ui_MainWindow
from .camera_window_ui import Ui_MainWindow as Ui_CameraWindow
from .arm_window_ui import Ui_Form
# Custom Modules
from .tf_monitor_window import TfMonitorWindow
from .map_viz import MapWidget

# ===========================================================
# 🗺️ [Backend] Navigation Worker (Map + Nav2)
# ===========================================================
class NavRosWorker(QThread):
    # Signal to update UI with new map
    map_received = pyqtSignal(OccupancyGrid)

    def __init__(self,node):
        super().__init__()
        self.node = node
        self.nav_client = None
        self.executor = None  # Independent executor

    def run(self):
        # self.node = Node('gui_nav_backend')
        
        # [Critical Fix] Create independent executor and bind node
        # This prevents conflict with the main thread (fixes "wait set index too big")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # QoS Profile for Map (Transient Local is required for SLAM maps)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.node.get_logger().info(f"🔧 Configuring /map sub with Durable={map_qos.durability}")
        # 1. Subscribe to Map
        self.node.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        # 2. Connect to Nav2 Action Server
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        self.node.get_logger().info("Nav backend started (Independent Executor). Waiting for /map...")
        
        # Spin using the independent executor
        try:
            self.executor.spin()
        except Exception as e:
            self.node.get_logger().error(f"Nav Executor spin failed: {e}")
        finally:
            self.executor.shutdown()
            # self.node.destroy_node()

    def map_callback(self, msg):
        self.node.get_logger().info(f"🎉 RECEIVED MAP DATA! Size: {msg.info.width}x{msg.info.height}")
        self.map_received.emit(msg)

    def send_goal(self, x, y):
        """Send Navigation Goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            print("[Error] Nav2 Action Server not found! Is Nav2/SLAM running?")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation: Default to facing Forward (0 degrees)
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.pose.orientation.z = 0.0

        print(f"[Nav] Sending Goal to Nav2: x={x:.2f}, y={y:.2f}")
        self.nav_client.send_goal_async(goal_msg)

    def stop(self):
        if self.executor:
            self.executor.shutdown()
        self.quit()
        self.wait()


# ===========================================================
# 🦾 [Backend] Arm Worker
# ===========================================================
class ArmRosWorker(QThread):
    def __init__(self,node):
        super().__init__()
        self.node = node
        self.action_client = None
        self.executor = None # Independent executor
        self.current_joints = {}
        self.JOINT_NAMES = [
            "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
            "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"
        ]

    def run(self):
        # self.node = Node('arm_control_backend')
        
        # [Critical Fix] Independent executor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Subscribe to joint states (Sensor Data QoS for best effort compatibility)
        self.node.create_subscription(
            JointState, '/joint_states', self.sub_callback, qos_profile_sensor_data
        )
        
        self.action_client = ActionClient(
            self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.node.get_logger().info("Arm backend started (Independent Executor)")
        
        try:
            self.executor.spin()
        except Exception:
            pass
        finally:
            self.executor.shutdown()
            # self.node.destroy_node()

    def sub_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def send_cmd(self, joint_name, delta):
        if not self.action_client or not self.action_client.server_is_ready():
            print("[Error] Arm controller not connected!")
            return
        
        curr = self.current_joints.get(joint_name, 0.0)
        target_val = curr + delta
        print(f"[{joint_name}] Current: {curr:.2f} -> Target: {target_val:.2f}")
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0.5).to_msg()
        
        target_positions = []
        for name in self.JOINT_NAMES:
            c = self.current_joints.get(name, 0.0)
            if name == joint_name:
                target_positions.append(target_val)
            else:
                target_positions.append(c)
        
        point.positions = target_positions
        goal.trajectory.points = [point]
        self.action_client.send_goal_async(goal)

    def stop(self):
        if self.executor:
            self.executor.shutdown()
        self.quit()
        self.wait()


# ===========================================================
# 🦾 [Frontend] Arm Control Window
# ===========================================================
class ArmControlWindow(QWidget, Ui_Form):
    def __init__(self, node): # 传入主 GUI 的 node 引用
        super().__init__()
        self.setupUi(self)
        self.node = node # 保存引用
        self.setWindowTitle("Robot Arm Debug")
        self.resize(600, 400)

        # 内部辅助函数：安全地获取参数，如果没声明则先声明
        def get_safe_param(name, default_val):
            if not self.node.has_parameter(name):
                self.node.declare_parameter(name, default_val)
            return self.node.get_parameter(name).value

        # 1. 声明并从参数服务器（YAML）获取 6 个关节的步长
        step_pan   = get_safe_param('arm_joint_steps.shoulder_pan', 0.15)
        step_lift  = get_safe_param('arm_joint_steps.shoulder_lift', 0.15)
        step_elbow = get_safe_param('arm_joint_steps.elbow', 0.15)
        step_w1    = get_safe_param('arm_joint_steps.wrist_1', 0.10)
        step_w2    = get_safe_param('arm_joint_steps.wrist_2', 0.10)
        step_w3    = get_safe_param('arm_joint_steps.wrist_3', 0.10)

        self.worker = ArmRosWorker(self.node)
        self.worker.start()

        # 2. 将按钮点击事件绑定到对应的关节和步长上
        # Shoulder Pan
        self.btn_pan_left.clicked.connect(lambda: self.worker.send_cmd("ur_shoulder_pan_joint", step_pan))
        self.btn_pan_right.clicked.connect(lambda: self.worker.send_cmd("ur_shoulder_pan_joint", -step_pan))
        
        # Shoulder Lift
        self.btn_lift_up.clicked.connect(lambda: self.worker.send_cmd("ur_shoulder_lift_joint", -step_lift))
        self.btn_lift_down.clicked.connect(lambda: self.worker.send_cmd("ur_shoulder_lift_joint", step_lift))
        
        # Elbow
        self.btn_elbow_ext.clicked.connect(lambda: self.worker.send_cmd("ur_elbow_joint", -step_elbow))
        self.btn_elbow_ret.clicked.connect(lambda: self.worker.send_cmd("ur_elbow_joint", step_elbow))
        
        # Wrist 1
        self.btn_wrist1_pos.clicked.connect(lambda: self.worker.send_cmd("ur_wrist_1_joint", step_w1))
        self.btn_wrist1_neg.clicked.connect(lambda: self.worker.send_cmd("ur_wrist_1_joint", -step_w1))

        # Wrist 2
        self.btn_wrist2_pos.clicked.connect(lambda: self.worker.send_cmd("ur_wrist_2_joint", step_w2))
        self.btn_wrist2_neg.clicked.connect(lambda: self.worker.send_cmd("ur_wrist_2_joint", -step_w2))

        # Wrist 3
        self.btn_wrist3_pos.clicked.connect(lambda: self.worker.send_cmd("ur_wrist_3_joint", step_w3))
        self.btn_wrist3_neg.clicked.connect(lambda: self.worker.send_cmd("ur_wrist_3_joint", -step_w3))

# ===========================================================
# 📸 [Frontend] Camera Monitor Window
# ===========================================================
class CameraWindow(QMainWindow):
    def __init__(self,node):
        super().__init__()
        self.ui = Ui_CameraWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Camera Monitor")
        self.node = node
        self._frame = None
        self.setMinimumSize(640, 480)
        self.setMaximumSize(1280, 720)
        self.resize(800, 600)
        self.ui.label_camera.setScaledContents(True)
        self.ui.label_camera.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)

    def update_frame(self, frame_bgr):
        if frame_bgr is None: return
        self._frame = frame_bgr.copy()
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        self.ui.label_camera.setPixmap(QPixmap.fromImage(qt_img))

    def resizeEvent(self, event):
        if self._frame is not None: self.update_frame(self._frame)
        super().resizeEvent(event)


# ===========================================================
# 🚀 [Frontend] Main GUI (TeleopGUI)
# ===========================================================
class TeleopGUI(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, "teleop_gui")
        QMainWindow.__init__(self)
        self.declare_parameter('base_movement.linear_step', 0.5)
        self.declare_parameter('base_movement.angular_step', 0.5)
        self.l_step = self.get_parameter('base_movement.linear_step').value
        self.a_step = self.get_parameter('base_movement.angular_step').value
        # ------------------------------------------------------------------
        # 1. ROS2 Initialization
        # ------------------------------------------------------------------
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # State Variables
        self.current_speed = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.current_frame = None

        # ------------------------------------------------------------------
        # 2. [Critical] Initialize Layout First
        # ------------------------------------------------------------------
        # This loads the UI file and sets up the Map/Control split
        self.init_ui_layout()

        # ------------------------------------------------------------------
        # 3. Connect Menu Actions (After UI is loaded)
        # ------------------------------------------------------------------
        
        # Camera Menu
        self.camera_window = CameraWindow(self)
        if hasattr(self.ui, "actionCamera"):
            self.ui.actionCamera.triggered.connect(self.show_camera_window)

        # Arm Control Menu
        self.arm_window = None 
        if hasattr(self.ui, "actionOpenArmWindow"):
            self.ui.actionOpenArmWindow.triggered.connect(self.show_arm_window)

        # TF Monitor Menu
        self.tf_window = None
        if hasattr(self.ui, "actionOpenTfWindow"):
            self.ui.actionOpenTfWindow.triggered.connect(self.show_tf_window)
        else:
            print("[Warning] actionOpenTfWindow not found in UI (Check Qt Designer)")

        # ------------------------------------------------------------------
        # 4. Bind Control Buttons
        # ------------------------------------------------------------------
        self.bind_control_buttons()

        # ------------------------------------------------------------------
        # 5. Start Background Workers
        # ------------------------------------------------------------------
        # Navigation Worker
        self.nav_thread = NavRosWorker(self)
        self.nav_thread.map_received.connect(self.update_map_display)
        self.nav_thread.start()
        
        # Connect Map Clicks to Nav Thread
        self.map_widget.signal_goal_clicked.connect(self.nav_thread.send_goal)

        # 6. UI Update Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start(100)

    # ======================================================================
    # Layout Setup: Left (Controls) | Right (Map)
    # ======================================================================
    def init_ui_layout(self):
        """
        [Final Layout] Fixed Control Panel on Left | Map expands on Right
        """
        # 1. Instantiate and Load UI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # 2. Connect the "Show Map" button from Designer
        if hasattr(self.ui, "btn_show_map"):
            self.ui.btn_show_map.clicked.connect(self.toggle_map_view)
            self.ui.btn_show_map.setCheckable(True)

        # 3. Prepare Panels
        # Panel A: Control Panel (Left)
        control_panel = self.centralWidget()
        # Fix width to ensure buttons don't disappear or move wildly
        control_panel.setMinimumWidth(400) 
        control_panel.setMaximumWidth(500)

        # Panel B: Map (Right)
        self.map_widget = MapWidget()
        self.map_widget.setVisible(False) # Hidden by default

        # 4. Assemble Splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(control_panel)   # Index 0: Left
        splitter.addWidget(self.map_widget) # Index 1: Right
        
        # 5. Set Stretch Factors
        # 0 = Keep minimum size (Left)
        # 1 = Expand to fill space (Right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        
        # Prevent left panel from collapsing
        splitter.setCollapsible(0, False)

        # 6. Apply Layout
        self.setCentralWidget(splitter)

        # 7. Initial Window Size (Small, no map)
        self.resize(450, 600)

    def toggle_map_view(self, checked):
        """Handle Map visibility toggling"""
        if checked:
            # Show Map
            self.map_widget.setVisible(True)
            if hasattr(self.ui, "btn_show_map"):
                self.ui.btn_show_map.setText("Hide Map")
            
            # Auto-expand window if it's too narrow
            if self.width() < 800:
                self.resize(1000, self.height())
        else:
            # Hide Map
            self.map_widget.setVisible(False)
            if hasattr(self.ui, "btn_show_map"):
                self.ui.btn_show_map.setText("Show Map")
            
            # Auto-shrink window
            self.resize(450, self.height())

    def bind_control_buttons(self):
        """Connect button signals"""
        self.ui.btn_forward.clicked.connect(self.forward)
        self.ui.btn_backward.clicked.connect(self.backward)
        self.ui.btn_left.clicked.connect(self.turn_left)
        self.ui.btn_right.clicked.connect(self.turn_right)
        self.ui.btn_stop.clicked.connect(self.stop)
        
        # Check for diagonal buttons (optional)
        if hasattr(self.ui, "btn_forward_left"):
            self.ui.btn_forward_left.clicked.connect(self.forward_left)
        if hasattr(self.ui, "btn_forward_right"):
            self.ui.btn_forward_right.clicked.connect(self.forward_right)
        if hasattr(self.ui, "btn_backward_left"):
            self.ui.btn_backward_left.clicked.connect(self.backward_left)
        if hasattr(self.ui, "btn_backward_right"):
            self.ui.btn_backward_right.clicked.connect(self.backward_right)

    # --- Popups ---
    def show_arm_window(self):
        if self.arm_window is None:
            # 关键：将 self 传递给机械臂窗口，使其能访问参数服务器
            self.arm_window = ArmControlWindow(self) 
        self.arm_window.show()
        self.arm_window.raise_()
        self.arm_window.activateWindow()

    def show_tf_window(self):
        if self.tf_window is None:
            self.tf_window = TfMonitorWindow()
        self.tf_window.show()
        self.tf_window.raise_()
        self.tf_window.activateWindow()
        
    def show_camera_window(self):
        self.camera_window.show()
        self.camera_window.raise_()
        self.camera_window.activateWindow()

    # --- Teleop Logic ---
    def forward(self): self._pub_vel(self.l_step, 0.0)
    def backward(self): self._pub_vel(-self.l_step, 0.0)
    def turn_left(self): self._pub_vel(0.0, self.a_step)
    def turn_right(self): self._pub_vel(0.0, -self.a_step)
    def stop(self): self._pub_vel(0.0, 0.0)
    def forward_left(self): self._pub_vel(self.l_step, self.a_step)
    def forward_right(self): self._pub_vel(self.l_step, -self.a_step)
    def backward_left(self): self._pub_vel(-self.l_step, -self.a_step)
    def backward_right(self): self._pub_vel(-self.l_step, self.a_step)

    def _pub_vel(self, x, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.angular.z = float(z)
        self.publisher.publish(msg)

    # --- Callbacks ---
    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        import math
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_theta = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_frame = cv_img
            if self.camera_window.isVisible():
                self.camera_window.update_frame(cv_img)
        except Exception:
            pass
            
    def update_map_display(self, map_msg):
        #if self.map_widget.isVisible():
        print(f"🖥️ GUI SLOT HIT! Updating Widget...")
        self.map_widget.update_map(map_msg)

    # --- UI Refresh ---
    def refresh_ui(self):
        self.ui.label_speed.setText(f"Speed: {self.current_speed:.2f} m/s")
        self.ui.lcd_speed.display(self.current_speed)
        
        # Speed Progress bar (Assume max speed 1.0 m/s)
        progress = int((abs(self.current_speed) / 1.0) * 100)
        self.ui.progress_speed.setValue(progress)
        
        self.ui.label_position.setText(
            f"Pos: x={self.pose_x:.2f}, y={self.pose_y:.2f}, θ={self.pose_theta:.1f}°"
        )
        
        # Main window preview (if label exists)
        if self.current_frame is not None and hasattr(self.ui, "label_image"):
            rgb = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qt_img).scaled(
                self.ui.label_image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.ui.label_image.setPixmap(pix)


# ===========================================================
# 🚀 Entry Point
# ===========================================================
def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    gui = TeleopGUI()
    gui.show()

    # Use a QTimer to spin the ROS node so Qt doesn't freeze
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(gui, timeout_sec=0))
    timer.start(20) # 50Hz

    app.exec_()
    
    # Clean up
    gui.destroy_node()
    if gui.arm_window: gui.arm_window.worker.stop()
    if gui.nav_thread: gui.nav_thread.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()