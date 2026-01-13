import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor # [Fix] Import Executor
from tf2_msgs.msg import TFMessage
import math

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTreeWidget, QTreeWidgetItem, QHeaderView
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal

# ===========================================================
# 📡 Backend: TF Listener Thread
# ===========================================================
class TfListenerThread(QThread):
    # Signal: (parent, child, x, y, z, roll, pitch, yaw)
    update_signal = pyqtSignal(str, str, float, float, float, float, float, float)

    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None # [Fix] Independent executor

    def run(self):
        # Create dedicated node
        self.node = Node('gui_tf_monitor_node')
        
        # [Fix] Create independent executor to prevent "Wait Set" crash
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Subscribe to /tf and /tf_static
        # using sensor_data QoS for best compatibility
        self.node.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, '/tf_static', self.tf_callback, qos_profile_sensor_data)
        
        # [Fix] Spin using local executor, NOT rclpy.spin()
        try:
            self.executor.spin()
        except Exception as e:
            pass
        finally:
            self.executor.shutdown()
            self.node.destroy_node()

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            
            # Extract Translation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            
            # Extract Rotation (Quaternion -> Euler)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Math conversion
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)

            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Emit signal (Convert radians to degrees)
            self.update_signal.emit(parent, child, tx, ty, tz, 
                                    math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def stop(self):
        if self.executor:
            self.executor.shutdown()
        self.quit()
        self.wait()

# ===========================================================
# 🖥️ Frontend: TF Monitor Window
# ===========================================================
class TfMonitorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TF Transform Monitor")
        self.resize(800, 400)
        
        layout = QVBoxLayout(self)
        
        # 1. Create Tree Widget
        self.tree = QTreeWidget()
        self.tree.setColumnCount(8)
        self.tree.setHeaderLabels([
            "Child Frame", "Parent Frame", 
            "X", "Y", "Z", 
            "Roll (°)", "Pitch (°)", "Yaw (°)"
        ])
        # Auto resize columns
        self.tree.header().setSectionResizeMode(QHeaderView.ResizeToContents)
        layout.addWidget(self.tree)
        
        # Cache for existing rows: {child_frame_name: QTreeWidgetItem}
        self.rows = {}

        # 2. Start Listener Thread
        self.thread = TfListenerThread()
        self.thread.update_signal.connect(self.update_row)
        self.thread.start()

    def update_row(self, parent, child, x, y, z, r, p, yaw):
        # Update existing row or create new one
        if child in self.rows:
            item = self.rows[child]
        else:
            item = QTreeWidgetItem(self.tree)
            self.rows[child] = item
        
        # Update text
        item.setText(0, child)
        item.setText(1, parent)
        item.setText(2, f"{x:.3f}")
        item.setText(3, f"{y:.3f}")
        item.setText(4, f"{z:.3f}")
        item.setText(5, f"{r:.1f}")
        item.setText(6, f"{p:.1f}")
        item.setText(7, f"{yaw:.1f}")

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()