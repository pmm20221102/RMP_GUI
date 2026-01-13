import numpy as np
from PyQt5.QtWidgets import QLabel, QSizePolicy
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen
from PyQt5.QtCore import Qt, pyqtSignal

class MapWidget(QLabel):
    """
    工业级重构版：支持动态 SLAM 地图扩张、坐标原点补偿及点击安全校验。
    """
    signal_goal_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.setStyleSheet("background-color: #1a1a1a; border: 2px solid #3f3f3f; border-radius: 4px;")
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 核心地图元数据 (与 ROS 消息严格同步)
        self.map_img = None
        self.current_data = None  # 存储原始栅格数据用于安全检查
        self.info = {
            'res': 0.05,
            'ox': 0.0,
            'oy': 0.0,
            'w': 0,
            'h': 0
        }
        
        # 渲染辅助变量
        self.draw_offset_x = 0
        self.draw_offset_y = 0
        self.scale_factor = 1.0

    def update_map(self, msg):
        """
        处理 nav_msgs/OccupancyGrid 并实现 ARGB32 对齐渲染
        """
        # 1. 实时更新元数据快照
        self.info['w'] = msg.info.width
        self.info['h'] = msg.info.height
        self.info['res'] = msg.info.resolution
        self.info['ox'] = msg.info.origin.position.x
        self.info['oy'] = msg.info.origin.position.y
        
        # 2. 保存原始数据用于点击时的障碍物校验
        self.current_data = np.array(msg.data, dtype=np.int8).reshape((self.info['h'], self.info['w']))
        
        # 3. 构造 ARGB32 图像缓冲区 (防止地图宽度非4倍数时的显示错位)
        # 默认灰色 (Unknown: 0xFF808080)
        img_buffer = np.full((self.info['h'], self.info['w']), 0xFF808080, dtype=np.uint32)
        
        # 填充自由空间 (Free: 0xFFFFFFFF) 和 障碍物 (Occupied: 0xFF000000)
        img_buffer[self.current_data == 0] = 0xFFFFFFFF
        img_buffer[self.current_data == 100] = 0xFF000000

        # 4. 垂直翻转以对齐像素坐标与 ROS 坐标
        img_buffer = np.flipud(img_buffer)

        # 5. 构造 QImage
        q_img = QImage(
            img_buffer.tobytes(), 
            self.info['w'], 
            self.info['h'], 
            self.info['w'] * 4, 
            QImage.Format_ARGB32
        )
        
        self.map_img = q_img.copy() 
        self.update() # 触发 paintEvent

    def paintEvent(self, event):
        super().paintEvent(event)
        if self.map_img is None:
            painter = QPainter(self)
            painter.setPen(QColor("#888888"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Waiting for Map Data...")
            return

        # 保持比例缩放
        scaled_pixmap = QPixmap.fromImage(self.map_img).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        
        # 计算偏移量和当前缩放比例
        self.draw_offset_x = (self.width() - scaled_pixmap.width()) // 2
        self.draw_offset_y = (self.height() - scaled_pixmap.height()) // 2
        self.scale_factor = scaled_pixmap.width() / self.info['w']

        painter = QPainter(self)
        painter.drawPixmap(self.draw_offset_x, self.draw_offset_y, scaled_pixmap)

    def mousePressEvent(self, event):
        """
        核心修复：实现动态原点补偿与点击安全过滤
        """
        if self.map_img is None or self.current_data is None:
            return

        # 1. 计算点击位置相对于地图左上角的像素坐标
        rel_x = event.x() - self.draw_offset_x
        rel_y = event.y() - self.draw_offset_y

        # 2. 还原为原始栅格像素索引
        pixel_x = rel_x / self.scale_factor
        pixel_y = rel_y / self.scale_factor

        # 3. 边界检查
        if not (0 <= pixel_x < self.info['w'] and 0 <= pixel_y < self.info['h']):
            return

        # 4. 安全检查：拦截点击障碍物或未知区域的行为，防止 Nav2 Action 报错
        # 注意：因为渲染时用了 flipud，这里对应原始数据的索引需要处理 Y 轴
        grid_x = int(pixel_x)
        grid_y = self.info['h'] - 1 - int(pixel_y) # 还原回 OccupancyGrid 的 2D 索引
        
        grid_val = self.current_data[grid_y, grid_x]
        if grid_val != 0:
            print(f"[Map] Rejected: Point ({grid_x}, {grid_y}) is Not Free (Val: {grid_val})")
            return

        # 5. 动态原点补偿计算 (World Coordinates)
        # 核心公式：World = Origin + (Pixel * Resolution)
        world_x = self.info['ox'] + (pixel_x * self.info['res'])
        world_y = self.info['oy'] + ((self.info['h'] - pixel_y) * self.info['res'])

        print(f"[Map] Navigating to: World({world_x:.2f}, {world_y:.2f}) | Origin:({self.info['ox']:.2f}, {self.info['oy']:.2f})")
        
        self.signal_goal_clicked.emit(world_x, world_y)