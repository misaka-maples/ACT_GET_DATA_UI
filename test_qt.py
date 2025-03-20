
from tt import CAMERA_HOT_PLUG
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import sys
import cv2
import numpy as np

class CameraApp(QWidget):
    def __init__(self, camera_system):
        super().__init__()
        self.camera_system = camera_system  # 相机管理对象
        self.init_ui()
        self.start_timer()

    def init_ui(self):
        self.setWindowTitle("Orbbec Camera Viewer")
        self.setGeometry(100, 100, 800, 600)
        self.layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.layout.addWidget(self.image_label)
        self.setLayout(self.layout)

    def start_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 每 30ms 获取一次帧数据

    def update_frame(self):
        frame_data = self.camera_system.rendering_frame()
        if isinstance(frame_data, np.ndarray):  # 只有一台相机
            self.display_image(frame_data)
        elif isinstance(frame_data, dict):  # 多台相机
            for serial, img in frame_data.items():
                self.display_image(img)  # 这里只显示最后一台相机的画面

    def display_image(self, image):
        height, width, channel = image.shape
        bytes_per_line = 3 * width
        qt_image = QImage(image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_image))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    camera_system = CAMERA_HOT_PLUG()  # 初始化相机
    window = CameraApp(camera_system)
    window.show()
    sys.exit(app.exec_())
