import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal

class VideoThread(QThread):
    frame_signal = pyqtSignal(QImage)  # 发送处理后的 QImage

    def __init__(self):
        super().__init__()
        self.running = True  # 线程运行状态
        self.cap = cv2.VideoCapture(0)  # 连接摄像头

    def run(self):
        """ 运行线程，读取摄像头帧 """
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qt_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.frame_signal.emit(qt_img)  # 发送信号

    def stop(self):
        """ 停止线程 """
        self.running = False
        self.cap.release()
        self.quit()
        self.wait()

class CameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt5 QThread 采集相机")
        self.setGeometry(100, 100, 800, 600)

        self.label = QLabel(self)
        self.label.setFixedSize(640, 480)

        self.start_button = QPushButton("启动相机", self)
        self.stop_button = QPushButton("关闭相机", self)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        self.setLayout(layout)

        self.start_button.clicked.connect(self.start_camera)
        self.stop_button.clicked.connect(self.stop_camera)

        self.thread = None  # 视频线程

    def start_camera(self):
        if not self.thread:
            self.thread = VideoThread()
            self.thread.frame_signal.connect(self.update_frame)
            self.thread.start()

    def update_frame(self, qt_img):
        """ 更新 QLabel 上的画面 """
        self.label.setPixmap(QPixmap.fromImage(qt_img))

    def stop_camera(self):
        if self.thread:
            self.thread.stop()
            self.thread = None
            self.label.clear()

    def closeEvent(self, event):
        self.stop_camera()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = CameraApp()
    win.show()
    sys.exit(app.exec_())
