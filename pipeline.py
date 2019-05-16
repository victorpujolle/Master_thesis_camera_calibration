import sys
from PySide2.QtGui import QPixmap, QImage

from PySide2.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget, QFileDialog, QTextEdit, QSizePolicy, QMessageBox, QHBoxLayout
from PySide2.QtCore import Slot, Qt, QStringListModel, QSize, QTimer

import os
import cv2
import pyrealsense2 as rs
import numpy as np
import cv2


class UI_Window(QWidget):

    def __init__(self):
        QWidget.__init__(self)


        # Create a timer.
        self.timer = QTimer()
        self.timer.timeout.connect(self.nextFrameSlot)

        # Create a layout.
        layout = QVBoxLayout()

        # Add a button
        button_layout = QHBoxLayout()

        btnCamera = QPushButton("Open camera")
        btnCamera.clicked.connect(self.openCamera)
        button_layout.addWidget(btnCamera)

        btnCamera = QPushButton("Stop camera")
        btnCamera.clicked.connect(self.stopCamera)
        button_layout.addWidget(btnCamera)

        layout.addLayout(button_layout)



        # Add a label
        self.label = QLabel()
        self.label.setFixedSize(640, 480)
        pixmap = self.resizeImage('dream.jpg')
        self.label.setPixmap(pixmap)

        layout.addWidget(self.label)


        # Set the layout
        self.setLayout(layout)
        self.setWindowTitle("Cam")
        self.setFixedSize(800, 800)

    # https://stackoverflow.com/questions/1414781/prompt-on-exit-in-pyqt-application
    def closeEvent(self, event):

        msg = "Close the app?"
        reply = QMessageBox.question(self, 'Message',
                        msg, QMessageBox.Yes, QMessageBox.No)

        if reply == QMessageBox.Yes:
            event.accept()
            self.stopCamera()
        else:
            event.ignore()


    def resizeImage(self, filename):
        pixmap = QPixmap(filename)
        lwidth = self.label.maximumWidth()
        pwidth = pixmap.width()
        lheight = self.label.maximumHeight()
        pheight = pixmap.height()

        wratio = pwidth * 1.0 / lwidth
        hratio = pheight * 1.0 / lheight

        if pwidth > lwidth or pheight > lheight:
            if wratio > hratio:
                lheight = pheight / wratio
            else:
                lwidth = pwidth / hratio

            scaled_pixmap = pixmap.scaled(lwidth, lheight)
            return scaled_pixmap
        else:
            return pixmap


    def openCamera(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

        self.timer.start(1000./24)

    def stopCamera(self):
        self.timer.stop()


    def nextFrameSlot(self):

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        #rval, frame = self.vc.read()
        frame = cv2.cvtColor(images, cv2.COLOR_BGR2RGB)
        image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.label.setPixmap(pixmap)


def main():
    app = QApplication(sys.argv)
    ex = UI_Window()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()