# interface lib
#from PySide2.QtGui import QPixmap, QImage
#from PySide2.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget, QFileDialog, QTextEdit, QSizePolicy, QMessageBox, QHBoxLayout
#from PySide2.QtCore import Slot, Qt, QStringListModel, QSize, QTimer

from PyQt5 import QtGui, QtWidgets, QtCore
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QPushButton, QAction, QMenu, QLabel
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.QtCore import Qt, QStringListModel, QSize, QTimer

# scientific and image lib
import pyrealsense2 as rs
import numpy as np
import cv2

# basic lib
import sys
import os
import glob
import socket
import math
import time

# plotting lib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D

# my lib
from Arm import Arm
from utils import *





class robot_cam_interface(QMainWindow):
    """
    The goal of this interface is the simulation of the IK
    Element of this interface will be incorporate later in an other interface for robot control
    """


    def __init__(self, arm):

        # super init
        super(robot_cam_interface, self).__init__()

        # link to the arm
        self.arm = arm

        # init UI
        self.init_UI()

        # init figure
        self.init_figure()

        # init menu
        self.init_menu()

        # init video pipeline
        self.init_pipeline()

        # init robot position
        self.arm.initialize_position()
        self.arm.activate_all_reg()

        # init randomness for the sake of reproductability
        np.random.seed(19680801)



    # ------INIT FUNCTIONS-----

    def init_UI(self):
        """
        initialisation of the UI windows
        """
        self._count = True
        # Create Widget for Figure/Menu/Image
        self.FigureWidget = QtWidgets.QWidget(self)
        self.MenuWidget = QtWidgets.QWidget(self)
        self.ImageWidget = QtWidgets.QWidget(self)



        # Add Layout to each Widget
        self.FigureLayout = QtWidgets.QVBoxLayout(self.FigureWidget)
        self.MenuLayout = QtWidgets.QVBoxLayout(self.MenuWidget)
        self.ImageLayout = QtWidgets.QVBoxLayout(self.ImageWidget)

        # Delete Margin of the layout
        self.FigureLayout.setContentsMargins(0, 0, 0, 0)
        self.MenuLayout.setContentsMargins(0, 0, 0, 0)
        self.ImageLayout.setContentsMargins(0, 0, 0, 0)

        # create image label
        self.ImageLabel = QtWidgets.QLabel(self.ImageWidget)
        self.ImageLabel.setFixedSize(640, 480)


        #self.ImageLayout.addWidget(self.ImageLabel)

        # set title
        self.title = 'Robot interface'
        self.setWindowTitle(self.title)


        # set Geometry
        self.setGeometry(0, 0, 1500, 715)
        self.FigureWidget.setGeometry(15, 300, 400, 400)
        self.MenuWidget.setGeometry(0, 0, 400, 280)
        self.ImageWidget.setGeometry(415,0,640,480)


        return 0

    def init_figure(self):
        """
        initialisation of the figure
        """
        ## Creat the Figure to drow 3D plot
        self.Figure = plt.figure()
        # Add FigureCanvas to Figure
        self.FigureCanvas = FigureCanvas(self.Figure)
        # Addd FigureCanvas to Layout
        self.FigureLayout.addWidget(self.FigureCanvas)
        self.axis = self.Figure.add_subplot(121, projection='3d')

        self.axis_xlabel = "X"
        self.axis_ylabel = "Y"
        self.axis_zlabel = "Z"
        self.axis_xlim3d = [-0.4, 0.4]
        self.axis_ylim3d = [-0.4, 0.4]
        self.axis_zlim3d = [0, 0.8]

        # set the label for each axis
        self.axis.set_xlabel(self.axis_xlabel)
        self.axis.set_ylabel(self.axis_ylabel)
        self.axis.set_zlabel(self.axis_zlabel)
        # set the limit of each axis
        self.axis.set_xlim3d(self.axis_xlim3d[0], self.axis_xlim3d[1])
        self.axis.set_ylim3d(self.axis_ylim3d[0], self.axis_ylim3d[1])
        self.axis.set_zlim3d(self.axis_zlim3d[0], self.axis_zlim3d[1])

        return 0

    def init_pipeline(self):
        """
        init of the video pipeline
        """
        # Create a timer.
        self.timer = QTimer()
        self.timer.timeout.connect(self.nextFrameSlot)

        # Create the pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Configure depth and color streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        return 0

    def init_menu(self):
        """
        Initialisation of the menu
        """
        # menu bars
        menubar = self.menuBar()

        # menu camera
        menu_camera = menubar.addMenu('Camera')   


        action_open_cam = QAction('Open Camera', self)
        action_open_cam.setStatusTip('Open Camera')
        action_open_cam.triggered.connect(self.openCamera)
        menu_camera.addAction(action_open_cam)

        action_stop_cam = QAction('Stop Camera', self)
        action_stop_cam.setStatusTip('Stop Camera')
        action_stop_cam.triggered.connect(self.stopCamera)
        menu_camera.addAction(action_stop_cam)

        # menu figure
        menu_figure = menubar.addMenu('Figure')

        action_clear_figure = QAction('Clear', self)
        action_clear_figure.setStatusTip('Clear')
        action_clear_figure.triggered.connect(self.on_click_clear)
        menu_figure.addAction(action_clear_figure)



        # windows
        

        # create label
        self.label_goal = QtWidgets.QLabel(self.MenuWidget)
        self.label_result_X = QtWidgets.QLabel(self.MenuWidget)

        self.label_X  = QtWidgets.QLabel(self.MenuWidget)
        self.label_Y  = QtWidgets.QLabel(self.MenuWidget)
        self.label_Z  = QtWidgets.QLabel(self.MenuWidget)
        self.label_aX = QtWidgets.QLabel(self.MenuWidget)
        self.label_aY = QtWidgets.QLabel(self.MenuWidget)
        self.label_aZ = QtWidgets.QLabel(self.MenuWidget)

        self.label_q0 = QtWidgets.QLabel(self.MenuWidget)
        self.label_q1 = QtWidgets.QLabel(self.MenuWidget)
        self.label_q2 = QtWidgets.QLabel(self.MenuWidget)
        self.label_q3 = QtWidgets.QLabel(self.MenuWidget)
        self.label_q4 = QtWidgets.QLabel(self.MenuWidget)
        self.label_q5 = QtWidgets.QLabel(self.MenuWidget)

        # create textbox
        self.textbox_input_X  = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_input_Y  = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_input_Z  = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_input_aX = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_input_aY = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_input_aZ = QtWidgets.QLineEdit(self.MenuWidget)

        self.textbox_q0 = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_q1 = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_q2 = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_q3 = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_q4 = QtWidgets.QLineEdit(self.MenuWidget)
        self.textbox_q5 = QtWidgets.QLineEdit(self.MenuWidget)

        # create button
        self.button_calculate = QtWidgets.QPushButton('calculate', self.MenuWidget)
        self.button_draw_goal = QtWidgets.QPushButton('draw goal', self.MenuWidget)
        self.button_draw = QtWidgets.QPushButton('draw', self.MenuWidget)
        self.button_move = QtWidgets.QPushButton('move', self.MenuWidget)

        # resize button and textbox
        self.textbox_input_X.resize(100,20)
        self.textbox_input_Y.resize(100,20)
        self.textbox_input_Z.resize(100,20)
        self.textbox_input_aX.resize(100,20)
        self.textbox_input_aY.resize(100,20)
        self.textbox_input_aZ.resize(100,20)

        self.textbox_q0.resize(100, 20)
        self.textbox_q1.resize(100, 20)
        self.textbox_q2.resize(100, 20)
        self.textbox_q3.resize(100, 20)
        self.textbox_q4.resize(100, 20)
        self.textbox_q5.resize(100, 20)

        self.button_calculate.resize(100,30)
        self.button_draw_goal.resize(100,30)
        self.button_draw.resize(100,30)
        self.button_move.resize(100, 30)

        # set text of lables)

        self.label_X.setText('X :')
        self.label_Y.setText('Y :')
        self.label_Z.setText('Z :')
        self.label_aX.setText('aX :')
        self.label_aY.setText('aY :')
        self.label_aZ.setText('aZ :')

        self.label_q0.setText('q0 :')
        self.label_q1.setText('q1 :')
        self.label_q2.setText('q2 :')
        self.label_q3.setText('q3 :')
        self.label_q4.setText('q4 :')
        self.label_q5.setText('q5 :')

        # set location of everythink

        self.label_X.move( 15, 30)
        self.label_Y.move( 15, 30 + 30)
        self.label_Z.move( 15, 30 + 2 * 30)
        self.label_aX.move(15, 30 + 3 * 30)
        self.label_aY.move(15, 30 + 4 * 30)
        self.label_aZ.move(15, 30 + 5 * 30)

        self.textbox_input_X.move( 15 + 35, 35)
        self.textbox_input_Y.move( 15 + 35, 35 + 30)
        self.textbox_input_Z.move( 15 + 35, 35 + 2 * 30)
        self.textbox_input_aX.move(15 + 35, 35 + 3 * 30)
        self.textbox_input_aY.move(15 + 35, 35 + 4 * 30)
        self.textbox_input_aZ.move(15 + 35, 35 + 5 * 30)

        self.label_q0.move(180, 35)
        self.label_q1.move(180, 35 + 30)
        self.label_q2.move(180, 35 + 2 * 30)
        self.label_q3.move(180, 35 + 3 * 30)
        self.label_q4.move(180, 35 + 4 * 30)
        self.label_q5.move(180, 35 + 5 * 30)

        self.textbox_q0.move(215, 35)
        self.textbox_q1.move(215, 35 + 30)
        self.textbox_q2.move(215, 35 + 2 * 30)
        self.textbox_q3.move(215, 35 + 3 * 30)
        self.textbox_q4.move(215, 35 + 4 * 30)
        self.textbox_q5.move(215, 35 + 5 * 30)

        self.button_calculate.move(15+35, 250)
        self.button_draw_goal.move(15+35, 220)
        self.button_draw.move(215, 250)
        self.button_move.move(215, 220)

        # link button to method
        self.button_calculate.clicked.connect(self.on_click_calculate)
        self.button_draw_goal.clicked.connect(self.on_click_draw_goal)
        self.button_draw.clicked.connect(self.on_click_draw)
        self.button_move.clicked.connect(self.on_click_move)

        return 0

    # -----BUTTON FUNCTIONS-----

    def on_click_calculate(self):
        """
        method linked to the button calculate
        """
        print('--- EVENT : CALCULATE CLICK ---')
        # clear the figure
        self.clear_figure()


        # take the value for the goal
        [X, Y, Z, aX, aY, aZ] = np.array(charlist2floatlist(self.get_X_input()))
        [aX_rad, aY_rad, aZ_rad] = deg2rad(np.array([aX, aY, aZ]))


        # draw goal
        self.draw_single_point([X, Y, Z])
        R = eulerAnglesToRotationMatrix(deg2rad(np.array([aX, aY, aZ])))
        self.draw_reference_frame([X, Y, Z], R)
        self.FigureCanvas.draw()

        # take the values of the angles as first guess but with small random deviation
        q_deg = np.array(charlist2floatlist(self.get_q_values()))
        self.set_q_values(floatlist2charlist(q_deg))
        q_rad = deg2rad(q_deg)



        # compute the IK
        goal = np.array([X, Y, Z, aX, aY, aZ])
        first_guess = q_rad


        result = self.arm.IK.gradient_descent(goal, first_guess)
        q_f = rad2deg(result['x'])


        # set output
        self.set_q_values(floatlist2charlist(np.round(q_f,3)))


        # draw arm
        x, y, z = self.arm.IK.compute_pose(q_rad)  # compute all partial kinematic
        R, t = self.arm.IK.compute_FK(q_rad)  # compute end effector kinematic
        self.draw_reference_frame(t, R)
        self.draw_arm([x, y, z])  # draw the arm

        #a = rotationMatrixToEulerAngles(R)
        #a = rad2deg(np.array(a))

        #fk = floatlist2charlist(np.round(np.array([t[0],t[1],t[2],a[0],a[1],a[2]]),2))


        self.FigureCanvas.draw()
        return 0

    def on_click_draw_goal(self, verbose=True):
        """
        method linked with the button draw goal
        """
        print('--- EVENT : DRAW GOAL CLICK ---')
        [X, Y, Z, aX, aY, aZ] = np.array(charlist2floatlist(self.get_X_input()))

        if verbose: print('goal input :', X, Y, Z, aX, aY, aZ)

        self.draw_single_point([X,Y,Z])

        R = eulerAnglesToRotationMatrix(deg2rad(np.array([aX, aY, aZ])))
        self.draw_reference_frame([X,Y,Z],R)

        self.FigureCanvas.draw()

        return 0

    def on_click_draw(self):
        """
        method linked to the button draw
        """
        print('--- EVENT : DRAW ARM CLICK ---')

        # take the value
        q_deg = np.array(charlist2floatlist(self.get_q_values()))
        self.set_q_values(floatlist2charlist(q_deg))
        q_rad = deg2rad(q_deg)


        # figure and UI control
        x, y, z = self.arm.IK.compute_pose(q_rad)  # compute all partial kinematic
        R,t = self.arm.IK.compute_FK(q_rad) # compute end effector kinematic

        self.draw_reference_frame(t,R)
        self.draw_arm([x, y, z])  # draw the arm
        # self.draw_ref(list_vector_frame, list_origin_frame)
        self.FigureCanvas.draw()

        return 0

    def on_click_clear(self):
        """
        method linked to the button clear
        """
        print('--- EVENT : ClEAR CLICK ---')
        self.clear_figure()  # clear figure
        self.FigureCanvas.draw()
        return 0

    def on_click_move(self):
        """
        method linked to the button clear
        """
        print('--- EVENT : MOVE CLICK ---')
        th0, th1, th2, th3, th4, th5 = self.get_q_values()
        X = [th0, th1, th2, th3, th4, th5]
        angles = [float(X[i]) if (X[i] != '') else X[i] for i in range(len(X))]
        self.arm.set_arm_position(angles)
        self.arm.activate_all_reg()
        return 0


    # -----MENU ACTION-----

    def openCamera(self):
        print('--- EVENT : OPEN CAMERA ACTION ---')

        # Start streaming

        self.timer.start(1000./24)

        return 0

    def stopCamera(self):
        print('--- EVENT : STOP CAMERA ACTION ---')
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

        self.ImageLabel.setPixmap(self.resizeImage(pixmap))


    def resizeImage(self, pixmap):
        lwidth = self.ImageLabel.maximumWidth()
        pwidth = pixmap.width()
        lheight = self.ImageLabel.maximumHeight()
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


    # -----GETTER AND SETTER-----

    def get_X_input(self):
        """
        get the input values of the goal position
        :return: return the input values
        """
        X  = self.textbox_input_X.text()
        Y  = self.textbox_input_Y.text()
        Z  = self.textbox_input_Z.text()
        aX = self.textbox_input_aX.text()
        aY = self.textbox_input_aY.text()
        aZ = self.textbox_input_aZ.text()

        return X, Y, Z, aX, aY, aZ

    def set_X_input(self, X):
        """
        method used for testing
        :param X: list of string values that will by displayed in the textbox
        """

        self.textbox_input_X.setText(X[0])
        self.textbox_input_Y.setText(X[1])
        self.textbox_input_Z.setText(X[2])
        self.textbox_input_aX.setText(X[3])
        self.textbox_input_aY.setText(X[4])
        self.textbox_input_aZ.setText(X[5])

        return 0

    def get_q_values(self):
        """
        method read the postion of the robot and set the textboxes
        """

        q0 = self.textbox_q0.text()
        q1 = self.textbox_q1.text()
        q2 = self.textbox_q2.text()
        q3 = self.textbox_q3.text()
        q4 = self.textbox_q4.text()
        q5 = self.textbox_q5.text()

        return q0, q1, q2, q3, q4, q5

    def set_X_output(self, X):
        """
        :param X: list of string values that will by displayed in the textbox
        """

        self.textbox_output_X.setText(X[0])
        self.textbox_output_Y.setText(X[1])
        self.textbox_output_Z.setText(X[2])
        self.textbox_output_aX.setText(X[3])
        self.textbox_output_aY.setText(X[4])
        self.textbox_output_aZ.setText(X[5])

        return 0

    def set_q_values(self, X):
        """
        :param X: list of string values that will by displayed in the textbox
        """

        self.textbox_q0.setText(X[0])
        self.textbox_q1.setText(X[1])
        self.textbox_q2.setText(X[2])
        self.textbox_q3.setText(X[3])
        self.textbox_q4.setText(X[4])
        self.textbox_q5.setText(X[5])

        return 0

    # -----DRAWING FUNCTIONS-----

    def clear_figure(self):
        """
        This function clear the figure
        """
        self.axis.clear()

        # set the label for each axis
        self.axis.set_xlabel(self.axis_xlabel)
        self.axis.set_ylabel(self.axis_ylabel)
        self.axis.set_zlabel(self.axis_zlabel)
        # set the limit of each axis
        self.axis.set_xlim3d(self.axis_xlim3d[0], self.axis_xlim3d[1])
        self.axis.set_ylim3d(self.axis_ylim3d[0], self.axis_ylim3d[1])
        self.axis.set_zlim3d(self.axis_zlim3d[0], self.axis_zlim3d[1])

        return 0

    def draw_arm(self, X):
        """
        draw the arm in the figure
        :param X: [x,y,z] where x in the list of x coord of each joint of the arm, and so on.
        """
        [x, y, z] = X

        self.axis.plot(x, y, z, color='orange', linewidth=3.0)
        self.axis.scatter(x, y, z, linewidth=3.0)

        return 0

    def draw_single_point(self,X):
        """
        this method draw on the figure a single point, with a special emphasis on this point
        :param X: x,y,z coordinates of the point
        """
        [x,y,z] = X
        x_axis = np.array([[self.axis_xlim3d[0], y, z], [self.axis_xlim3d[1], y, z]]).T
        y_axis = np.array([[x, self.axis_ylim3d[0], z], [x, self.axis_ylim3d[1], z]]).T
        z_axis = np.array([[x, y, self.axis_zlim3d[0]], [x, y, self.axis_zlim3d[1]]]).T

        self.axis.scatter(x, y, z, color='gray', linewidth = 0.05)
        self.axis.plot(x_axis[0], x_axis[1], x_axis[2], color='gray', linewidth = 1.0)
        self.axis.plot(y_axis[0], y_axis[1], y_axis[2], color='gray', linewidth = 1.0)
        self.axis.plot(z_axis[0], z_axis[1], z_axis[2], color='gray', linewidth = 1.0)

        return 0

    def draw_reference_frame(self, origin, R, scale=0.1):
        """
        draw a reference frame at the given origin with a rotation R
        :param origin: point
        :param R: rotation matrix
        """
        scale_x = (self.axis_xlim3d[1] - self.axis_xlim3d[0]) * scale
        scale_y = (self.axis_ylim3d[1] - self.axis_ylim3d[0]) * scale
        scale_z = (self.axis_zlim3d[1] - self.axis_zlim3d[0]) * scale
        x_vect = np.array([1, 0, 0])
        y_vect = np.array([0, 1, 0])
        z_vect = np.array([0, 0, 1])

        rotated_x_vect = origin + R.dot(x_vect)*scale_x
        rotated_y_vect = origin + R.dot(y_vect)*scale_x
        rotated_z_vect = origin + R.dot(z_vect)*scale_x

        ref = np.array([rotated_x_vect, origin, rotated_y_vect, origin, rotated_z_vect]).T



        self.axis.plot(ref[0], ref[1], ref[2], color='red', linewidth=2.0)

        return 0

    def draw_arm_trajectory(self, q_list):
        """
        draw an animation of the arm following each position in X_list
        :param X_list: list of x,y,z coordinates of the points shape : (len(data), 3, nb_points_in_the_arm)
        """
        nb_frame = q_list.shape[0]
        print(nb_frame)

        def update_lines(frame, plot, scatter):
            q = q_list[frame]
            [x,y,z] = self.arm.IK.compute_pose(q)
            plot.set_data(x, y)
            plot.set_3d_properties(z)
            scatter.set_data(x, y)
            scatter.set_3d_properties(z)
            return 0

        # Attaching 3D axis to the figure
        fig = self.Figure
        ax = self.axis

        # Setting the axes properties
        self.clear_figure()

        # init data
        [x, y, z] = self.arm.IK.compute_pose(q_list[0])

        # Creating fifty line objects.
        # NOTE: Can't pass empty arrays into 3d version of plot()
        plot = ax.plot(x, y, z, color='orange', linewidth=3.0)
        scatter = ax.scatter(x, y, z, linewidth=3.0)


        # Creating the Animation object
        line_ani = animation.FuncAnimation(fig, update_lines, nb_frame, fargs=(plot,scatter), interval=50, blit=False)


        self.FigureCanvas.draw()

        return 0

