import sys
import os

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5 import QtWidgets,QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

#from mymodules import fk_module
#from mymodules import Closed_form_ik
from Arm import Arm

from utils import *

import glob
import socket
import math


class Application(QtWidgets.QWidget):

    def __init__(self,arm):

        # super init
        super(Application, self).__init__()

        # link to the arm
        self.arm = arm

        #init UI
        self.init_UI()

        # init figure
        self.init_figure()

        # init menu
        self.init_menu()

    def init_UI(self):
        """
        initialisation of the UI windows
        """
        self._count = True
        # Create Widget for Figure/Menu
        self.FigureWidget = QtWidgets.QWidget(self)
        self.MenuWidget = QtWidgets.QWidget(self)

        # Add Layout to each Widget
        self.FigureLayout = QtWidgets.QVBoxLayout(self.FigureWidget)
        self.MenuLayout = QtWidgets.QVBoxLayout(self.MenuWidget)

        # Delete Margin of the layout
        self.FigureLayout.setContentsMargins(0, 0, 0, 0)
        self.MenuLayout.setContentsMargins(0, 0, 0, 0)

        # set Geometry
        self.setGeometry(0, 0, 900, 600)
        self.FigureWidget.setGeometry(400, 0, 700, 600)
        self.MenuWidget.setGeometry(0, 0, 200, 600)

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

    def init_menu(self):
        """
        Initialisation of the menu
        """
        # Create the label objects

        self.inputlabel = QtWidgets.QLabel(self)
        self.outputlabel = QtWidgets.QLabel(self)
        self.robotpos_label = QtWidgets.QLabel(self)


        self.xlabel = QtWidgets.QLabel(self)
        self.ylabel = QtWidgets.QLabel(self)
        self.zlabel = QtWidgets.QLabel(self)

        self.th0label = QtWidgets.QLabel(self)
        self.th1label = QtWidgets.QLabel(self)
        self.th2label = QtWidgets.QLabel(self)
        self.th3label = QtWidgets.QLabel(self)
        self.th4label = QtWidgets.QLabel(self)
        self.th5label = QtWidgets.QLabel(self)

        self.deg0label = QtWidgets.QLabel(self)
        self.deg1label = QtWidgets.QLabel(self)
        self.deg2label = QtWidgets.QLabel(self)
        self.deg3label = QtWidgets.QLabel(self)
        self.deg4label = QtWidgets.QLabel(self)
        self.deg5label = QtWidgets.QLabel(self)

        # Create the textbox objects
        self.xtextbox = QtWidgets.QLineEdit(self)
        self.ytextbox = QtWidgets.QLineEdit(self)
        self.ztextbox = QtWidgets.QLineEdit(self)

        self.deg0textbox = QtWidgets.QLineEdit(self)
        self.deg1textbox = QtWidgets.QLineEdit(self)
        self.deg2textbox = QtWidgets.QLineEdit(self)
        self.deg3textbox = QtWidgets.QLineEdit(self)
        self.deg4textbox = QtWidgets.QLineEdit(self)
        self.deg5textbox = QtWidgets.QLineEdit(self)

        self.throbot0textbox = QtWidgets.QLineEdit(self)
        self.throbot1textbox = QtWidgets.QLineEdit(self)
        self.throbot2textbox = QtWidgets.QLineEdit(self)
        self.throbot3textbox = QtWidgets.QLineEdit(self)
        self.throbot4textbox = QtWidgets.QLineEdit(self)
        self.throbot5textbox = QtWidgets.QLineEdit(self)

        # Create the button objects
        self.drowbutton = QtWidgets.QPushButton('draw', self)
        self.readposbutton = QtWidgets.QPushButton('read', self)
        self.movebutton = QtWidgets.QPushButton('move', self)

        # Resize text box

        self.xtextbox.resize(100, 20)
        self.ytextbox.resize(100, 20)
        self.ztextbox.resize(100, 20)

        self.deg0textbox.resize(100, 20)
        self.deg1textbox.resize(100, 20)
        self.deg2textbox.resize(100, 20)
        self.deg3textbox.resize(100, 20)
        self.deg4textbox.resize(100, 20)
        self.deg5textbox.resize(100, 20)

        self.throbot0textbox.resize(100, 20)
        self.throbot1textbox.resize(100, 20)
        self.throbot2textbox.resize(100, 20)
        self.throbot3textbox.resize(100, 20)
        self.throbot4textbox.resize(100, 20)
        self.throbot5textbox.resize(100, 20)

        # set the size of the button object
        self.drowbutton.resize(100, 30)
        self.readposbutton.resize(100, 30)
        self.movebutton.resize(100,30)

        # set the name of each label
        self.inputlabel.setText('INPUT SIMULATION')
        self.outputlabel.setText('OUTPUT SIMULATION')
        self.robotpos_label.setText('ROBOT POSITION')

        self.xlabel.setText('x :')
        self.ylabel.setText('y :')
        self.zlabel.setText('z :')

        self.th0label.setText('th0 :')
        self.th1label.setText('th1 :')
        self.th2label.setText('th2 :')
        self.th3label.setText('th3 :')
        self.th4label.setText('th4 :')
        self.th5label.setText('th5 :')

        self.deg0label.setText('[deg]')
        self.deg1label.setText('[deg]')
        self.deg2label.setText('[deg]')
        self.deg3label.setText('[deg]')
        self.deg4label.setText('[deg]')
        self.deg5label.setText('[deg]')

        # set the location of each labels


        self.inputlabel.move(15, 10)

        self.th0label.move(15, 30)
        self.th1label.move(15, 30 +   30)
        self.th2label.move(15, 30 + 2*30)
        self.th3label.move(15, 30 + 3*30)
        self.th4label.move(15, 30 + 4*30)
        self.th5label.move(15, 30 + 5*30)

        self.deg0textbox.move(15 + 35, 30)
        self.deg1textbox.move(15 + 35, 30 +   30)
        self.deg2textbox.move(15 + 35, 30 + 2*30)
        self.deg3textbox.move(15 + 35, 30 + 3*30)
        self.deg4textbox.move(15 + 35, 30 + 4*30)
        self.deg5textbox.move(15 + 35, 30 + 5*30)

        self.deg0label.move(200 + 100, 30)
        self.deg1label.move(200 + 100, 30 +   30)
        self.deg2label.move(200 + 100, 30 + 2*30)
        self.deg3label.move(200 + 100, 30 + 3*30)
        self.deg4label.move(200 + 100, 30 + 4*30)
        self.deg5label.move(200 + 100, 30 + 5*30)

        self.throbot0textbox.move(200, 30)
        self.throbot1textbox.move(200, 30 +   30)
        self.throbot2textbox.move(200, 30 + 2*30)
        self.throbot3textbox.move(200, 30 + 3*30)
        self.throbot4textbox.move(200, 30 + 4*30)
        self.throbot5textbox.move(200, 30 + 5*30)

        self.robotpos_label.move(200, 10)

        self.outputlabel.move(15, 230)

        self.xlabel.move(  15, 250)
        self.ylabel.move(  15, 250 + 30)
        self.zlabel.move(  15, 250 + 60)
        self.xtextbox.move(15 + 35, 250)
        self.ytextbox.move(15 + 35, 250 + 30)
        self.ztextbox.move(15 + 35, 250 + 60)

        self.drowbutton.move(15 + 35, 200 + 180)
        self.readposbutton.move(200, 200 + 180)
        self.movebutton.move(15 + 35, 380 + 50)

        #connect the button to the click_action
        self.drowbutton.clicked.connect(self.on_drow_click)
        self.readposbutton.clicked.connect(self.on_readpos_click)
        self.movebutton.clicked.connect(self.on_move_click)

        return 0

    def on_drow_click(self):
        """
        method linked to the button draw
        """
        print('--- EVENT : DRAW CLICK ---')
        q_deg = np.array(self.get_input()).astype(float)
        q_rad = deg2rad(q_deg)
        print('q [deg] :', q_deg)
        print('q [rad] :', q_rad)
        if not self.check_input(q_rad):
            # no valid input
            return 1
        else:
            # valid input

            # figure and UI control
            R,t = self.arm.kinematic.compute_FK(q_rad) # compute the total kinematics
            self.set_output([t[0], t[1], t[2]]) # set the output

            x, y, z = self.arm.kinematic.compute_pose(q_rad)  # compute all partial kinematics
            #list_vector_frame, list_origin_frame = self.arm.kinematic.compute_ref(q)  # compute all the local ref

            self.clear_figure() # clear figure
            self.draw_arm([x, y, z]) # draw the arm
            #self.draw_ref(list_vector_frame, list_origin_frame)
            self.FigureCanvas.draw()

        return 0

    def on_readpos_click(self):
        """
        method read the postion of the robot and set the textboxes
        """
        print('--- EVENT : READ CLICK ---')
        self.arm.read_arm_postion()
        pos = self.arm.joint_angles
        print(pos)

        self.throbot0textbox.setText('{:6.2f}'.format(pos[0]))
        self.throbot1textbox.setText('{:6.2f}'.format(pos[1]))
        self.throbot2textbox.setText('{:6.2f}'.format(pos[2]))
        self.throbot3textbox.setText('{:6.2f}'.format(pos[3]))
        self.throbot4textbox.setText('{:6.2f}'.format(pos[4]))
        self.throbot5textbox.setText('{:6.2f}'.format(pos[5]))

        return 0

    def on_move_click(self):
        """
        method linked to the button move
        """
        print('--- EVENT : MOVE CLICK ---')

    def get_input(self):
        """
        getter of the input values
        :return: return the input values
        """
        th0 = self.deg0textbox.text()
        th1 = self.deg1textbox.text()
        th2 = self.deg2textbox.text()
        th3 = self.deg3textbox.text()
        th4 = self.deg4textbox.text()
        th5 = self.deg5textbox.text()

        if th0 == '':
            th0 = '0'
            self.deg0textbox.setText('0')

        if th1 == '':
            th1 = '0'
            self.deg1textbox.setText('0')

        if th2 == '':
            th2 = '0'
            self.deg2textbox.setText('0')

        if th3 == '':
            th3 = '0'
            self.deg3textbox.setText('0')

        if th4 == '':
            th4 = '0'
            self.deg4textbox.setText('0')

        if th5 == '':
            th5 = '0'
            self.deg5textbox.setText('0')


        return th0, th1, th2, th3, th4, th5

    def check_input(self, q):
        """
        this method check if the input are allowed
        :return: true or false depend of the input
        """
        ANGLE_LIMIT_INF = self.arm.ANGLE_LIMIT_INF
        ANGLE_LIMIT_SUP = self.arm.ANGLE_LIMIT_SUP

        for i in range(len(q)):
            if q[i] < ANGLE_LIMIT_INF[i] or q[i] > ANGLE_LIMIT_SUP[i]:
                print('theta {} is not is the acceptable range {} given, range : [{} {}]'.format(i,q[i],ANGLE_LIMIT_INF[i],ANGLE_LIMIT_SUP[i]))
                return False

        return True

    def set_output(self, X):
        """
        method to set the output text box
        :param X: list [x,y,z]
        """

        self.xtextbox.setText('{:6.2f}'.format(X[0]))
        self.ytextbox.setText('{:6.2f}'.format(X[1]))
        self.ztextbox.setText('{:6.2f}'.format(X[2]))

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

        self.axis.plot(x, y, z, color='orange', linewidth = 3.0)
        self.axis.scatter(x, y, z, linewidth=3.0)

        return 0

    def draw_ref(self, list_vector_frame, list_origin_frame):
        """
        draw on the figure a local reference frame
        :param list_vector_frame: list of reference frame rotated
        :param list_origin_frame: list of origin points for each frame
        """
        # scale calculation, it is better if all the scales are the same
        scale_x = (self.axis_xlim3d[1] - self.axis_xlim3d[0]) * 0.1
        scale_y = (self.axis_ylim3d[1] - self.axis_ylim3d[0]) * 0.1
        scale_z = (self.axis_zlim3d[1] - self.axis_zlim3d[0]) * 0.1

        for i in range(len(list_origin_frame)):

            origin = list_origin_frame[i]
            vectors = list_vector_frame[i]
            x_vect = vectors[0] * scale_x
            y_vect = vectors[1] * scale_y
            z_vect = vectors[2] * scale_z
            ref_to_plot = np.array([origin, origin + x_vect, origin, origin + y_vect, origin, origin + z_vect,]).T
            self.axis.plot(ref_to_plot[0], ref_to_plot[1], ref_to_plot[2], color='grey', linewidth = 2.0)

