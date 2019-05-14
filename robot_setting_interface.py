import sys
import os

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5 import QtWidgets,QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from Arm import Arm

from utils import *

import glob
import socket
import math


class robot_setting_interface(QtWidgets.QWidget):
    """
    The goal of this interface is the control of the robot
    """


    def __init__(self,arm):

        # super init
        super(robot_setting_interface, self).__init__()

        # link to the arm
        self.arm = arm

        #init UI
        self.init_UI()

        # init figure
        self.init_figure()

        # init menu
        self.init_menu()

    #------INIT FUNCTIONS-----

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
        self.setGeometry(0, 0, 1100, 600)
        self.FigureWidget.setGeometry(600, 0, 700, 600)
        self.MenuWidget.setGeometry(0, 0, 400, 600)

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
        self.robotpos_label = QtWidgets.QLabel(self)
        self.robotmove_label = QtWidgets.QLabel(self)

        self.th0label = QtWidgets.QLabel(self)
        self.th1label = QtWidgets.QLabel(self)
        self.th2label = QtWidgets.QLabel(self)
        self.th3label = QtWidgets.QLabel(self)
        self.th4label = QtWidgets.QLabel(self)
        self.th5label = QtWidgets.QLabel(self)


        # Create the textbox objects

        self.simu0_textbox = QtWidgets.QLineEdit(self)
        self.simu1_textbox = QtWidgets.QLineEdit(self)
        self.simu2_textbox = QtWidgets.QLineEdit(self)
        self.simu3_textbox = QtWidgets.QLineEdit(self)
        self.simu4_textbox = QtWidgets.QLineEdit(self)
        self.simu5_textbox = QtWidgets.QLineEdit(self)

        self.robotread0_textbox = QtWidgets.QLineEdit(self)
        self.robotread1_textbox = QtWidgets.QLineEdit(self)
        self.robotread2_textbox = QtWidgets.QLineEdit(self)
        self.robotread3_textbox = QtWidgets.QLineEdit(self)
        self.robotread4_textbox = QtWidgets.QLineEdit(self)
        self.robotread5_textbox = QtWidgets.QLineEdit(self)

        self.robotmove0_textbox = QtWidgets.QLineEdit(self)
        self.robotmove1_textbox = QtWidgets.QLineEdit(self)
        self.robotmove2_textbox = QtWidgets.QLineEdit(self)
        self.robotmove3_textbox = QtWidgets.QLineEdit(self)
        self.robotmove4_textbox = QtWidgets.QLineEdit(self)
        self.robotmove5_textbox = QtWidgets.QLineEdit(self)


        # Create the button objects
        self.drowbutton = QtWidgets.QPushButton('draw', self)
        self.readposbutton = QtWidgets.QPushButton('read', self)
        self.movebutton = QtWidgets.QPushButton('move', self)
        self.transfertbutton_right = QtWidgets.QPushButton('=>', self)
        self.transfertbutton_left = QtWidgets.QPushButton('<=', self)

        # Resize text box
        self.simu0_textbox.resize(100, 20)
        self.simu1_textbox.resize(100, 20)
        self.simu2_textbox.resize(100, 20)
        self.simu3_textbox.resize(100, 20)
        self.simu4_textbox.resize(100, 20)
        self.simu5_textbox.resize(100, 20)

        self.robotread0_textbox.resize(100, 20)
        self.robotread1_textbox.resize(100, 20)
        self.robotread2_textbox.resize(100, 20)
        self.robotread3_textbox.resize(100, 20)
        self.robotread4_textbox.resize(100, 20)
        self.robotread5_textbox.resize(100, 20)

        self.robotmove0_textbox.resize(100, 20)
        self.robotmove1_textbox.resize(100, 20)
        self.robotmove2_textbox.resize(100, 20)
        self.robotmove3_textbox.resize(100, 20)
        self.robotmove4_textbox.resize(100, 20)
        self.robotmove5_textbox.resize(100, 20)

        # set the size of the button object
        self.drowbutton.resize(100, 30)
        self.readposbutton.resize(100, 30)
        self.movebutton.resize(100, 30)
        self.transfertbutton_right.resize(50, 30)
        self.transfertbutton_left.resize(50, 30)

        # set the name of each label
        self.inputlabel.setText('INPUT SIMULATION')
        self.robotpos_label.setText('ROBOT POSITION')
        self.robotmove_label.setText('ROBOT ORDER')

        self.th0label.setText('th0 :')
        self.th1label.setText('th1 :')
        self.th2label.setText('th2 :')
        self.th3label.setText('th3 :')
        self.th4label.setText('th4 :')
        self.th5label.setText('th5 :')

        # set the location of each labels


        self.inputlabel.move(15, 10)

        self.th0label.move(15, 30)
        self.th1label.move(15, 30 +   30)
        self.th2label.move(15, 30 + 2*30)
        self.th3label.move(15, 30 + 3*30)
        self.th4label.move(15, 30 + 4*30)
        self.th5label.move(15, 30 + 5*30)

        self.simu0_textbox.move(15 + 35, 30)
        self.simu1_textbox.move(15 + 35, 30 +   30)
        self.simu2_textbox.move(15 + 35, 30 + 2*30)
        self.simu3_textbox.move(15 + 35, 30 + 3*30)
        self.simu4_textbox.move(15 + 35, 30 + 4*30)
        self.simu5_textbox.move(15 + 35, 30 + 5*30)

        self.robotread0_textbox.move(360, 30)
        self.robotread1_textbox.move(360, 30 +   30)
        self.robotread2_textbox.move(360, 30 + 2*30)
        self.robotread3_textbox.move(360, 30 + 3*30)
        self.robotread4_textbox.move(360, 30 + 4*30)
        self.robotread5_textbox.move(360, 30 + 5*30)

        self.robotmove_label.move(200,10)

        self.robotmove0_textbox.move(200, 30)
        self.robotmove1_textbox.move(200, 30 +   30)
        self.robotmove2_textbox.move(200, 30 + 2*30)
        self.robotmove3_textbox.move(200, 30 + 3*30)
        self.robotmove4_textbox.move(200, 30 + 4*30)
        self.robotmove5_textbox.move(200, 30 + 5*30)

        self.robotpos_label.move(360, 10)



        self.drowbutton.move(15 + 35, 220)
        self.readposbutton.move(360, 220)
        self.movebutton.move(200, 220)
        self.transfertbutton_right.move(150, 220)
        self.transfertbutton_left.move(150, 250)

        #connect the button to the click_action
        self.drowbutton.clicked.connect(self.on_draw_click)
        self.readposbutton.clicked.connect(self.on_readpos_click)
        self.movebutton.clicked.connect(self.on_move_click)
        self.transfertbutton_right.clicked.connect(self.on_transfert_button_right)
        self.transfertbutton_left.clicked.connect(self.on_transfert_button_left)

        return 0

    #-----BUTTON FUNCTIONS-----

    def on_draw_click(self):
        """
        method linked to the button draw
        """
        print('--- EVENT : DRAW CLICK ---')
        q_deg = np.array(self.get_simu_input()).astype(float)
        q_rad = deg2rad(q_deg)
        print('q [deg] :', q_deg)
        print('q [rad] :', q_rad)

        # figure and UI control
        R,t = self.arm.FK_Generator.compute_FK(q_rad) # compute the total kinematics

        x, y, z = self.arm.FK_Generator.compute_pose(q_rad)  # compute all partial kinematic

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

        self.robotread0_textbox.setText('{:6.2f}'.format(pos[0]))
        self.robotread1_textbox.setText('{:6.2f}'.format(pos[1]))
        self.robotread2_textbox.setText('{:6.2f}'.format(pos[2]))
        self.robotread3_textbox.setText('{:6.2f}'.format(pos[3]))
        self.robotread4_textbox.setText('{:6.2f}'.format(pos[4]))
        self.robotread5_textbox.setText('{:6.2f}'.format(pos[5]))

        return 0

    def on_move_click(self):
        """
        method linked to the button move
        """
        print('--- EVENT : MOVE CLICK ---')
        th0, th1, th2, th3, th4, th5 = self.get_move_input()
        X = [th0, th1, th2, th3, th4, th5]
        angles = [float(X[i]) if (X[i] != '' ) else X[i] for i in range(len(X))]
        self.arm.set_arm_position(angles)
        return 0

    def on_transfert_button_right(self):
        """
        method linked to the button transfert =>
        """
        print('--- EVENT : TRANSFERT => CLICK ---')
        th0, th1, th2, th3, th4, th5 = self.get_simu_input()
        angles = [th0, th1, th2, th3, th4, th5]
        print(angles)
        self.set_move_input(angles)
        return 0

    def on_transfert_button_left(self):
        """
        method linked to the button transfert <=
        """
        print('--- EVENT : TRANSFERT <= CLICK ---')
        th0, th1, th2, th3, th4, th5 = self.get_move_input()
        angles = [th0, th1, th2, th3, th4, th5]
        print(angles)
        self.set_simu_input(angles)
        return 0

    #-----GETTER AND SETTER-----

    def get_simu_input(self):
        """
        getter of the input values
        :return: return the input values
        """
        th0 = self.simu0_textbox.text()
        th1 = self.simu1_textbox.text()
        th2 = self.simu2_textbox.text()
        th3 = self.simu3_textbox.text()
        th4 = self.simu4_textbox.text()
        th5 = self.simu5_textbox.text()

        if th0 == '':
            th0 = '0'
            self.simu0_textbox.setText('0')

        if th1 == '':
            th1 = '0'
            self.simu1_textbox.setText('0')

        if th2 == '':
            th2 = '0'
            self.simu2_textbox.setText('0')

        if th3 == '':
            th3 = '0'
            self.simu3_textbox.setText('0')

        if th4 == '':
            th4 = '0'
            self.simu4_textbox.setText('0')

        if th5 == '':
            th5 = '0'
            self.simu5_textbox.setText('0')


        return th0, th1, th2, th3, th4, th5

    def get_move_input(self):
        """
        getter of the input values
        :return: return the input values
        """
        th0 = self.robotmove0_textbox.text()
        th1 = self.robotmove1_textbox.text()
        th2 = self.robotmove2_textbox.text()
        th3 = self.robotmove3_textbox.text()
        th4 = self.robotmove4_textbox.text()
        th5 = self.robotmove5_textbox.text()

        return th0, th1, th2, th3, th4, th5
    
    def set_move_input(self,X):
        """
        method read the postion of the robot and set the textboxes
        """

        self.robotmove0_textbox.setText(X[0])
        self.robotmove1_textbox.setText(X[1])
        self.robotmove2_textbox.setText(X[2])
        self.robotmove3_textbox.setText(X[3])
        self.robotmove4_textbox.setText(X[4])
        self.robotmove5_textbox.setText(X[5])

        return 0

    def set_simu_input(self,X):
        """
        method read the postion of the robot and set the textboxes
        """

        self.simu0_textbox.setText(X[0])
        self.simu1_textbox.setText(X[1])
        self.simu2_textbox.setText(X[2])
        self.simu3_textbox.setText(X[3])
        self.simu4_textbox.setText(X[4])
        self.simu5_textbox.setText(X[5])

        return 0

    #-----DRAWING FUNCTIONS-----

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
