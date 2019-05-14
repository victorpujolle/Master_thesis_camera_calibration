import time

from DXSerialAPI import DXSerialAPI
from utils import *
from kinematic_generator import FK_Generator, IK_Generator

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

class Arm(DXSerialAPI):
    """
    This class represent my arm and can be used to control it
    """


    def __init__(self, PORT_NAME, BAUDRATE, TIME_OUT=0.001, joint_number=6, motor_number=6, motors_id=None):

        # __init__ of the parent class
        super(Arm, self).__init__(PORT_NAME, BAUDRATE, TIME_OUT=TIME_OUT)

        # general properties of the arm
        self.joint_number = joint_number # number of joint of the arm
        self.motor_number = motor_number # number of motor of the arm
        self.motors_id = motors_id # i :  the id of the motors, motors_id : real id of the device (in term of serial communication)

        # motors values that have to be initialized by reading the state of each motors in the - initialisation - function
        self.motors_angle_limits_byte = [] # the i-th element this list will contains [angle_clockwise_limit , angle_counterclockwise_limit] of the i-th motor (byte units used)
        self.motors_torque_limits_byte = [] # the i-th element this list will contains [max torque] of the i-th motor (byte units used)
        self.motors_angles_byte = [] # the angles of each motors (byte units used)
        self.motors_speed_byte = [] # the speed of each motors (byte unit used) (the speed of the motors has to be initialized first)

        # class constant init
        self.ANGLES_INIT = np.array([-30, 45, 0, -90, 0, -90]) # angles initialisation of each JOINTS (not motors!!!!)
        self.DX_SPEED = 100 # speed of the motors
        self.LINKS_LENGTH = [0.045, 0.11, 0.04, 0.04, 0.11, 0.13] # links length
        self.TRANS_ORIGIN = [0, 0, 0.26] # the translation between the origin of the world referecne and the origin of the robot
        self.ANGLE_OFFSET = np.array([150, 132, 150, 150, 150, 150]) # angle offset used to calculate DH parameters (because the angle origin of the motors is not the same that the angle origin of the DH param)
        self.ANGLE_LIMIT_INF = [-90, -90, -90, -90, -90, -90]
        self.ANGLE_LIMIT_SUP = [90, 90, 90, 90, 90, 90]

        # usefull flags
        self.flag_is_position_init = False # become true when the position of each joint has been initialize in set_arm_position (or initialize_position)
        self.flag_is_DH_param_init = False # become true when the DH parameters of each joint has been initialize in set_arm_position (or initialize_position)

        # arm values, this values need to be setted for any kinematics computation
        self.joint_position = np.zeros((self.joint_number)) # position of each joint

        # variables used for kinematics computation
        self.FK = FK_Generator(self.LINKS_LENGTH, self.ANGLE_OFFSET, self.TRANS_ORIGIN, self.ANGLE_LIMIT_INF, self.ANGLE_LIMIT_SUP)
        self.IK = IK_Generator(self.LINKS_LENGTH, self.ANGLE_OFFSET, self.TRANS_ORIGIN, self.ANGLE_LIMIT_INF, self.ANGLE_LIMIT_SUP)


    # ------------- READING -------------

    def test_multiple_id(self, list_id):
        """
        this function will test all the id in list_id
        :param list_id: list of id to be tested
        :return: exist: boolean list, if the motor list_id[i] exists then exist[i] == True
        """
        exist = []
        for id in list_id:
            exists, _ = self._PING(id)
            exist.append(exists)

        return exist

    def test_motors(self):
        """
        this function test if all the motors that are supposed to be in the arm exist
        this function use the variable motors_id, be sure to correctly initialize this variable
        :return: all_motors_exists, details: all_motors_exists is True if all motors exists, False otherwise, details give you details about which motors exist or not
                 details_msg: the responces of each motors
        """
        details = []
        details_msg = []
        all_motors_exists = True
        if self.motors_id == None:
            raise ValueError('self.motors_id is None, please link you motors')
        else:
            for id in self.motors_id:
                exists, message = self._PING(id)
                details.append(exists)
                details_msg.append(message)
                if not(exists): all_motors_exists = False

        return all_motors_exists, details, details_msg

    def read_arm_postion(self):
        """
        This function read the position of each motors
        """

        self.motors_angles = []

        for motor in self.motors_id:
            self.motors_angles.append(round(self.read_present_position_byte(motor) * self.ANGLE_UNIT,2))

        #print(self.motors_angles)

        joint_angles = self.motors_angles.copy()
        joint_angles.pop(2)
        self.joint_angles = np.array(joint_angles) - self.ANGLE_OFFSET

        print(self.joint_angles)

        return 0

    # -----------------------------------

    # ------------- INITIALIZATION -------------

    def initialize_speed(self, init_speed=None):
        """
        This function initialize the rotation speed of all the motor
        :param init_speed: the speed you want (in rmp) (if None, the class default speed will be taken)
        """
        speed = self.DX_SPEED if init_speed == None else init_speed
        self.set_moving_speed(0xFE, speed) # 0xFE is the broadcast ID, all the motors will read the signal

        return 0

    def is_angle_valid(self, motor_id, angle, error_raising=True):
        """
        The functions tests if the angle is a valid goal postion value of the motor, this function will also raise an error if the mode error_raising is set to true
        :param motor_id: id of the motor
        :param angle: angle (degree) that you want to test
        :param error_raising : true or false depending if you want to raise an error or not
        :return: True if the angle given is a valid value, False otherwise
        """
        if angle > 90 or angle < 90:
            return True

        else:
            if error_raising:
                raise ValueError('The angle value ({} given) have to be between the limit values [90 , 90] of the motor {}'.format(angle))
            return False

    def initialize_position(self, init_angles=None):
        """
        This function initialize the angle of each JOINT (this is actualy tricky if you have multiple motors for the same joint, this if why you should
        REWRITE THE UGLY FUNCTION _write_single_joint_position WITH ALL THE IF STATEMENTS for your robot (sorry but this is the more robust and simple way to sovle this problem)
        :param init_angles: init_angles[i] is the angle (in degree) of the i-th joint
        """

        angles = self.ANGLES_INIT if init_angles == None else init_angles

        self.set_arm_position(angles)

        return 0

    # -----------------------------------

    # ------------- CONTROLS -------------

    def write_single_joint_position(self, joint_id, angle):
        """
        :param joint_id: the id of the JOINT
        :param angle: the angle you want for the joint, if angle is '' it does nothing
        depending of the configuration of your robot, you have to rewrite this function
        """

        angle += self.ANGLE_OFFSET[joint_id]

        if joint_id == 0:
            if self.is_angle_valid(0, angle, error_raising=True):
                self.set_reg_goal_position(0, angle)

        if joint_id == 1:
            if self.is_angle_valid(1, angle, error_raising=True) and self.is_angle_valid(2, 300 - angle, error_raising=True):
                self.set_reg_goal_position(1, angle)
                self.set_reg_goal_position(2, 300 - angle)

        if joint_id == 2:
            if self.is_angle_valid(3, angle, error_raising=True):
                self.set_reg_goal_position(3, angle)

        if joint_id == 3:
            if self.is_angle_valid(4, angle, error_raising=True):
                self.set_reg_goal_position(4, angle)

        if joint_id == 4:
            if self.is_angle_valid(5, angle, error_raising=True):
                self.set_reg_goal_position(5, angle)

        if joint_id == 5:
            if self.is_angle_valid(6, angle, error_raising=True):
                self.set_reg_goal_position(6, angle)

        if joint_id == 6:
            if self.is_angle_valid(7, angle, error_raising=True):
                self.set_reg_goal_position(7, angle)

        return 0

    def set_arm_position(self, angles):
        """
        This function sets the position of all the joints
        :param angles: the goal position (degree units)
        """
        #print('set_arm_position :', angles)
        if len(angles) != self.joint_number:
            raise ValueError('The length of the input angles : the input list has a length of {} and  this length should be the number of joint {}'.format(len(angles),self.joint_number))

        for joint in range(self.joint_number):
            if angles[joint] != '':
                self.write_single_joint_position(joint, angles[joint])
                self.joint_position[joint] = angles[joint]

        self.flag_is_position_init = True # the position is now setted

        return 0

    def activate_all_reg(self):
        """
        This function send the ACTION message to all motors
        """
        self._ACTION(0xFE)

        return 0







    # TODO : forward cinematic function, backward cinematic solver


