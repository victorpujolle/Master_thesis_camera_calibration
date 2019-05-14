import numpy as np
from utils import *
import scipy.optimize


class FK_Generator():
    """
    This class is used for all formward kinematics calculation of the arm
    """

    def __init__(self, L, OFFSET, ORIGIN_TRANS, Q_MIN, Q_MAX):
        """
        init the const value of the arm
        :param L: length of each links
        :param Q_MIN: values of each q_min for each links (unit : rad)
        :param Q_MAX: values of each q_max for each links (unit : rad)
        """
        self.L = L
        self.OFFSET = OFFSET
        self.ORIGIN_TRANS = ORIGIN_TRANS
        self.Q_MIN = Q_MIN
        self.Q_MAX = Q_MAX

        self.T0 = np.eye(4)
        self.T1 = np.eye(4)
        self.T2 = np.eye(4)
        self.T3 = np.eye(4)
        self.T4 = np.eye(4)
        self.T5 = np.eye(4)

    # basic 3D matrix generator
    def Trans(self, x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
    def RotX(self, alpha):
        return np.array([
            [ 1 ,       0        ,        0        , 0 ],
            [ 0 , np.cos(alpha), -np.sin(alpha), 0 ],
            [ 0 , np.sin(alpha),  np.cos(alpha), 0 ],
            [ 0 ,       0        ,        0        , 1 ]
        ])
    def RotY(self, beta):
        return np.array([
            [  np.cos(beta), 0 , np.sin(beta), 0 ],
            [       0        , 1 ,      0        , 0 ],
            [ -np.sin(beta), 0 , np.cos(beta), 0 ],
            [       0        , 0 ,      0        , 1 ]
        ])
    def RotZ(self, gamma):
        return np.array([
            [ np.cos(gamma), -np.sin(gamma), 0 , 0 ],
            [ np.sin(gamma),  np.cos(gamma), 0 , 0 ],
            [       0        ,        0        , 1 , 0 ],
            [       0        ,        0        , 0 , 1 ]
        ])

    '''Transformation matrix of each joint & certain part'''

    def compute_T0(self):
        # return self.RotY(math.pi/2) @ self.Trans(0,0,-self.L[0])
        #self.T0 = self.RotY(np.pi / 2).dot(self.Trans(0, 0, self.L[0]))
        self.T0 = self.Trans(self.ORIGIN_TRANS[0], self.ORIGIN_TRANS[1], self.ORIGIN_TRANS[2] + self.L[0])
        return self.T0

    def compute_T1(self, q1):
        # return self.RotZ(q1) @ self.Trans(0,0,-self.L[1]) @ self.RotX(math.pi/2)
        self.T1 = self.RotZ(q1).dot(self.Trans(0, 0, 0).dot(self.RotX(np.pi / 2)))
        return self.T1

    def compute_T2(self, q2):
        self.T2 = self.RotZ(q2 + np.pi).dot(self.Trans(0, 0, 0).dot(self.RotX(np.pi / 2)))
        #self.T2 = self.RotZ(q2).dot(self.Trans(self.L[1],0,0))
        return self.T2

    def compute_Tel(self): # elbow
        self.T_el = self.Trans(0, 0, self.L[1] + self.L[2])
        return self.T_el

    def compute_T3(self, q3):
        #self.T3 = self.RotZ(q3 + np.pi / 2).dot(self.Trans(0, 0, 0).dot(self.RotX(np.pi / 2)))
        self.T3 = self.RotZ(q3).dot(self.Trans(0, 0, 0).dot(self.RotX(np.pi / 2)))
        return self.T3

    def compute_T4(self, q4):
        self.T4 = self.RotZ(-q4).dot(self.Trans(0, 0, 0).dot(self.RotX(-np.pi / 2)))
        return self.T4

    def compute_Twr(self):  # wrest
        self.T_wr = self.RotZ(-np.pi / 2).dot(self.Trans(0, 0, self.L[3] + self.L[4]))
        return self.T_wr

    def compute_T5(self, q5):
        self.T5 = self.RotZ(q5 + np.pi / 2).dot(self.Trans(0, 0, 0).dot(self.RotX(-np.pi / 2)))
        return self.T5

    def compute_T6(self, q6):
        self.T6 = self.RotZ(q6).dot(self.Trans(0, 0, 0).dot(self.RotX(np.pi / 2).dot(self.RotZ(-np.pi / 2))))
        return self.T6

    def compute_Teef(self):  # end effector
        self.T_eef = self.Trans(0, 0, self.L[5])
        return self.T_eef

    ''' Transformation matrix from global coordinate to an certain part'''

    def compute_T0_sh(self, q1, q2):  # shoulder
        self.Tglobal2sh =  self.compute_T0().dot(self.compute_T1(q1).dot(self.compute_T2(q2)))
        return self.Tglobal2sh

    def compute_T0_el(self, q1, q2, q3, q4):  # elbow
        self.Tglobal2el = self.compute_T0_sh(q1, q2).dot(self.compute_Tel().dot(self.compute_T3(q3).dot(self.compute_T4(q4))))
        return self.Tglobal2el

    def compute_T0_wr(self, q1, q2, q3, q4, q5, q6):  # wrest
        self.Tglobal2wr = self.compute_T0_el(q1, q2, q3, q4).dot(self.compute_Twr().dot(self.compute_T5(q5).dot(self.compute_T6(q6))))
        return self.Tglobal2wr

    def compute_T0_eef(self, q1, q2, q3, q4, q5, q6):  # end effector
        self.Tglobal2eef = self.compute_T0_wr(q1, q2, q3, q4, q5, q6).dot(self.compute_Teef())
        return self.Tglobal2eef


    # toolkit for homogenous coordinates
    def homogenous2space(self,v):
        """
        project the homogenous vector v to the projective space in 3D
        :param v: homogenous vector
        :return: projected vector
        """
        return np.array([v[0]/v[3], v[1]/v[3], v[2]/v[3]])

    def space2homogenous(self,v):
        """
        project of 3D point into the homogenous considering it is not at the infinity
        :param v: 3D vector
        :return: projected homogenous vector
        """
        return np.array([v[0], v[1], v[2], 1])

    def homo_add(self,v,w):
        """
        add two homogenous vectors
        :param v: vector
        :param w: vector
        :return: v + w
        """
        return self.space2homogenous(self.homogenous2space(v) + self.homogenous2space(w))

    # transfert matrices computation with DH parameters
    def calculate_DH_param(self, q):
        """
        Initialisation of the DH parameters of the arm (change this function for your arm)
        :param q: the q parameters of your arm, here the angle of each joints
        """
        self.DH_parameters = np.array([
            [self.L[0], q[0], 0 , 0],
            [0, q[1], self.L[1], np.pi / 2],
            [self.L[2], q[2], 0, 0],
            [0, q[3], self.L[3], np.pi / 2],
            [self.L[4], q[4], 0, 0],
            [0, q[5], self.L[5], np.pi / 2],
        ])

        return 0
    def calculate_T(self, d, theta, r, alpha):
        """
        calculate the Denavit and Hartenberg matrix for a joint
        :param d: link offset
        :param r: link length
        :param alpha: link twist
        :param theta: link angle
        :return: the Denavit and Hartenberg matrix
        """
        T = np.zeros((4,4))

        ct, st, ca, sa = 0, 0, 0, 0

        if  round(theta, 5) == 0: # zero case
            ct = 1
            st = 0
        elif round(theta, 5) == round(np.pi / 2, 5): # pi/2 case
            ct = 0
            st = 1
        elif round(theta, 5) == round(np.pi, 5):  # pi case
            ct = -1
            st = 0
        elif round(theta, 5) == round(3 * np.pi / 2, 5):  # 3pi/2 case
            ct = 0
            st = -1
        else: # general case
            ct = np.cos(theta)
            st = np.sin(theta)

        if round(alpha, 5) == 0: # zero case
            ca = 1
            sa = 0
        elif round(alpha, 5) == round(np.pi/2, 5): # pi/2 case
            ca = 0
            sa = 1
        elif round(alpha, 5) == round(np.pi, 5):  # pi case
            ca = -1
            sa = 0
        elif round(alpha, 5) == round(3 * np.pi / 2, 5):  # 3pi/2 case
            ca = 0
            sa = -1
        else: # general case
            ca = np.cos(alpha)
            sa = np.sin(alpha)

        # rotation, first line
        T[0, 0] = ct
        T[0, 1] = - st * ca
        T[0, 2] = st * sa

        # rotation second line
        T[1, 0] = st
        T[1, 1] = ct * ca
        T[1, 2] = - ct * sa

        # rotation last line
        T[2, 1] = sa
        T[2, 2] = ca

        # translation
        T[0, 3] = r * ct
        T[1, 3] = r * st
        T[2, 3] = d

        # last value
        T[3, 3] = 1

        return T

    # forward kinematic computation
    def compute_FK(self, q):
        """
        Compute the forward kinematics of the arm
        :param q: the angle of each joint
        :return: the final rotation matrix and translation
        """
        T = self.compute_T0_eef(q[0],q[1],q[2],q[3],q[4],q[5])
        R = T[0:3,0:3]
        t = T[0:3,3]

        return R, t

    # pose computation for drawing
    def compute_pose(self, q):
        """
        Compute the position of each joints
        :param q: the angle of each joint
        :return: the list of x, y, z coord of each joint
        """
        joint_pos_init = np.array([0,0,0,1])
        #self.compute_T_wr(q[0], q[1], q[2], q[3], q[4], q[5])
        self.compute_T0_eef(q[0], q[1], q[2], q[3], q[4], q[5]) # will compute all the transformation matrices

        origin = self.Trans(0, 0, 0)[0:3, -1]
        T0_sh  = self.compute_T0_sh(q[0], q[1])[0:3, -1]
        T0_el  = self.compute_T0_el(q[0], q[1], q[2], q[3])[0:3, -1]
        T0_wr  = self.compute_T0_wr(q[0], q[1], q[2], q[3], q[4], q[5])[0:3, -1]
        T0_eef = self.compute_T0_eef(q[0], q[1], q[2], q[3], q[4], q[5])[0:3, -1]

        x = [0, T0_sh[0], T0_el[0], T0_wr[0], T0_eef[0]]
        y = [0, T0_sh[1], T0_el[1], T0_wr[1], T0_eef[1]]
        z = [0, T0_sh[2], T0_el[2], T0_wr[2], T0_eef[2]]

        return x,y,z

    def compute_ref(self,q):
        """
        Compute all the local referencial to help the visualisation of the arm
        :param q: the angle of each joint
        :return: the list with all the referencials
        """
        # reference frame
        frame = np.eye(3)

        # base ref to draw
        origin = self.Trans(0, 0, 0)
        T0_sh  = self.compute_T0_sh(q[0], q[1])
        T0_el  = self.compute_T0_el(q[0], q[1], q[2], q[3])
        T0_wr  = self.compute_T0_wr(q[0], q[1], q[2], q[3], q[4], q[5])
        T0_eef = self.compute_T0_eef(q[0], q[1], q[2], q[3], q[4], q[5])

        origin_T0 = origin[0:3, -1]
        origin_T0_sh  = T0_sh[0:3, -1]
        origin_T0_el  = T0_el[0:3, -1]
        origin_T0_wr  = T0_wr[0:3, -1]
        origin_T0_eef = T0_eef[0:3, -1]

        rot_T0_sh  = T0_sh [0:3, 0:3]
        rot_T0_el  = T0_el [0:3, 0:3]
        rot_T0_wr  = T0_wr [0:3, 0:3]
        rot_T0_eef = T0_eef[0:3, 0:3]

        frame_sh = rot_T0_sh.dot(frame)
        frame_el = rot_T0_el.dot(frame)
        frame_wr = rot_T0_wr.dot(frame)
        frame_eef = rot_T0_eef.dot(frame)

        list_vector_frame = np.array([frame, frame_sh, frame_el, frame_wr, frame_eef])
        list_origin_frame = np.array([origin_T0, origin_T0_sh, origin_T0_el, origin_T0_wr, origin_T0_eef])

        return list_vector_frame, list_origin_frame


class IK_Generator(FK_Generator):
    """
    This class is used for all inverse kinematics calculation of the arm
    It inherites from the forward generator class
    """
    def __init__(self, L, OFFSET, ORIGIN_TRANS, Q_MIN, Q_MAX):

        # super init
        super(IK_Generator, self).__init__(L, OFFSET, ORIGIN_TRANS, Q_MIN, Q_MAX)

    def FK(self,q):
        """
        computes the FK
        :param q: angles of joints
        :return: X, Y, Z, aX, aY, aZ position and angles of each axis
        """
        T = self.compute_T0_eef(q[0], q[1], q[2], q[3], q[4], q[5])
        [X, Y, Z] = T[0:3, -1]
        R = T[0:3, 0:3]
        angles = rotationMatrixToEulerAngles(R)

        return X, Y, Z, angles[0], angles[1], angles[2]

    def compute_J(self, q, h=0.001):
        """
        this function compute a numerical approximation of the jacobian of the FK
        :param q: angles of joints
        :param h: the computational step
        :return: J
        """
        J = np.zeros((len(q), 6))
        q_copy = np.copy(q)
        f_q = np.array(self.FK(q_copy))
        
        for i in range(len(q)):
            q_copy = q
            q_copy[i] += h
            f_qh = np.array(self.FK(q_copy))
            J[i] = (f_qh - f_q) / h


        return J.T # J is actually the transpose of the matrix we calculated

    def compute_J_plus(self, q, h=0.001):
        """
        This function computes the Moore*Penrose pseudoinverse of J
        :param q: angles of joints
        :param h: the computational step
        :return: J+
        """
        J = self.compute_J(q, h)
        J_plus = np.linalg.pinv(J)

        return J_plus

    def norm_euclidienne(self, q1, q2):
        """
        defines the norm used to mesure the space during the optimisation
        :param q1: [X,Y,Z,aX,aY,aZ]
        :param q2: [X,Y,Z,aX,aY,aZ]
        :return: the norm between q1 and q2 : d, for now it works but there is no physical meaning to this distance
        """
        [X1,Y1,Z1,aX1,aY1,aZ1] = q1
        [X2,Y2,Z2,aX2,aY2,aZ2] = q2

        P = eulerAnglesToRotationMatrix([aX1,aY1,aZ1])
        Q = eulerAnglesToRotationMatrix([aX2, aY2, aZ2])
        theta = norm_geodesic(P,Q)

        d = np.sqrt((X1 - X2)**2 + (Y1 - Y2)**2 +(Z1 - Z2)**2)

        return d

    def norm_eval(self, q1, q2, w = 1):
        """
        defines the norm used to mesure the space during the evaluation
        :param q1: [X,Y,Z,aX,aY,aZ]
        :param q2: [X,Y,Z,aX,aY,aZ]
        :param w: weigth of the distance compare to the angle
        :return: the norm between q1 and q2 : d, for now it works but there is no physical meaning to this distance
        """
        [X1, Y1, Z1, aX1, aY1, aZ1] = q1
        [X2, Y2, Z2, aX2, aY2, aZ2] = q2

        P = eulerAnglesToRotationMatrix([aX1, aY1, aZ1])
        Q = eulerAnglesToRotationMatrix([aX2, aY2, aZ2])
        theta = norm_geodesic(P, Q) * 180/(2*np.pi)

        d = np.sqrt((X1 - X2) ** 2 + (Y1 - Y2) ** 2 + (Z1 - Z2) ** 2)

        return [d**2 * theta**2, d, theta]

    def norm_direction(self, eef, goal):
        """
        the norm used to control the robot holding the camera
        the line continues the eef, the returned value is the distance between the goal point and this line
        :param eef: position of the eef
        :param goal: position of the goal (the angle is not important)
        """
        [X1, Y1, Z1, aX1, aY1, aZ1] = eef # eef
        [X2, Y2, Z2, _, _, _] = goal # goal

        R = eulerAnglesToRotationMatrix([aX1, aY1, aZ1])
        uz = np.array([0,0,1])
        u = np.dot(R,uz)

        B = np.array([X1, Y1, Z1])
        A = np.array([X2, Y2, Z2])
        BA = A-B
        n = np.linalg.norm(u)
        BA_u = np.cross(BA,u)
        d = np.linalg.norm(BA_u)/n

        return d

    def gradient_descent(self, goal, q_in):
        """
        Computes the gradient descent for the IK
        :param goal: goal point and orientation
        :param q_in: initial guess for the angles
        """

        # loss function used to find the position
        def loss_direction(q):
            fk = self.FK(q)
            n = self.norm_direction(fk, goal)
            return n

        # loss function used the find a correct first guess
        def loss_distance(q):
            fk = self.FK(q)
            n = self.norm_euclidienne(fk, goal)
            return n

        # angles boundaries
        bounds = [(np.pi / 180 * self.Q_MIN[i], np.pi / 180 * self.Q_MAX[i]) for i in range(len(self.Q_MAX))]
        # bounds only for L-BFGS-B, TNC, SLSQP and trust-constr method

        # optimization algo
        # limiting the number of iteration for the first opti
        options1 = {'eps': 1e-08, 'scale': None, 'offset': None, 'mesg_num': None, 'maxCGit': -1, 'maxiter': 10,
                   'eta': -1, 'stepmx': 0, 'accuracy': 0, 'minfev': 0, 'ftol': -1, 'xtol': -1, 'gtol': -1,
                   'rescale': -1, 'disp': False}

        options2 = {'eps': 1e-08, 'scale': None, 'offset': None, 'mesg_num': None, 'maxCGit': -1, 'maxiter': None,
                   'eta': -1, 'stepmx': 0, 'accuracy': 0, 'minfev': 0, 'ftol': -1, 'xtol': -1, 'gtol': -1,
                   'rescale': -1, 'disp': False}
        print('hello there')
        res0 = scipy.optimize.minimize(loss_distance, q_in, method='TNC', jac='2-point' , bounds=bounds, options=options1)
        x0 = res0['x']
        res = scipy.optimize.minimize(loss_direction, x0, method='TNC', jac='2-point' , bounds=bounds, options=options2)


        return (res)








        


