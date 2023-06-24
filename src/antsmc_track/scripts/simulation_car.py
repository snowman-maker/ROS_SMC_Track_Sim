#!/usr/bin/env python
# coding=utf-8
import roslib
import rospy
# import PyKDL

# import msgs
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import WrenchStamped
from itertools import chain
# from compiler.ast import flatten
# import services
from std_srvs.srv import Empty
# More imports
import math
import numpy as np
from TD_velocity import TD_func_fast
import time
# import tf
# from sensor_msgs.msg import LaserScan                                                 
# import termios, fcntl, sys, os
# import math
import linecache

import logging
import sys
import colorprint

color = colorprint.Log()
logging.basicConfig(
	stream=sys.stdout,
	format=color.green("[%(asctime)s] {%(filename)s:%(lineno)d} %(levelname)s - %(message)s"),
	level=logging.INFO,
)


class NTSM_Control:

    def __init__(self):

        self.cvg = [[0], [0]]
        self.attract_k = 5
        self.repulsion_k = 10
        self.P_0 = 2.3
        self.left = 0
        self.l_for_center = 0
        self.right = 0
        self.orien_l = 0
        self.orien_r = 0
        self.Fvelocity = 10
        self.fuzzcontrol = 'fuzz.txt'
        self.fuzzc_eta = 'eta.txt'
        self.f_eta = 0.5
        # disturbance
        self.v_d = [[0], [0]]  # velocity
        self.d_est = 0
        self.u_1s = 0
        self.u_2s = 0
        self.u_3s = 0
        self.d_est_dot = 0.0
        self.d_est = 0.0
        self.d_est_ddot = 0.0
        self.d_est_ddot_int = 0.0
        self.last_time = 0.0
        self.vel_track = 0.0
        self.vel_track_dot = 0.0
        self.vel_track_dotreal = 0.0
        self.track_dot = 0.
        self.track = 0.
        # x, y, yaw, v_x, v_y
        self.state_ref = [[0], [0], [0], [0], [0]]
        self.last_y = 0
        self.distance_error = 0.0
        self.dot_error = 0.0
        self.height_error, self.dot_height_error = 0., 0.

    def calc_input(self, x, y, yaw, vel_x, vel_y, vel_yaw, state_ref, move_right):
        # D = 250
        D = 25
        # D_X = 120
        D_X = 12
        M = 1
        self.delta_x = state_ref[0] - x
        self.delta_y = state_ref[1] - y
        # arctangent sliding mode controller
        if M == 1:
            distance_error = self.delta_x
            self.distance_error, self.dot_error = TD_func_fast(distance_error, self.distance_error, self.dot_error, 1,
                                                               0.05)
            height_error = self.delta_y
            self.height_error, self.dot_height_error = TD_func_fast(height_error, self.height_error,
                                                                    self.dot_height_error, 1, 0.05)
            yaw_error = state_ref[2] - yaw
            rad_yawerror = np.radians(yaw_error)
            logging.info(f"distance_error:{self.distance_error}, height_error={self.height_error}, yaw_error:{yaw_error}")
            # print(self.delta_x, self.delta_y, distance_error, current_yaw, desired_yaw, yaw_error)
            error = np.asarray([distance_error, height_error, rad_yawerror])
            error_dot = np.asarray([self.dot_error, self.dot_height_error, 0.])
            s_errdot = np.sign(error_dot)
            logging.info(f"arctan([distance_error, height_error, rad_yawerror]):{error}")
            # print(arctan([distance_error, height_error, rad_yawerror]),'*************')
            s_error = pow((1 + pow(error, 2)), 5 / 3.) * np.arctan(error)
            s_error_dot = 1.0 / 2 * pow(abs(error_dot), 5 / 3.) * s_errdot
            s = s_error_dot + s_error
            k_s = 0.25 * s
            sign_s = np.sign(s)
            if abs(s[0]) <= 3:
                sign_s[0] = 1 / 3. * s[0]
            if abs(s[1]) <= 5:
                sign_s[1] = 1 / 5. * s[1]
            # eta_s=(0.5+pow(abs(s), 0.5))*sign_s
            eta_s = (self.f_eta + pow(abs(s), 0.5)) * sign_s
            
            # System input
            u_wai = 1.2 * s_errdot * pow(abs(error_dot), 1 / 3.) * (1 + 10 / 3. * error * np.arctan(error)) * pow(
                (1 + pow(error, 2)), 2 / 3.) + 1.2 * k_s + 1.2 * eta_s
            u_sum = [40. * u_wai[0], 75. * u_wai[1], 1.2 * u_wai[2]]
            self.cvg = np.asarray([D_X * vel_x + D * vel_y, 12., 0.])
            t = self.cvg + u_sum
            self.u_1s = t[0] / 30.
            self.u_2s = t[1] * 2.2
            self.u_3s = t[2] * 5. * 1.2
            if abs(yaw_error) <= 13.0:
                t[2] = 0.0
            # print(height_error, self.u_2s)
        return self.u_1s, self.u_2s, self.u_3s

    def integral(self, x_dot, x, t):
        """ Computes the integral o x dt """
        return (x_dot * t) + x

    def iterate(self, vel_x, t0):
        l_d = -47 * 10.
        # self.cvgx = self.cvg[0]
        # estimate disturbance based on following function
        self.track_dot = (l_d * abs(self.track - vel_x) ** 2.0 / 3 * np.sign(self.track - vel_x)) / 40.0 \
                         + (self.d_est + np.asarray(t0 - self.cvg[0]))
        if abs(self.track_dot) >= 1.5:
            self.track_dot = np.sign(self.track_dot)
        self.track = self.integral(self.track_dot, self.track, 0.1)
        # estimate velocity of UBVMS
        self.vel_track_dot = (l_d * abs(self.vel_track - vel_x) ** 2.0 / 3 * np.sign(self.vel_track - vel_x) \
                              + self.d_est + np.asarray(t0 - self.cvg[0])) / 40.0
        self.vel_track = self.integral(self.vel_track_dot, self.vel_track, 0.2)
        v_0 = (l_d * abs(self.track - vel_x) ** 2.0 / 3 * np.sign(self.track - vel_x)) / 40.0 + self.d_est
        self.d_est_dot = -25.0 * (abs(self.d_est - v_0)) ** 1. / 2 * np.sign(self.d_est - v_0) + self.d_est_ddot_int
        self.d_est = self.integral(self.d_est_dot, self.d_est, 0.15)
        self.d_est_ddot = -1.3 * np.sign(self.d_est_ddot_int - self.d_est_dot)
        self.d_est_ddot_int = self.integral(self.d_est_ddot, self.d_est_ddot_int, 0.1)
        # print(self.d_est - self.cvg[0] + t0, self.vel_track, self.d_est)
        if abs(self.cvg[0] + t0) < 10.:
            delta_time = rospy.Time.now().to_sec() - self.last_time
            if delta_time > 20.:
                self.d_time = 0.0
            else:
                self.d_time = self.d_time + delta_time
            with open('d_est.txt', 'a') as DRec:
                DRec.write(str('%.2f' % self.d_time) + ' ')
                DRec.write(str('%.2f' % self.d_est) + ' ')
                DRec.write(str('%.2f' % self.vel_track) + ' ')
                DRec.write(str('%.2f' % (self.d_est - self.cvg[0] + t0)) + '\n')
            self.last_time = rospy.Time.now().to_sec()  
