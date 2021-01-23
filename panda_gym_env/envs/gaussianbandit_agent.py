#!/usr/bin/env python
from os.path import dirname, join, abspath, os

#os.environ["COPPELIASIM_ROOT"] = "/home/jonas/Ubuntu_extern/CoppeliaSim_Edu_V4_1_0_Ubuntu18_04"
#os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH") + ":" + os.environ.get("COPPELIASIM_ROOT")
#print(os.environ.get("LD_LIBRARY_PATH"))
#os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.environ.get("COPPELIASIM_ROOT")
#print(os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH"))


#from pyrep.objects.joint import Joint
import os

import numpy as np
import math
import roslib
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

from rosgraph_msgs.msg import Clock

from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState

import actionlib
from panda_motion_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from panda_std.msg import Float64ArrayStamped, Float64Array
from geometry_msgs.msg import PoseStamped

import franka_gripper.msg

import rospy
from std_msgs.msg import String

import time

import tf

import copy

from geometry_msgs.msg import Twist

from coppeliasim_ros_control.srv import *


from tf.transformations import *
from .CONSTANTS import *


class Agent(object):

    def __init__(self):
        self.position_offset = [0.0, 0.0, 0.0]
        #self.orientation_offset_rpy = [3.141, 0.0, -3.141/4]
        #self.orientation_offset_rpy = [3.141, 0.0, 3.141/4]
        self.orientation_offset_rpy = [3.141, 0.0, 0.0]
        self.output_position = [0.0, 0.0, 0.0]
        self.output_orientation = [1.0, 0.0, 0.0, 0.0]

    def act(self, state, action):
        self.output_position = action.position
        output_orientation_rpy = action.orientation
        self.output_orientation = tf.transformations.quaternion_from_euler(output_orientation_rpy[0], output_orientation_rpy[1], output_orientation_rpy[2])
        output = (self.output_position, self.output_orientation)
        return output

    def learn(self, replay_buffer):
        del replay_buffer
        pass

    def calc_mean(self, x, y, alpha):
        num_rows, num_cols = y.shape
        mean = 0.0

        for i in range(num_rows):
            mean = mean + self.calcKernel(x,y[i,:]) * alpha[i]

        return mean

    def calcKernel(self, x, y):
        tmp_1 = x - y

        kernel = SIGMA_A * SIGMA_A * np.exp(-0.5 * (tmp_1.T).dot(W.dot(tmp_1)))

        return kernel

    def calcKernel2(self, x, y):
        tmp_1 = x - y

        kernel = SIGMA_A * SIGMA_A * np.exp(-(tmp_1.T).dot(W.dot(tmp_1)))

        return kernel

    def calcKernel3(self, x, y):
        tmp_1 = x - y

        kernel = SIGMA_A * SIGMA_A * np.exp(-(tmp_1.T).dot(W.dot(tmp_1)) / 6)

        return kernel

    def calcGrammMatrix(self,y):
        num_rows, num_cols = y.shape
        Gramm_matrix = np.zeros((num_rows, num_rows))
        for i in range(num_rows):
            for j in range(i, num_rows):
                Gramm_matrix[i,j] = self.calcKernel(y[i,:], y[j,:])
                Gramm_matrix[j,i] = self.calcKernel(y[j,:], y[i,:])

        return Gramm_matrix


    def calcAlpha(self, Gramm_matrix, t):
        num_rows, num_cols = Gramm_matrix.shape
        I = np.identity(num_rows)

        alpha = (np.linalg.inv(Gramm_matrix + SIGMA_S*SIGMA_S*I)).dot(t)

        return alpha

    def calcGamma(self, Gramm_matrix, y):
        num_rows, num_cols = Gramm_matrix.shape
        I = np.identity(num_rows)

        gamma = np.linalg.inv(Gramm_matrix + SIGMA_S*SIGMA_S*I)

        for i in range(num_rows):
            for j in range(i, num_rows):
                tmp_2 = y[i,:] - y[j,:]
                tmp_3 = y[j,:] - y[i,:]
                gamma[i,j] = gamma[i,j] * np.exp(-0.25 * (tmp_2.T).dot(W.dot(tmp_2)))
                gamma[j,i] = gamma[j,i] * np.exp(-0.25 * (tmp_3.T).dot(W.dot(tmp_3)))

        return gamma

    def calcSigma(self,x, y, gamma, Gramm_matrix):
        num_rows, num_cols = y.shape

        kernel_vector = np.zeros(num_rows)
        for i in range(num_rows):
            kernel_vector[i] = self.calcKernel(x,y[i,:])

        I = np.identity(num_rows)
        tmp_1 = np.linalg.inv(Gramm_matrix + SIGMA_S*SIGMA_S*I)

        sigma_squared = self.calcKernel(x,x) - (kernel_vector.T).dot(tmp_1.dot(kernel_vector))

        #print("res1:", sigma_squared)

        #print(kernel_vector)

        kernel_xx = self.calcKernel(x,x)

        tmp_3 = 0.0
        for i in range(num_rows):
            for j in range(num_rows):
                tmp_2 =  0.5* (y[i,:] + y[j,:])
                tmp_3 = tmp_3 + self.calcKernel2(x, tmp_2) * gamma[i,j]

        sigma_squared2 = kernel_xx - tmp_3

        #print("res2",lower_bound)
        return math.sqrt(sigma_squared)


    def calcUpdateStep(self, y, xn, x_start, alpha, gamma, Gramm_matrix, max_step):
        num_rows, num_cols = y.shape

        delta_my = np.zeros(6)
        for j in range(num_rows):
            tmp_1 = (y[j,:] - xn)
            tmp_2 = self.calcKernel(xn,y[j,:])
            delta_my = delta_my + (W.dot(tmp_1)).dot(tmp_2) * alpha[j]


        delta_sigma = np.zeros(6)
        delta_sigma_alt = np.zeros(6)
        #print("xn",xn)
        #print(y)
        #print("gamma",gamma)
        #print("res: ",self.calcLowerBound(xn, y, gamma,Gramm_matrix))
        current_xn = copy.deepcopy(xn)
        (upper_bound_my, upper_bound_std) = self.calcUpperBound(xn,y,alpha, Gramm_matrix)

        sigma = self.calcSigma(current_xn, y, gamma, Gramm_matrix)
        for i in range(num_rows):
            for j in range(num_rows):
                tmp_3 = (y[i,:] + y[j,:]) / 2
                delta_sigma = delta_sigma + 2/sigma * gamma[i,j] * (W.dot((tmp_3 - xn)).dot(self.calcKernel2(x_start, tmp_3)))
                delta_sigma_alt = delta_sigma_alt + 2/sigma * gamma[i,j] * (W.dot((tmp_3 - xn)).dot(self.calcKernel2(xn, tmp_3)))

        #(upper_bound_my, upper_bound_std) = self.calcUpperBound(xn,y,alpha, Gramm_matrix)
        #print("delta_my", delta_sigma)
        step = (delta_my + delta_sigma)#/(upper_bound_my[0] + math.sqrt(upper_bound_std)#/sigma)

        #print("________________________________")
        #print("delta_sigma", delta_sigma)
        #print("delta_sigma alt", delta_sigma_alt)
        #print("delta_my",delta_my)
        #print("upper bound my: ", upper_bound_my)
        #print("upper bound std:", upper_bound_std)
        #print("sigma: ", sigma)
        #print("alpgha: ", alpha)

        #print("step: ", delta_my)
        #print(delta_sigma)

        if(abs(np.linalg.norm((step))) > max_step):
            step = step * max_step / abs(np.linalg.norm((step)))



        #print("start step")
        #print(step)
        #print("end step")
        return step

    def calcUpdateStep2(self, y, xn, alpha, gamma, Gramm_matrix, max_step):
        m = 0.01

        num_rows, num_cols = y.shape

        tmp_1 = 0.0
        tmp_2 = 0.0
        for i in range(num_rows):
            tmp_1 = tmp_1 + alpha[i]*(y[i,:] - xn).dot(self.calcKernel(xn, y[i,:]))
            tmp_2 = tmp_2 + abs(alpha[i])*self.calcKernel(xn, y[i,:])

        if(tmp_2 < 1e-10):
            step = np.zeros(6)
        else:
            step = tmp_1 / tmp_2
            
        if(abs(np.linalg.norm((step))) > max_step):
            step = step * max_step / abs(np.linalg.norm((step)))

        #print("end step")
        return step

    def calcLowerBound(self, x, y, gamma, Gramm_matrix):
        SIGMA_A = 0.75

        num_rows, num_cols = y.shape

        kernel_xx = self.calcKernel(x,x)

        tmp_1 = 0.0

        for i in range(num_rows):
            for j in range(num_rows):
                tmp_2 = (x - 0.5* (y[i,:] + y[j,:]))
                tmp_1 = tmp_1 + (SIGMA_A * SIGMA_A * np.exp(- (tmp_2.T).dot(W.dot(tmp_2))) * gamma[i,j])

        lower_bound = kernel_xx - tmp_1

        kernel_vector = np.zeros(num_rows)
        for i in range(num_rows):
            kernel_vector[i] = self.calcKernel(x,y[i,:])


        I = np.identity(num_rows)
        SIGMA_S = 0.01
        tmp_3 = np.linalg.inv(Gramm_matrix + SIGMA_S*SIGMA_S*I)

        sigma2 = kernel_xx - (kernel_vector.T).dot(tmp_3) .dot(kernel_vector)

        print("lower bound: ", lower_bound)
        print("sigma2", sigma2)
        return sigma2

    def calcUpperBound(self, x, y, alpha, Gramm_matrix):
        num_rows, num_cols = y.shape

        tmp_1 = 0.0

        for j in range(num_rows):
            tmp_1 = tmp_1 + self.calcKernel3(x,y[j,:])*alpha[j]
            #print(tmp_1)

        upper_bound_my = tmp_1
        #print(upper_bound_my)
        #######################################################
        I = np.identity(num_rows)
        gamma2 = np.linalg.inv(Gramm_matrix + SIGMA_S*SIGMA_S*I)

        for i in range(num_rows):
            for j in range(i, num_rows):
                tmp_2 = y[i,:] - y[j,:]
                gamma2[i,j] = gamma2[i,j] *  np.exp(-tmp_2.T.dot(W.dot(tmp_2))/12)
                gamma2[j,i] = gamma2[i,j]


        #######################################################

        kernel_xx = self.calcKernel(x,x)
        tmp_3 = 0.0
        for i in range(num_rows):
            for j in range(num_rows):
                tmp_4 = (x - 0.5*(y[i,:] + y[j,:]))
                tmp_3 = tmp_3 + SIGMA_A * SIGMA_A * np.exp(- (tmp_4.T).dot(W.dot(tmp_4))/3) * gamma2[i,j]

        upper_bound_std = kernel_xx - tmp_3

        kernel_vector = np.zeros(num_rows)
        for i in range(num_rows):
            kernel_vector[i] = self.calcKernel(x,y[i,:])


        I = np.identity(num_rows)

        tmp_3 = np.linalg.inv(Gramm_matrix + SIGMA_S*SIGMA_S*I)

        sigma2 = kernel_xx - (kernel_vector.T).dot(tmp_3).dot(kernel_vector)

        return (upper_bound_my, sigma2)
