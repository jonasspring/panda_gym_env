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
import geometry_msgs.msg

import franka_gripper.msg

import rospy
from std_msgs.msg import String

import time

import tf

from geometry_msgs.msg import Twist

from coppeliasim_ros_control.srv import *

#import tensorflow

import copy

import roslaunch
import multiprocessing as mp

from tf.transformations import *

from .CONSTANTS import *
#import gaussianbandit_agent
from .gaussianbandit_agent import Agent

from panda_motion_msgs.srv import SwitchControlMode
from panda_motion_msgs.msg import PandaControlMode

#import moveit_commander
#import moveit_msgs.msg
#from moveit_commander.conversions import pose_to_list


class State(object):
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]



class Object2Pick(object):
    def __init__(self, port):
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]
        self.name = ""
        self.mass = 0.1
        self.existing = False

        self.port = port

    def createObject(self, size, pos, orientation, mass):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'
        if not(self.existing):
            rospy.wait_for_service('/panda/simRosCreateSimpleObject')

            try:
                create_simple_Object = rospy.ServiceProxy('/panda/simRosCreateSimpleObject', createSimpleObject)
                resp = create_simple_Object(0, size, mass, False, True, False, True, False, False, pos, orientation, [0,0,0], 1.0)

                self.position = pos
                self.orientation = orientation
                self.mass = mass
                self.name = resp.name
                self.existing = True

                return resp.name
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            print ("Object already existing, remove old one first!")
            return self.name

    def removeObject(self, name):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'
        print("Waiting")
        rospy.wait_for_service('/panda/simRosRemoveSimpleObject')
        try:
            remove_simple_Object = rospy.ServiceProxy('/panda/simRosRemoveSimpleObject', removeSimpleObject)
            resp = remove_simple_Object(self.name)
            if(resp.successful):
                self.existing = False
            return resp.successful
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def createModel(self, filename, pos, orientation, mass, friction=1.0):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'
        if not(self.existing):
            rospy.wait_for_service('/panda/simRosLoadModel')

            try:
                load_model = rospy.ServiceProxy('/panda/simRosLoadModel', loadModel)
                resp = load_model(filename, pos, orientation, friction, mass)
                print("POS: ", pos)
                print("ORIENTATION", orientation)

                self.position = pos
                self.orientation = orientation
                self.mass = mass
                self.name = resp.name
                self.existing = True

                return resp.name
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            print ("Object already existing, remove old one first!")
            return self.name

    def removeModel(self, name):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'
        rospy.wait_for_service('/panda/simRosRemoveModel')
        try:
            remove_model = rospy.ServiceProxy('/panda/simRosRemoveModel', removeModel)
            resp = remove_model(self.name)
            if(resp.successful):
                self.existing = False
            return resp.successful
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class Grasping_Env(object):
    def __init__(self, port):
        super(Grasping_Env, self).__init__()

        self.port = port

    def init(self, uuid):

        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'

        print("HALLO1")
        print(rospy.is_shutdown())
        self.finished = False
        self.gripper_grasp = actionlib.SimpleActionClient(
            '/panda/franka_gripper/grasp',
            franka_gripper.msg.GraspAction)
        self.gripper_grasp.wait_for_server()
        print("HALLO12")

        self.gripper_move = actionlib.SimpleActionClient(
            '/panda/franka_gripper/move',
            franka_gripper.msg.MoveAction)
        self.gripper_move.wait_for_server()
        self.tf_listener = tf.TransformListener()


        self.block = Object2Pick(self.port)
        print("HALLO13")

        self.name = ""

        self.client = actionlib.SimpleActionClient(('/panda/motion_generator/panda_1_motion'), ExecuteTrajectoryAction)
        #print(data.data[0])
        #print(client.wait_for_server())
        self.client.wait_for_server()



    def goto_cart_pos(self, position, orientation, frame):
        os.environ['ROS_MASTER_URI'] = "http://localhost:"+str(self.port)
        #client = actionlib.SimpleActionClient(('/panda/motion_generator/panda_1_motion'), ExecuteTrajectoryAction)
        #print(data.data[0])
        #print(client.wait_for_server())
        print("start goto cart")
        self.client.wait_for_server()

        self.finished = False

        cart_goal = ExecuteTrajectoryGoal()
        point_1_c = PoseStamped()
        point_1_c.header.frame_id = frame
        point_1_c.header.stamp = rospy.Time.now()
        point_1_c.pose.position.x = position[0]
        point_1_c.pose.position.y = position[1]
        point_1_c.pose.position.z = position[2]
        point_1_c.pose.orientation.x = orientation[0]
        point_1_c.pose.orientation.y = orientation[1]
        point_1_c.pose.orientation.z = orientation[2]
        point_1_c.pose.orientation.w = orientation[3]

        cart_goal.cart_states.append(point_1_c)
        cart_goal.mode.id = cart_goal.mode.CART


        # Sends the goal to the action server.
        self.client.send_goal(cart_goal)

        self.client.wait_for_result()

        self.finished = True


        return self.client.get_result()

    def move_around(self):


        self.client.wait_for_server()
        self.finished = True

        # Creates a goal to send to the action server.
        goal = ExecuteTrajectoryGoal()
        point_1 = Float64ArrayStamped()
        point_1.array.data = [0.0, 0.0, 0.0, -0.2, 0.0, 3.141/2, 3.141/2]
        point_1.header.stamp = rospy.Time.from_sec(2.0)
        goal.joint_states.append(point_1)

        point_2 = Float64ArrayStamped()
        point_2.array.data = [0.0, 0.0, 0.0, -3.141/2, 0.0, 3.141/2, 0]
        point_2.header.stamp = rospy.Time.from_sec(4.0)
        goal.joint_states.append(point_2)

        goal.mode.id = goal.mode.JOINT

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        # Prints out the result of executing the action
        return self.client.get_result()

    def move_to_start(self):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(11311) + '/'
        print("start movetostart")

        self.client.wait_for_server()
        self.finished = True
        print("start movetostart2")

        # Creates a goal to send to the action server.
        goal = ExecuteTrajectoryGoal()
        point_1 = Float64ArrayStamped()
        point_1.array.data = STARTING_POINT.array.data[:]
        point_1.header.stamp = rospy.Time.from_sec(2.0)
        goal.joint_states.append(point_1)

        goal.mode.id = goal.mode.JOINT

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        print("start movetostart3")
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        # Prints out the result of executing the action
        print("start movetostart4")
        return self.client.get_result()

    def grasp(self):
        width = 0.0
        epsilon_inner=1.0
        epsilon_outer=1.0
        speed=0.1
        force=20
        epsilon = franka_gripper.msg.GraspEpsilon(inner=epsilon_inner,outer=epsilon_outer)
        goal = franka_gripper.msg.GraspGoal(width=width,epsilon=epsilon,speed=speed,force=force)
        #rospy.loginfo('Grasping:\n{}'.format(goal))
        self.gripper_grasp.send_goal(goal)
        self.gripper_grasp.wait_for_result()

    def move(self):
        width = 0.08
        speed=0.5
        goal = franka_gripper.msg.MoveGoal(width=width, speed=speed)
        #rospy.loginfo('Moving gripper:\n{}'.format(goal))
        self.gripper_move.send_goal(goal)
        self.gripper_move.wait_for_result()


    def getDistanceAndVel(self, name, duration):
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/panda_1_link8', name, rospy.Time(0))
            if (duration.to_sec() > 0.0):
                (lin, ang) = self.tf_listener.lookupTwist('/panda_1_link8', name, rospy.Time(0), duration)
            else:
                lin = 99.0 * [1,1,1]
                ang = 99.0 * [1,1,1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")
        distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] **2)
        lin_vel = math.sqrt(lin[0] ** 2 + lin[1] ** 2 + lin[2] **2)
        ang_vel = math.sqrt(ang[0] ** 2 + ang[1] ** 2 + ang[2] **2)

        print(distance)

        return (distance, lin_vel, ang_vel)

    def initializePoints(self, starting_points, rewards, random_obj, model_name):
        print("initialize starting points")
        local_state = State()
        local_action = State()
        local_agent = Agent()
        local_state.position = list(random_obj.uniform(POS_MIN, POS_MAX))
        local_state.orientation = list(random_obj.uniform([0,0,-3.14], [0,0,3.14]))
        it = 0
        for row in starting_points:
            #local_name = self.block.createObject(SIZE,local_state.position,local_state.orientation, 0.1)
            local_name = self.block.createModel(model_name +".ttm", [0.4, 0.2], [0], -0.4)
            self.block.name = local_name

            local_action.position = row[0:3]
            local_action.orientation = row[3:6]
            (local_action_output_pos, local_action_output_orien) = local_agent.act(local_state, local_action)
            time.sleep(1.0)

            local_reward = self.step_without_moving(local_state,local_action_output_pos,local_action_output_orien)
            print("local state: ", local_state.position)
            print("action_pos: ", local_action_output_pos)
            print("action_ori: ", local_action_output_orien)

            #self.block.removeObject(local_name)
            self.block.removeModel(local_name)

            rewards[it] = local_reward

            it = it + 1

    def step(self, state, action_position, action_orientation):
        os.environ['ROS_MASTER_URI'] = "http://localhost:"+str(self.port)
        reward = 0.0
        pos = [0,0,0.35]
        orientation = action_orientation[:]

        ### Get transformation to object
        try:
            (trans_before,rot_before) = self.tf_listener.lookupTransform('/panda_1_link0', self.block.name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")

        ### Open Gripper and move over the object
        self.move()
        self.move_to_grasp_pose(action_position, action_orientation)
        time.sleep(2.0)



        ### Get the tf before grasping to check for collision
        position_change = np.array([0,0,0])
        orientation_change = np.array([0,0,0])
        try:
            (trans_after,rot_after) = self.tf_listener.lookupTransform('/panda_1_link0', self.block.name, rospy.Time(0))
            position_change = np.array(trans_after) - np.array(trans_before)
            orientation_change = np.array(tf.transformations.euler_from_quaternion(rot_after)) - np.array(tf.transformations.euler_from_quaternion(rot_before))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")

        if (abs(np.linalg.norm((position_change))) > POSITION_COLLISION_TRESHOLD or abs(np.linalg.norm((orientation_change))) > ORIENTATION_COLLISION_TRESHOLD):
            print("Collision with object, reward = 0.0")
            print(abs(np.linalg.norm((position_change))))
            print(abs(np.linalg.norm((orientation_change))))
            reward = 0.0
        else:
            print(abs(np.linalg.norm((position_change))))
            print(abs(np.linalg.norm((orientation_change))))

            self.grasp()
            time.sleep(2.0)


            time1 = rospy.Time.now()
            (distance1, lin_vel1, ang_vel1) = self.getDistanceAndVel(self.block.name, rospy.Duration(0.1))
            pos[2] = pos[2] + 0.3
            result = self.goto_cart_pos(pos, orientation, self.block.name)

            time2 = rospy.Time.now()
            (distance2, lin_vel2, ang_vel2) = self.getDistanceAndVel(self.block.name, rospy.Duration((time2 - time1).to_sec()))
            time.sleep(2.0)

            #if (distance1 < COLLISION_DIST):
                #reward = reward - 5

            distance_diff = distance2 - distance1
            if ((abs(distance_diff) < GRASP_DIST_TRESHOLD)  ):
                print("Grasp successful, change in distance: ", distance_diff)
                reward = reward + 1.0
                time.sleep(1.0)
                time3 = rospy.Time.now()
                self.move_around()

                time4 = rospy.Time.now()
                (distance3, lin_vel3, ang_vel3) = self.getDistanceAndVel(self.block.name, rospy.Duration((time4 - time3).to_sec()))

                reward = reward - abs(ang_vel3) * 5 - abs(distance3-distance2)

                if(reward < 0.0):
                    reward = 0.2

                #print("Velocity during movement: " )
                #print("linear: ", lin_vel3)
                #print("angular: ", ang_vel3)
                print("change in distance: ", distance3-distance2)
                print("distance", distance1)
            else:
                print("Grasp failed, change in distance: ", distance_diff)
                reward = 0.0

            if (place_object):
                self.goto_cart_pos(pickup_position, pickup_orientation, 'panda_1_link0')
                time.sleep(1.0)
                self.move()

        time.sleep(1.0)

        self.move_to_start()

        return reward

    def step_without_moving(self, state, action_position, action_orientation, place_object=False):
        os.environ['ROS_MASTER_URI'] = "http://localhost:"+str(self.port)
        reward = 0.0
        pos = [0,0,0.35]
        orientation = action_orientation[:]

        ### Get transformation to object
        try:
            (trans_before,rot_before) = self.tf_listener.lookupTransform('/panda_1_link0', self.block.name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")

        ### Open Gripper and move over the object
        self.move()
        self.move_to_grasp_pose(action_position, action_orientation)
        time.sleep(2.0)



        ### Get the tf before grasping to check for collision
        position_change = np.array([0,0,0])
        orientation_change = np.array([0,0,0])
        try:
            (trans_after,rot_after) = self.tf_listener.lookupTransform('/panda_1_link0', self.block.name, rospy.Time(0))
            position_change = np.array(trans_after) - np.array(trans_before)
            orientation_change = np.array(tf.transformations.euler_from_quaternion(rot_after)) - np.array(tf.transformations.euler_from_quaternion(rot_before))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")


        if (abs(np.linalg.norm((position_change))) > POSITION_COLLISION_TRESHOLD or abs(np.linalg.norm((orientation_change))) > ORIENTATION_COLLISION_TRESHOLD):
            print("Collision with object, reward = 0.0")
            print(abs(np.linalg.norm((position_change))))
            print(abs(np.linalg.norm((orientation_change))))
            reward = 0.0
        else:
            #safe position
            try:
                (pickup_position,pickup_orientation) = self.tf_listener.lookupTransform('/panda_1_link0', '/panda_1_EE', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("exception")

            self.grasp()
            time.sleep(2.0)

            time1 = rospy.Time.now()
            (distance1, lin_vel1, ang_vel1) = self.getDistanceAndVel(self.block.name, rospy.Duration(0.1))
            try:
                (trans_1,rot_1) = self.tf_listener.lookupTransform('/panda_1_EE', self.block.name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("exception")

            pos[2] = pos[2] + 0.3
            self.move_to_start()

            time2 = rospy.Time.now()
            (distance2, lin_vel2, ang_vel2) = self.getDistanceAndVel(self.block.name, rospy.Duration((time2 - time1).to_sec()))
            try:
                (trans_2,rot_2) = self.tf_listener.lookupTransform('/panda_1_EE', self.block.name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("exception")
            time.sleep(2.0)

            distance_diff = distance2 - distance1
            orientation_diff = np.array(tf.transformations.euler_from_quaternion(rot_2)) - np.array(tf.transformations.euler_from_quaternion(rot_1))

            if ((abs(distance_diff) < GRASP_DIST_TRESHOLD) and abs(np.linalg.norm(orientation_diff)) < 0.785):
                print("Grasp successful, change in distance: ", distance_diff)
                reward = reward + 1.0

            if (place_object):
                try:
                    self.goto_cart_pos(pickup_position, pickup_orientation, 'panda_1_link0')
                    time.sleep(1.0)
                    self.move()
                except NameError:
                    print("Failed to place object")

        self.move_to_start()

        return reward

    def move_to_grasp_pose(self, action_position, action_orientation):
        pos = [0,0,0.35]
        orientation = action_orientation[:]

        result = self.goto_cart_pos(pos, orientation, self.block.name)
        time.sleep(2.0)

        approaching_pose = [0,0,-0.1,1]
        rot_mat = tf.transformations.quaternion_matrix(orientation)
        approaching_pose = rot_mat.dot(approaching_pose)
        result = self.goto_cart_pos(approaching_pose[0:3] + action_position, orientation, self.block.name)
        time.sleep(4.0)

        print(approaching_pose[0:3] + action_position)
        result = self.goto_cart_pos(action_position, orientation, self.block.name)
        time.sleep(4.0)

