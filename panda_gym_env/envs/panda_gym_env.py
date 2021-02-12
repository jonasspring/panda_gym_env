from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import gym
import numpy as np
import sys
import copy
import random
import math
from gym import spaces, error
from .panda_environment import Grasping_Env
from .CONSTANTS import *

from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
import rospy
import roslaunch
import os
import time
import tf

import glob
import random

#from absl import logging
#import gin
#from six.moves import range

#import pybullet

MAX_EPISODE_STEPS = 15

OSS_DATA_ROOT = ''

#@gin.configurable
class PandaGymEnv(gym.Env):

    def __init__(
        self,
        block_random=0.3,
        camera_random=0,
        simple_observations=True,
        continuous=False,
        remove_height_hack=False,
        urdf_list=None,
        render_mode='GUI',
        num_objects=5,
        dv=0.06,
        target=False,
        target_filenames=None,
        non_target_filenames=None,
        num_resets_per_setup=1,
        render_width=128,
        render_height=128,
        downsample_width=64,
        downsample_height=64,
        test=False,
        allow_duplicate_objects=True,
        max_num_training_models=900,
        max_num_test_models=100):
        #self.seed = seed
        #if seed is None:
            #self.seed = random.randint(0, sys.maxsize)

        self._time_step = 1. / 200.

        # Open-source search paths.
        self._urdf_root = OSS_DATA_ROOT
        self._models_dir = os.path.join(self._urdf_root, 'random_urdfs')

        self._action_repeat = 200
        self._env_step = 0
        self._renders = render_mode in ['GUI', 'TCP']
        # Size we render at.
        self._width = render_width
        self._height = render_height
        # Size we downsample to.
        self._downsample_width = downsample_width
        self._downsample_height = downsample_height
        self._target = target
        self._num_objects = num_objects
        self._dv = dv
        self._urdf_list = urdf_list
        if target_filenames:
          target_filenames = [self._get_urdf_path(f) for f in target_filenames]
        if non_target_filenames:
          non_target_filenames = [
              self._get_urdf_path(f) for f in non_target_filenames]
        self._object_filenames = (target_filenames or []) + (
            non_target_filenames or [])
        self._target_filenames = target_filenames or []
        self._block_random = block_random
        self._cam_random = camera_random
        self._simple_obs = simple_observations
        self._continuous = continuous
        self._remove_height_hack = remove_height_hack
        self._resets = 0
        self._num_resets_per_setup = num_resets_per_setup
        self._test = test
        self._allow_duplicate_objects = allow_duplicate_objects
        self._max_num_training_models = max_num_training_models
        self._max_num_test_models = max_num_test_models

        self.port = random.randint(11300, 15000)
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'


        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        parent_panda = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file_panda_bringup, port=self.port)
        parent_panda.start()

        time.sleep(15.0)
        parent_motion = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file_motion_generator, port=self.port)
        parent_motion.start()
        time.sleep(15.0)

        #rospy.init_node("test_gym")
        self.tf_listener = None

        self.env = Grasping_Env(self.port)
        time.sleep(10.0)
        #self.env.init(uuid)
        #time.sleep(10.0)
        self.init()

        self.num_steps = 0
        self._max_steps = 15
        self._attempted_grasp = False
        self._remove_height_hack = remove_height_hack

        self._dv = 0.2

        #self.action_space = spaces.Box(np.array([0.0,-0.5, 0.0]),np.array([0.6,0.5,0.6]),dtype=np.float32)
        #self.observation_space = spaces.Dict({"cartesian_pos": spaces.Box(np.array([-0.8,-0.8, -0.8]),np.array([2,2,2]),dtype=np.float32),
 						# "object_pos": spaces.Box(np.array([-0.8,-0.8, -0.8]),np.array([0.8,0.8,0.8]),dtype=np.float32)})
       # self.observation_space = spaces.Box(np.array([0.0,-0.7, 0.0, 0.0,-0.8, 0.0]),np.array([0.8,0.7,0.4, 0.0,0.8,0.4]),dtype=np.float32)
        #self.observation_space = spaces.Box(np.array([0.0,-0.5, 0.0]),np.array([0.6,0.5,0.6]),dtype=np.float32)

        self.action_space = spaces.Box(low=-1, high=1, shape=(4,))
        if self._remove_height_hack:
          self.action_space = spaces.Box(
              low=-1, high=1, shape=(5,))  # dx, dy, dz, da, close

        self.observation_space = spaces.Box(low=-0.5, high=0.5, shape=(14,))
        #self.reset()

    def init(self):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'
        #if rospy.is_shutdown():
        rospy.init_node("test", anonymous=True)
        self.env.init(self.uuid)
        self.tf_listener = tf.TransformListener()

    def reset(self):
        """
        This function resets the environment and returns the game state.
        """
        print("start reset")
        self.num_steps = 0
        self._attempted_grasp = False
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(11311) + '/'
        self.env.move_to_start()
        print("start reset1")
        self.env.move()
        print("before remove")
        #self.env.block.removeModel(self.env.block.name)
        self.env.block.removeObject(self.env.block.name)
        print("after remove")
        time.sleep(1.0)
        #self.env.block.createModel("tape.ttm", [0.4, 0.2], [0], -0.4)
        self.env.block.createObject([0.05, 0.05, 0.05],np.random.uniform([0.3, -0.2, 0.025],[0.4, 0.2, 0.025]), [0], 0.2)

        return self._get_game_state()

    def render(self, mode='human', close=False):
        """
        This function renders the current game state in the given mode.
        """
        if(False):
            print("Hi")


    def step(self, action):
        """
        This method steps the game forward one step and
        shoots a bubble at the given angle.

        Parameters
        ----------
        action : int
            The action is an angle between 0 and 180 degrees, that
            decides the direction of the bubble.

        Returns
        -------
        ob, reward, episode_over, info : tuple
            ob (object) :
                an environment-specific object representing the
                state of the environment.
            reward (float) :
                amount of reward achieved by the previous action.
            episode_over (bool) :
                whether it's time to reset the environment again.
            info (dict) :
                diagnostic information useful for debugging.
        """
        dv = self._dv
        dx = dv * action[0]
        dy = dv * action[1]
        if self._remove_height_hack:
            dz = dv * action[2]
            da = 0.25 * action[3]
        else:
            dz = -dv
            da = 0.25 * action[2]

        state = self._get_game_state()


        print("Action", action)
        print("state: ", state[0])

        self.env.goto_cart_pos([state[0]+dx, state[1]+dy, state[2]+dz], [1,0,0,0],"/panda_1_link0")
        time.sleep(2.0)

        state = self._get_game_state()
        if(state[2] - state[9] < 0.05):
            self.env.grasp()
            self._attempted_grasp = True
            time.sleep(1.0)
            self.env.goto_cart_pos([state[0], state[1], state[2]+0.4], [1,0,0,0],"/panda_1_link0")
            time.sleep(2.0)


        self.num_steps = self.num_steps + 1

        done = self._attempted_grasp or self.num_steps >= self._max_steps
        reward = self._reward()
        #state = self._get_game_state()

        return state, reward, done, {}

    def _reward(self):
        reward = -0.005
        self._grasp_success = 0
        state = self._get_game_state()

        if (state[9] > 0.2):
            self._grasp_success = 1
            reward = 1
        return reward


    def _get_game_state(self):
        """
        This function returns the current game state.
        len(self.colors) means None
        """
        state = {}
        current_pose = [0,0,0]
        rot = [0,0,0,1]
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/panda_1_link0','/panda_1_EE', rospy.Time(0))
            current_pose = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")
        block_pos = [0,0,0]
        block_rot = [0,0,0,1]
        try:
            (block_pos,block_rot) = self.tf_listener.lookupTransform('/panda_1_link0',self.env.block.name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")
        state = np.concatenate((current_pose, rot, block_pos, block_rot))
        #state = [self.env.block.position[0], self.env.block.position[1], 0.2]

        return state
