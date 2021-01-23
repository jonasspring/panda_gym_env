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

MAX_EPISODE_STEPS = 5

class PandaGymEnv(gym.Env):

    def __init__(self, seed=None):
        #self.seed = seed
        #if seed is None:
            #self.seed = random.randint(0, sys.maxsize)

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
      
        self.num_steps = 0


        self.action_space = spaces.Box(np.array([0.0,-0.7, 0.0]),np.array([0.8,0.7,0.8]),dtype=np.float32)
        #self.observation_space = spaces.Dict({"cartesian_pos": spaces.Box(np.array([-0.8,-0.8, -0.8]),np.array([2,2,2]),dtype=np.float32),
 						# "object_pos": spaces.Box(np.array([-0.8,-0.8, -0.8]),np.array([0.8,0.8,0.8]),dtype=np.float32)})
        self.observation_space = spaces.Box(np.array([0.0,-0.7, 0.0, 0.0,-0.8, 0.0]),np.array([0.8,0.7,0.0, 0.0,0.8,0.0]),dtype=np.float32)
        #self.reset()

    def init(self):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(self.port) + '/'
        #if rospy.is_shutdown():
        #rospy.init_node("test", anonymous=True)
        self.env.init(self.uuid)
        self.tf_listener = tf.TransformListener()

    def reset(self):
        """
        This function resets the environment and returns the game state.
        """
        print("start reset")
        self.num_steps = 0
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(11311) + '/'
        self.env.move_to_start()
        print("start reset1")
        self.env.move()
        print("before remove")
        self.env.block.removeModel(self.env.block.name)
        print("after remove")
        time.sleep(1.0)
        self.env.block.createModel("tape.ttm", [0.4, 0.2], [0], -0.4)

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
        print("Action", action.tolist())
        self.env.goto_cart_pos(action.tolist(), [1,0,0,0],"/panda_1_link0")

        state = self._get_game_state()

        distance2object = np.linalg.norm(np.array(state[0:3]) - np.array((state[3:6])))
        reward = -distance2object

        done = False
        if (distance2object < 0.05):
            done = True

        self.num_steps = self.num_steps + 1
  
        if (self.num_steps > (MAX_EPISODE_STEPS - 1)):
            done = True

        return state, reward, done, {}

    def _get_reward(self, bubbles, result):
        """
        This function calculates the reward.
        """
        rewards = {"hit": 1,
                   "miss": -1,
                   "pop": 10,
                   "win": 200,
                   "lost": -200}

        # Return win or loose
        if len(result) > 0:
            return rewards[result]
        # Nothing hit
        elif bubbles == 1:
            return rewards["miss"]
        # Hit at least one bubble of the same color
        elif bubbles < 3:
            return rewards["hit"]
        # Hit enough bubbles to delete
        else:
            return bubbles * rewards["pop"]

    def _update_color_list(self):
        """
        This function updates the color list
        based on what colors are still in the game.
        """
        remaining_colors = set()

        for row in range(self.array_height):
            for column in range(self.array_width):
                if self.board[row][column].color is not None:
                    remaining_colors.add(self.board[row][column].color)

        return list(remaining_colors)

    def _make_blank_board(self):
        """
        This function creates an empty array with the global size and returns it.
        """
        array = []
        for _ in range(self.array_height):
            column = []
            for _ in range(self.array_width):
                column.append(Bubble())
            array.append(column)
        return array

    def _set_bubble_positions(self):
        """
        This function sets the positional arguments of the bubbles
        in the game board.
        """
        # set the x-values for every bubble
        for row in range(self.array_height):
            for column in range(self.array_width):
                self.board[row][column].center_x = (
                    self.bubble_radius * 2 + self.spacing) * column + self.bubble_radius + self.spacing

        # adjust the x-value in every second row
        for row in range(1, self.array_height, 2):
            for column in range(self.array_width):
                self.board[row][column].center_x += self.bubble_radius + \
                    0.5 * self.spacing

        # calculate the row distance on the y-axis based on the spacing
        y_distance = abs(math.sqrt((2 * self.bubble_radius + self.spacing)
                                   ** 2 - (self.bubble_radius + 0.5 * self.spacing)**2))

        # set the y-values for every bubble
        for row in range(self.array_height):
            for column in range(self.array_width):
                self.board[row][column].center_y = self.spacing + \
                    self.bubble_radius + row * y_distance

    def _fill_board(self):
        """
        This function fills the game board's initial
        lines with bubbles.
        """
        for row in range(self.initial_lines):
            for column in range(self.array_width):
                random.seed(self.seed+column)
                random.shuffle(self.color_list)
                self.board[row][column].color = self.color_list[0]

    def _move_next_bubble(self, angle):
        """
        Moves the next_bubble forward at the global speed.
        """
        # Calculate the movement in x- and y-direction
        if angle == 90:
            xmove = 0
            ymove = self.speed * -1
        elif angle < 90:
            xmove = math.cos(math.radians(angle)) * self.speed * -1
            ymove = math.sin(math.radians(angle)) * self.speed * -1
        else:
            xmove = math.cos(math.radians(180 - angle)) * self.speed
            ymove = math.sin(math.radians(180 - angle)) * self.speed * -1
        self.next_bubble.center_x += xmove
        self.next_bubble.center_y += ymove

        # collision with left wall
        if self.next_bubble.center_x - self.bubble_radius <= self.spacing:
            angle = 180 - angle
        # collision with right wall
        elif self.next_bubble.center_x + self.bubble_radius >= self.window_width - self.spacing:
            angle = 180 - angle
        return angle

    def _set_next_bubble_position(self):
        """
        Sets the next_bubble to its new position in the game board
        and returns the postion.
        """
        # calculate distances to all empty places
        empty_bubble_list = []
        for row in range(self.array_height):
            for column in range(self.array_width):
                if self.board[row][column].color is None:
                    current = self.board[row][column]
                    distance = self._bubble_center_distance(
                        self.next_bubble, current)
                    empty_bubble_list.append(
                        (distance, current.center_x, current.center_y, row, column))

        # select place with smallest distance to next_bubble
        minimum = min(empty_bubble_list, key=lambda t: t[0])

        # set the next_bubble to its new loaction
        self.next_bubble.center_x = minimum[1]
        self.next_bubble.center_y = minimum[2]
        self.board[minimum[3]][minimum[4]] = self.next_bubble

        return minimum[3], minimum[4]

    def _delete_bubbles(self, bubbles):
        """
        Deletes all given bubbles (tuples with x and y coordinate).
        """
        for bubble in bubbles:
            self.board[bubble[0]][bubble[1]].color = None

    def _delete_floaters(self):
        """
        Deletes all floating bubbles.
        """
        # All bubbles to keep
        connected_to_top = set()
        # All bubbles to examine
        pending = set()

        # Add all bubbles in the first row to pending
        for column in range(self.array_width):
            if self.board[0][column].color is not None:
                pending.add((0, column))

        # Calculate all bubbles that are connected to the top
        while len(pending) > 0:
            current = pending.pop()
            connected_to_top.add(current)
            for bubble in self._get_neighbors(
                    current[0], current[1], check_color=False):
                if bubble not in connected_to_top and self.board[bubble[0]
                                                                 ][bubble[1]].color is not None:
                    pending.add(bubble)

        # Get a set of all bubbles
        all_bubbles = set()
        for row in range(self.array_height):
            for column in range(self.array_width):
                if self.board[row][column].color is not None:
                    all_bubbles.add((row, column))

        # Delete bubbles
        to_be_deleted = all_bubbles.difference(connected_to_top)
        self._delete_bubbles(to_be_deleted)

    def _get_neighborhood(self, row, column):
        """
        Returns a list of all coherent bubbles of the same color.
        """
        # all visited bubbles in the neighborhood
        neighborhood = set()
        # all unvisited bubbles in the neighborhood
        pending = set()
        pending.add((row, column))

        while (len(pending) > 0):
            current = pending.pop()
            neighborhood.add(current)
            for bubble in self._get_neighbors(
                    current[0], current[1], self.board[current[0]][current[1]].color, check_color=True):
                if bubble not in neighborhood:
                    pending.add(bubble)

        return neighborhood

    def _get_neighbors(self, row, column, color=None, check_color=True):
        """
        Returns all direct neighbors of a bubble that are in the given color.
        """
        neighbors = []
        if row % 2 == 0:
            if column + 1 < self.array_width:
                if self.board[row][column +
                                   1].color == color or not check_color:
                    neighbors.append((row, column + 1))  # right
            if column - 1 >= 0:
                if self.board[row][column -
                                   1].color == color or not check_color:
                    neighbors.append((row, column - 1))  # left
            if row - 1 >= 0:
                if self.board[row -
                              1][column].color == color or not check_color:
                    neighbors.append((row - 1, column))  # top right
            if row - 1 >= 0 and column - 1 >= 0:
                if self.board[row - 1][column -
                                       1].color == color or not check_color:
                    neighbors.append((row - 1, column - 1))  # top left
            if row + 1 < self.array_height:
                if self.board[row +
                              1][column].color == color or not check_color:
                    neighbors.append((row + 1, column))  # bottom right
            if row + 1 < self.array_height and column - 1 >= 0:
                if self.board[row + 1][column -
                                       1].color == color or not check_color:
                    neighbors.append((row + 1, column - 1))  # bottom left
        else:
            if column + 1 < self.array_width:
                if self.board[row][column +
                                   1].color == color or not check_color:
                    neighbors.append((row, column + 1))  # right
            if column - 1 >= 0:
                if self.board[row][column -
                                   1].color == color or not check_color:
                    neighbors.append((row, column - 1))  # left
            if row - 1 >= 0:
                if self.board[row -
                              1][column].color == color or not check_color:
                    neighbors.append((row - 1, column))  # top left
            if row - 1 >= 0 and column + 1 < self.array_width:
                if self.board[row - 1][column +
                                       1].color == color or not check_color:
                    neighbors.append((row - 1, column + 1))  # top right
            if row + 1 < self.array_height:
                if self.board[row +
                              1][column].color == color or not check_color:
                    neighbors.append((row + 1, column))  # bottom left
            if row + 1 < self.array_height and column + 1 < self.array_width:
                if self.board[row + 1][column +
                                       1].color == color or not check_color:
                    neighbors.append((row + 1, column + 1))  # bottom right
        return neighbors

    def _is_collided(self):
        """
        This function returns true if next_bubble is collided
        with another bubble or the top of the screen, false otherwise.
        """
        # collsion with top
        if self.next_bubble.center_y - self.bubble_radius <= self.spacing:
            return True
        # collision with another bubble
        for row in range(self.array_height):
            for column in range(self.array_width):
                if self.board[row][column].color is None:
                    continue
                distance = self._bubble_center_distance(
                    self.next_bubble, self.board[row][column]) - self.bubble_radius * 2
                if distance < 0:
                    return True
        return False

    def _bubble_center_distance(self, bubble1, bubble2):
        """
        Calculates the distance between the centers of two given bubbles.
        """
        return math.sqrt((bubble1.center_x - bubble2.center_x)
                         ** 2 + (bubble1.center_y - bubble2.center_y)**2)

    def _get_game_state(self):
        """
        This function returns the current game state.
        len(self.colors) means None
        """
        state = {}
        current_pose = [0,0,0]
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/panda_1_EE', '/panda_1_link0', rospy.Time(0))
            current_pose = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")
        state = [current_pose[0], current_pose[1], current_pose[2], self.env.block.position[0], self.env.block.position[1], 0.1]

        return state

    def _is_over(self):
        """
        Returns a string and a bool in which way the game is
        over or not.
        """
        # check if deadline is reached
        for row in range(self.death_line, self.array_height):
            for column in range(self.array_width):
                if self.board[row][column].color is not None:
                    return "lost", True
        # check if board is blank
        for row in range(self.array_height):
            for column in range(self.array_width):
                if self.board[row][column].color is not None:
                    return "", False
        return "win", True
