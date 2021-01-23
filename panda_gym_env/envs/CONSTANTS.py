#!/usr/bin/env python

from panda_std.msg import Float64ArrayStamped, Float64Array
import numpy as np
import roslaunch

GRASP_DIST_TRESHOLD = 0.01#0.001
COLLISION_DIST = 0.1655

POSITION_COLLISION_TRESHOLD = 2e-2#5e-3
ORIENTATION_COLLISION_TRESHOLD = 2e-1#5e-2


SIZE = [0.05, 0.05, 0.2]

POS_MIN, POS_MAX = [0.4, 0.1, SIZE[2]/2 ], [0.4, 0.1, SIZE[2]/2]#[0.3, -0.3, 0.1], [0.5, 0.3, 0.1]
ORI_MIN, ORI_MAX = [0,0,0],[0,0,0]

STARTING_POINT = Float64ArrayStamped()
STARTING_POINT.array = Float64Array([0.0, 0.0, 0.0, -3.141/2, 0.0, 3.141/2, 0])

SIGMA_A = 0.75
SIGMA_S = 0.01

W = np.zeros((6,6))

W[0,0] = 50
W[1,1] = 50
W[2,2] = 50
W[3,3] = 2
W[4,4] = 2
W[5,5] = 2

USE_MOVEIT = True

cli_args_panda_bringup = ['/media/storage/Ubuntu/catkin_ws_coppeliaSim/src/panda_simulation_ros/panda_dynamics_simulation_interface/coppeliasim_ros_control/launch/panda_bringup_effort.launch','load_gripper:=true']
roslaunch_args_panda_bringup = cli_args_panda_bringup[1:]

roslaunch_file_panda_bringup = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_panda_bringup)[0], roslaunch_args_panda_bringup)]

cli_args_motion_generator = ['/media/storage/Ubuntu/catkin_ws_coppeliaSim/src/panda_simulation_ros/panda_dynamics_simulation_interface/coppeliasim_ros_control/launch/panda_motion_generator.launch']
roslaunch_args_motion_generator = cli_args_motion_generator[1:]

roslaunch_file_motion_generator = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_motion_generator)[0], roslaunch_args_motion_generator)]


cli_args_moveit = ['/media/storage/Ubuntu/catkin_ws_coppeliaSim/src/panda_moveit_config/launch/demo.launch']
roslaunch_args_moveit = cli_args_moveit[1:]
roslaunch_file_moveit = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_moveit)[0], roslaunch_args_moveit)]
