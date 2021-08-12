#!/usr/bin/env python

from flexbe_core import EventState, Logger
from arm_control import Command

right_c_pose = [[[-0.18, -0.15, -0.4700],  [45.000, 0.000, 0.000], 0],
                [[0.40, -0.3263, -0.5500], [45.000, 0.000, 0.000], 0],
                [[0.50, -0.2463, -0.700],  [45.000, 0.000, 0.000], 0],
                [[0.35, -0.2463, -0.700],  [45.000, 0.000, 0.000], 0],
                [[0.50, -0.0463, -0.700],  [45.000, 0.000, 0.000], 0],
                [[0.50, -0.2463, -0.700],  [45.000, 0.000, 0.000], 0],
                [[0.35, -0.3453, -0.700],  [45.000, 0.000, 0.000], 0]]

c_pose = {'left_arm' : [[[0.38,  0.21, 0.15],  [0.0, 65, 0.0]],
                        [[0.38,  0.2, 0.15],   [0.0, 65, 0.0]],
                        [[0.38,  0.19, 0.15],  [0.0, 65, 0.0]],
                        [[0.38,  0.21, 0.15],  [0.0, 65, 0.0]],
                        [[0.38,  0.2, 0.15],   [0.0, 65, 0.0]],
                        [[0.38,  0.19, 0.15],  [0.0, 65, 0.0]]],
          'right_arm': right_c_pose,}

class GetWipePose(EventState):
	"""
	Publishes a pose from userdata so that it can be execute.

	-- robot_name              string          Robots name to move

	#> robot_cmd               command(dict)   See arm_task.py

	<= done									   Pose has been published.
	<= finish								   Task finished

	"""
	
	def __init__(self, robot_name):
		"""Constructor"""
		super(GetWipePose, self).__init__(outcomes=['done', 'finish'], output_keys=['robot_cmd'])
		self.robot_name = robot_name
		self.move_mode = 'line'
		self.total_pose = len(c_pose[self.robot_name])
		self.pose_indx = 0

	def execute(self, userdata):
		if self.pose_indx == self.total_pose:
			return 'finish'
		else:
			userdata.robot_cmd = Command()
			userdata.robot_cmd['mode'] = 'line'
			userdata.robot_cmd['speed'] = 20
			userdata.robot_cmd['pos'] = c_pose[self.robot_name][self.pose_indx][0]
			userdata.robot_cmd['euler'] = c_pose[self.robot_name][self.pose_indx][1]
			userdata.robot_cmd['phi'] = c_pose[self.robot_name][self.pose_indx][2]
			return 'done'
	
	def on_enter(self, userdata):
		self.pose_indx += 1
			
