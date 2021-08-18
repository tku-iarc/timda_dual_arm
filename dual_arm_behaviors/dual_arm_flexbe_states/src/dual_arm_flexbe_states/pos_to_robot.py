#!/usr/bin/env python

import rospy
import tf
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller

from std_msgs.msg import Bool
from robotis_controller_msgs.msg import StatusMsg
from math import pi, radians
from enum import IntEnum
from hand_eye import eye2base, eye2baseRequest


'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

_POS = (0, 0, 0)
_ORI = (0, 0, 0)
_PHI = 0

class Pos2Robot(EventState):
	'''
	Get the position in robot coordinate from position in camera coordinate

	-- robot_name               string          Robots name to move

	># c_pos                    float[3]        pos with meter in camera coordinate
	#> pos						float[3]        pos with meter in robot coordinate

	<= done 									Robot move done.
	<= failed 									Robot move failed.
	'''


	def __init__(self, robot_name):
		'''
		Constructor
		'''
		super(Pos2Robot, self).__init__(outcomes=['done', 'failed'], input_keys=['c_pos'],
		                                   output_keys=['pos'])

		self.robot_name = robot_name
		self.pos2robot_service = self.robot_name +'/eye2base'
		self.pos2robot_client = ProxyServiceCaller({self.pos2robot_service: eye2base})
	
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if len(userdata.pix_and_depth) != 3:
			rospy.logerr("data illegal")
			return 'failed'
		req = eye2baseRequest()
		req.ini_pose = userdata.c_pos
		try:
			res = self.pos2robot_client.call(self.pos2robot_service, req)
		except rospy.ServiceException as e:
			rospy.logerr("pos2robot_service call failed: %s" % e)
			return 'failed'
		userdata.pos = res.pos
		return 'done'

	def on_enter(self, userdata):
		if not self.pos2robot_client.is_available(self.pos2robot_service):
			rospy.logerr("pos2robot_service is not available!")
			return 'failed'

	def stop(self):
		pass

