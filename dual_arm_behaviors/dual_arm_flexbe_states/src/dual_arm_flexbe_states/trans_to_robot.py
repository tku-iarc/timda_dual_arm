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

class Trans2Robot(EventState):
	'''
	Get the pose in robot coordinate from pose in camera coordinate

	-- robot_name               string          Robots name to move

	># c_trans                  float[16]       transformation matrix in camera coordinate with shape -1
	#> pos						float[3]		pos with meter in robot coordinate
	#> euler					float[3]        euler with degrees in robot coordinate
	#> trans					float[16]       transformation matrix in robot coordinate

	<= done 									Robot move done.
	<= failed 									Robot move failed.
	'''


	def __init__(self, robot_name):
		'''
		Constructor
		'''
		super(Trans2Robot, self).__init__(outcomes=['done', 'failed'], input_keys=['c_trans'],
		                                   output_keys=['trans', 'pos', 'euler'])

		self.robot_name = robot_name
		self.trans2robot_service = self.robot_name +'/eye_trans2base'
		self.trans2robot_client = ProxyServiceCaller({self.trans2robot_service: eye2base})
	
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if len(userdata.pix_and_depth) != 16:
			rospy.logerr("data illegal")
			return 'failed'
		req = eye2baseRequest()
		req.ini_pose = userdata.c_trans
		try:
			res = self.trans2robot_client.call(self.trans2robot_service, req)
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s" % e)
			return 'failed'
		userdata.trans = res.trans
		userdata.pos = res.pos
		userdata.euler = res.euler
		return 'done'

	def on_enter(self, userdata):
		if not self.trans2robot_client.is_available(self.trans2robot_service):
			rospy.logerr("trans2robot_service is not available!")
			return 'failed'

	def stop(self):
		pass

