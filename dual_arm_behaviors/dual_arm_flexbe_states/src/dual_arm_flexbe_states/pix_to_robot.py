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

class Pix2Robot(EventState):
	'''
	Get the position in robot coordinate from pixel and depth in image

	-- robot_name               string          Robots name to move

	># pix_and_depth            float[3]        pixel x, y, and the depth with meter
	#> pos						float[3]        pos with meter in robot coordinate

	<= done 									Robot move done.
	<= failed 									Robot move failed.
	'''


	def __init__(self, robot_name):
		'''
		Constructor
		'''
		super(Pix2Robot, self).__init__(outcomes=['done', 'failed'], input_keys=['pix_and_depth'],
		                                   output_keys=['pos'])

		self.robot_name = robot_name
		self.pix2robot_service = self.robot_name +'/pix2base'
		self.pix2robot_client = ProxyServiceCaller({self.pix2robot_service: eye2base})
	
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if len(userdata.pix_and_depth) != 3:
			rospy.logerr("data illegal")
			return 'failed'
		req = eye2baseRequest()
		req.ini_pose = userdata.pix_and_depth
		try:
			res = self.pix2robot_client.call(self.pix2robot_service, req)
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s" % e)
			return 'failed'
		userdata.pos = res.pos
		return 'done'

	def on_enter(self, userdata):
		if not self.pix2robot_client.is_available(self.pix2robot_service):
			rospy.logerr("pix2robot_service is not available!")
			return 'failed'

	def stop(self):
		pass

