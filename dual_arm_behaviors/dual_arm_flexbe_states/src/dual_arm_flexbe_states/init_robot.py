#!/usr/bin/env python

import rospy
import tf
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from std_msgs.msg import String, Bool
from robotis_controller_msgs.msg import StatusMsg
from math import pi, radians
from enum import IntEnum
# from manipulator_h_base_module_msgs.msg import IK_Cmd, JointPose
from manipulator_h_base_module_msgs.msg import JointPose


'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

_POS = (0, 0, 0)
_ORI = (0, 0, 0)
_PHI = 0

class Status(IntEnum):
	idle            = 0
	busy            = 1
	emergency_stop  = 2
	ik_fail         = 3
	finish          = 4

class InitRobotState(EventState):
	'''
	Robot set mode and go to initial pose

	-- robot_name               string          Robots name to move
	-- en_sim					bool			Use real robot or Gazebo
	-- speed                    int             default robot move speed

	<= done 									Robot move done.
	<= failed 									Robot move failed.
	'''


	def __init__(self, robot_name, en_sim, speed):#, speed, slide_pos, joints):
		'''
		Constructor
		'''
		super(InitRobotState, self).__init__(outcomes=['done', 'failed'])

		self.robot_name = robot_name
		self.en_sim = en_sim
		self.speed = speed
		self.slide_pos = 0 #slide_pos
		self.joints = [0, -20, 0, 40, 0, -20, 0] #joints
		self.status = Status.idle
		self.__set_pubSub()

	def __set_pubSub(self):
		print ("[Arm] name space : " + str(self.robot_name))
		self.set_mode_topic = str(self.robot_name) + '/set_mode_msg'
		self.__set_mode_pub = ProxyPublisher({
			self.set_mode_topic:
			String})

		self.joint_topic = str(self.robot_name) + '/joint_pose_msg'
		self.__joint_pub = ProxyPublisher({
            self.joint_topic:
            JointPose})

		self.arm_status_topic = str(self.robot_name) + '/status'
		self.__status_sub = ProxySubscriberCached({
		    self.arm_status_topic:
		    StatusMsg})

	def __status_callback(self, msg):
		if 'Start Trajectory' in msg.status_msg:
			self.status = Status.busy
			# rospy.loginfo('Arm Move!')
		elif 'IK Failed' in msg.status_msg:
			self.status = Status.ik_fail
			rospy.logwarn('IK Fail!')
		elif 'Emergency Stop' in msg.status_msg:
			self.status = Status.emergency_stop
			rospy.logwarn('Emergency Stop!')
		elif 'End Trajectory' in msg.status_msg:
			self.status = Status.finish
			rospy.loginfo('End Trajectory!')
		else:
			rospy.logwarn('Unknow Status: ' + msg.status_msg)

	def jointMove(self, speed, slide_pos = 0,cmd=[0, 0, 0, 0, 0, 0, 0]):
		"""Publish msg of joint cmd (rad) to manager node."""
		name  = list()
		value = list()

		for i, val in enumerate(cmd):
			name.append('joint{}'.format(i+1))
			value.append(radians(val))

		self.__joint_pub.publish(self.joint_topic, JointPose(name, value, slide_pos, speed))
	
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self.__status_sub.has_msg(self.arm_status_topic):
			msg = self.__status_sub.get_last_msg(self.arm_status_topic)
			self.__status_callback(msg)

		if self.status == Status.finish:
			return 'done'
		elif self.status == Status.ik_fail or self.status == Status.emergency_stop:
			return 'failed'

	def on_enter(self, userdata):
		self.__set_mode_pub.publish(self.set_mode_topic, String('set'))
		self.status = Status.busy
		self.__status_sub.remove_last_msg(self.arm_status_topic)
		self.jointMove(self.speed, self.slide_pos, self.joints)
		
	def stop(self):
		pass

	def on_stop(self):
		clear_pub = ProxyPublisher({
            str(self.robot_name) + '/clear_cmd':
            Bool})
		clear_pub.publish(str(self.robot_name) + '/clear_cmd', True)

	def on_pause(self):
		wait_pub = ProxyPublisher({
            str(self.robot_name) + '/wait':
            Bool})
		wait_pub.publish(str(self.robot_name) + '/wait', True)

	def on_resume(self, userdata):
		wait_pub = ProxyPublisher({
            str(self.robot_name) + '/wait':
            Bool})
		wait_pub.publish(str(self.robot_name) + '/wait', False)
		self.execute(userdata)
