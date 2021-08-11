#!/usr/bin/env python

import rospy
import tf
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from std_msgs.msg import Bool
from robotis_controller_msgs.msg import StatusMsg
from math import pi, radians
from enum import IntEnum
# from manipulator_h_base_module_msgs.msg import IK_Cmd, JointPose
from manipulator_h_base_module_msgs.msg import P2PPose, KinematicsPose



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

class IKMoveState(EventState):
	'''
	Input the pose and move robot with p2p or line mode

	-- robot_name               string          Robots name to move
	-- en_sim					bool			Use real robot or Gazebo

	># robot_cmd                command(dict)   See arm_task.py

	<= done 									Robot move done.
	<= failed 									Robot move failed.
	'''


	def __init__(self, robot_name, en_sim):
		'''
		Constructor
		'''
		super(IKMoveState, self).__init__(outcomes=['done', 'failed'],
											input_keys=['robot_cmd'])

		self.robot_name = robot_name
		self.en_sim = en_sim
		self.suction_angle = 0
		self.status = Status.idle
		self.__set_pubSub()

	def __set_pubSub(self):
		print ("[Arm] name space : " + str(self.robot_name))
		self.p2p_topic = str(self.robot_name) + '/p2p_pose_msg'
		self.__p2p_pub = ProxyPublisher({
		    self.p2p_topic:
		    P2PPose})
		
		self.line_topic = str(self.robot_name) + '/kinematics_pose_msg'
		self.__line_pub = ProxyPublisher({
		    self.line_topic:
		    KinematicsPose})

		self.arm_status_topic = str(self.robot_name) + '/status'
		self.__status_sub = ProxySubscriberCached({
		    self.arm_status_topic:
		    StatusMsg})

	def __status_callback(self, msg):
		if 'Start Trajectory' in msg.status_msg:
			self.status = Status.busy
			rospy.loginfo('Arm Move!')
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

	def euler2quaternion(self, euler):
		roll, pitch, yaw = euler
		quaternion = tf.transformations.quaternion_from_euler(-pitch+pi, -yaw, roll-pi, 'ryxz')
		return (quaternion)

	def ikMove(self, mode='line', pos=_POS, euler=_ORI, phi=_PHI, quat=None, speed=None):
		"""Publish msg of ik cmd (deg) to manager node."""
		roll, pitch, yaw = euler
		roll  = radians(roll)
		pitch = radians(pitch)
		yaw   = radians(yaw)

		msg = KinematicsPose()
		msg.name = 'arm'
		msg.pose.position.x = pos[0]
		msg.pose.position.y = pos[1]
		msg.pose.position.z = pos[2]
		if quat is not None:
			msg.pose.orientation.w = quat[0]
			msg.pose.orientation.x = quat[1]
			msg.pose.orientation.y = quat[2]
			msg.pose.orientation.z = quat[3]
		else:
			quat = self.euler2quaternion((roll, pitch, yaw))
			msg.pose.orientation.x = quat[0]
			msg.pose.orientation.y = quat[1]
			msg.pose.orientation.z = quat[2]
			msg.pose.orientation.w = quat[3]
		if speed is not None:
			msg.speed = speed

		msg.phi = radians(phi)

		if mode == 'line':
			self.__line_pub.publish(self.line_topic, msg)
		elif mode == 'p2p':
			self.__p2p_pub.publish(self.p2p_topic, msg)
	
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
		mode = userdata.robot_cmd['mode']
		speed = userdata.robot_cmd['speed']
		pos = userdata.robot_cmd['pos']
		euler = userdata.robot_cmd['euler']
		quat = userdata.robot_cmd['quat']
		phi = userdata.robot_cmd['phi']
		self.status = Status.busy
		self.__status_sub.remove_last_msg(self.arm_status_topic)
		self.ikMove(mode, pos, euler, phi, quat, speed)
		
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
