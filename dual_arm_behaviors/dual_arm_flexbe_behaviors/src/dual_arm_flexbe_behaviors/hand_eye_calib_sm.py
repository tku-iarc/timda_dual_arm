#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dual_arm_flexbe_states.IK_move import IKMoveState
from dual_arm_flexbe_states.calibration_state import ComputeCalibrationState
from dual_arm_flexbe_states.capture_charuco import CaptureCharUcoState
from dual_arm_flexbe_states.fixed_joint_move import FixedJointMoveState
from dual_arm_flexbe_states.get_pose import GetPoseState
from dual_arm_flexbe_states.init_robot import InitRobotState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 10 2021
@author: Andy
'''
class HandEyeCalibSM(Behavior):
	'''
	Use ChArUco to calibrate hand-eye transform
	'''


	def __init__(self):
		super(HandEyeCalibSM, self).__init__()
		self.name = 'Hand Eye Calib'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', True)
		self.add_parameter('default_speed', 30)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:790 y:360, x:699 y:202
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:32 y:153
			OperatableStateMachine.add('init_robot',
										InitRobotState(robot_name=self.robot_name, en_sim=self.en_sim, speed=self.default_speed),
										transitions={'done': 'get_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:716 y:115
			OperatableStateMachine.add('capture aruco',
										CaptureCharUcoState(robot_name=self.robot_name),
										transitions={'done': 'get_pose', 'fail': 'failed'},
										autonomy={'done': Autonomy.Off, 'fail': Autonomy.Off})

			# x:498 y:248
			OperatableStateMachine.add('compute calibration',
										ComputeCalibrationState(robot_name=self.robot_name),
										transitions={'done': 'back_home', 'fail': 'failed'},
										autonomy={'done': Autonomy.Off, 'fail': Autonomy.Off})

			# x:499 y:34
			OperatableStateMachine.add('get_pose',
										GetPoseState(robot_name=self.robot_name),
										transitions={'done': 'move_robot', 'finish': 'compute calibration'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:959 y:36
			OperatableStateMachine.add('move_robot',
										IKMoveState(robot_name=self.robot_name, en_sim=self.en_sim, speed=self.default_speed),
										transitions={'done': 'capture aruco', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:496 y:359
			OperatableStateMachine.add('back_home',
										FixedJointMoveState(robot_name=self.robot_name, en_sim=self.en_sim, speed=self.default_speed, slide_pos=0, joints=[0,0,0,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
