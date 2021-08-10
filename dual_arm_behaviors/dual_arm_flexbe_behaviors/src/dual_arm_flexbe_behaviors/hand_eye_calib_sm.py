#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dual_arm_flexbe_states.get_pose import GetPoseState
from dual_arm_flexbe_states.robot_move import RobotMoveState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 10 2021
@author: Andy
'''
class hand_eye_calibSM(Behavior):
	'''
	Use ChArUco to calibrate hand-eye transform
	'''


	def __init__(self):
		super(hand_eye_calibSM, self).__init__()
		self.name = 'hand_eye_calib'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:459 y:198
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:94 y:115
			OperatableStateMachine.add('get_pose',
										GetPoseState(robot_name=self.robot_name),
										transitions={'done': 'move_robot', 'finish': 'finished'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:298 y:114
			OperatableStateMachine.add('move_robot',
										RobotMoveState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'get_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
