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
Created on Thu Aug 12 2021
@author: L
'''
class testSM(Behavior):
	'''
	test_for_dual_arm
	'''


	def __init__(self):
		super(testSM, self).__init__()
		self.name = 'test'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:89 y:187, x:623 y:180
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:149 y:41
			OperatableStateMachine.add('GetPose',
										GetPoseState(robot_name=self.robot_name),
										transitions={'done': 'move', 'finish': 'finished'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:432 y:36
			OperatableStateMachine.add('move',
										RobotMoveState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'GetPose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
