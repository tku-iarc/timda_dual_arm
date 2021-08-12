#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dual_arm_flexbe_states.init_robot import InitRobotState
from dual_arm_flexbe_states.robotiq_2f_gripper_state import Robotiq2FGripperState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 12 2021
@author: Luis
'''
class gripper_testSM(Behavior):
	'''
	test robotiq_2f_gripper
	'''


	def __init__(self):
		super(gripper_testSM, self).__init__()
		self.name = 'gripper_test'

		# parameters of this behavior
		self.add_parameter('robot_name', 'left_arm')
		self.add_parameter('en_sim', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:462 y:152, x:515 y:359
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:234 y:63
			OperatableStateMachine.add('gripper_reset',
										Robotiq2FGripperState(robot_name=self.robot_name, en_sim=self.en_sim, gripper_cmd='reset'),
										transitions={'done': 'gripper_active'},
										autonomy={'done': Autonomy.Off})

			# x:226 y:344
			OperatableStateMachine.add('gripper_open',
										Robotiq2FGripperState(robot_name=self.robot_name, en_sim=self.en_sim, gripper_cmd='open'),
										transitions={'done': 'init_robot'},
										autonomy={'done': Autonomy.Off})

			# x:234 y:254
			OperatableStateMachine.add('gripper_position',
										Robotiq2FGripperState(robot_name=self.robot_name, en_sim=self.en_sim, gripper_cmd=58),
										transitions={'done': 'gripper_open'},
										autonomy={'done': Autonomy.Off})

			# x:224 y:448
			OperatableStateMachine.add('init_robot',
										InitRobotState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:234 y:153
			OperatableStateMachine.add('gripper_active',
										Robotiq2FGripperState(robot_name=self.robot_name, en_sim=self.en_sim, gripper_cmd='active'),
										transitions={'done': 'gripper_position'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
