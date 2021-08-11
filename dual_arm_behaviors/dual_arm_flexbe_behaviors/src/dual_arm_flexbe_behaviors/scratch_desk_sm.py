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
from dual_arm_flexbe_states.apporach_scratcher import ApporachScratcher
from dual_arm_flexbe_states.init_robot import InitRobotState
from dual_arm_flexbe_states.robotiq_2f_gripper_control import Robotiq2FGripperControl
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 12 2021
@author: Luis
'''
class scratch_deskSM(Behavior):
	'''
	Sample behavior of dual arm wipe desk
	'''


	def __init__(self):
		super(scratch_deskSM, self).__init__()
		self.name = 'scratch_desk'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:859 y:341, x:678 y:10
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:89 y:114
			OperatableStateMachine.add('initRobot',
										InitRobotState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'apporach_scratcher', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:489 y:175
			OperatableStateMachine.add('Grab_scratcher',
										Robotiq2FGripperControl(robot_name=self.robor_name, en_sim=self.en_sim, gripper_cmd=58),
										transitions={'done': 'Back_Home'},
										autonomy={'done': Autonomy.Off})

			# x:486 y:83
			OperatableStateMachine.add('Move_Robot',
										IKMoveState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'apporach_scratcher', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:280 y:172
			OperatableStateMachine.add('apporach_scratcher',
										ApporachScratcher(robot_name=self.robot_name),
										transitions={'done': 'Move_Robot', 'finish': 'Grab_scratcher'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:695 y:172
			OperatableStateMachine.add('Back_Home',
										InitRobotState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
