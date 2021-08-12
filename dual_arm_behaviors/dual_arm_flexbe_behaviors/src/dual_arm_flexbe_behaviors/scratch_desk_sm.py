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
from dual_arm_flexbe_states.fixed_pose_move import FixedPoseMoveState
from dual_arm_flexbe_states.get_scratch_pose import GetScratchPose
from dual_arm_flexbe_states.init_robot import InitRobotState
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
		# x:1250 y:407, x:734 y:2
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

			# x:517 y:292
			OperatableStateMachine.add('apporach_return_spot',
										FixedPoseMoveState(robot_name=self.robot_name, en_sim=self.en_sim, mode='line', speed=100, pos=[-0.16, -0.1820, -0.6500], euler=[-42.024, 0.005, 4.498], phi=0),
										transitions={'done': 'return_scratcher', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:269 y:170
			OperatableStateMachine.add('apporach_scratcher',
										FixedPoseMoveState(robot_name=self.robot_name, en_sim=self.en_sim, mode='line', speed=100, pos=[-0.16, -0.2863, -0.65000], euler=[-44.024, 0.005, 4.498], phi=0),
										transitions={'done': 'arrive_scratcher', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:270 y:260
			OperatableStateMachine.add('arrive_scratcher',
										FixedPoseMoveState(robot_name=self.robot_name, en_sim=self.en_sim, mode='line', speed=100, pos=[-0.16, -0.1820, -0.76000], euler=[-44.024, 0.005, 4.498], phi=0),
										transitions={'done': 'get_scratch_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:268 y:355
			OperatableStateMachine.add('get_scratch_pose',
										GetScratchPose(robot_name=self.robot_name),
										transitions={'done': 'scratch_desk', 'finish': 'apporach_return_spot'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:813 y:264
			OperatableStateMachine.add('return_scratcher',
										FixedPoseMoveState(robot_name=self.robot_name, en_sim=self.en_sim, mode='line', speed=100, pos=[-0.16, -0.1820, -0.7600], euler=[-42.024, 0.005, 4.498], phi=0),
										transitions={'done': 'Back_Home', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:517 y:230
			OperatableStateMachine.add('scratch_desk',
										IKMoveState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'get_scratch_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:978 y:71
			OperatableStateMachine.add('Back_Home',
										InitRobotState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
