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
from dual_arm_flexbe_states.fixed_joint_move import FixedJointMoveState
from dual_arm_flexbe_states.fixed_pose_move import FixedPoseMoveState
from dual_arm_flexbe_states.get_spray_alcohol_pose import GetSprayAlcoholPose
from dual_arm_flexbe_states.init_robot import InitRobotState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 12 2021
@author: Luis
'''
class wipe_deskSM(Behavior):
	'''
	Sample behavior of dual arm wipe desk
	'''


	def __init__(self):
		super(wipe_deskSM, self).__init__()
		self.name = 'wipe_desk'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:627 y:513, x:567 y:701
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:49 y:66
			OperatableStateMachine.add('init_left_arm',
										InitRobotState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'init_right_arm', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:44 y:523
			OperatableStateMachine.add('arrive_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=100, pos=[-0.1236, 0.1282, -0.6000], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'get_spray_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:481 y:408
			OperatableStateMachine.add('back_home',
										FixedJointMoveState(robot_name='left_arm', en_sim=self.en_sim, speed=100, slide_pos=0, joints=[0,0,0,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:331 y:50
			OperatableStateMachine.add('get_spray_pose',
										GetSprayAlcoholPose(robot_name='left_arm'),
										transitions={'done': 'move_robot', 'finish': 'back_home'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:44 y:425
			OperatableStateMachine.add('go_above_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='p2p', speed=100, pos=[-0.1236, 0.1282, -0.6680], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'arrive_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:44 y:263
			OperatableStateMachine.add('init_right_arm',
										InitRobotState(robot_name='right_arm', en_sim=self.en_sim),
										transitions={'done': 'approach_alcohol_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:671 y:46
			OperatableStateMachine.add('move_robot',
										IKMoveState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'get_spray_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:43 y:346
			OperatableStateMachine.add('approach_alcohol_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='p2p', speed=100, pos=[-0.1250, 0.2363, -0.600], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'go_above_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
