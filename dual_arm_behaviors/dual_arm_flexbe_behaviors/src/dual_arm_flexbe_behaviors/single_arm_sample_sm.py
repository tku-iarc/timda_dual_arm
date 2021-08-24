#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from customer_flexbe_states.IK_move import IKMoveState as customer_flexbe_states__IKMoveState
from dual_arm_flexbe_states.fixed_joint_move import FixedJointMoveState
from dual_arm_flexbe_states.fixed_pose_move import FixedPoseMoveState
from dual_arm_flexbe_states.get_pose import GetPoseState
from dual_arm_flexbe_states.init_robot import InitRobotState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 10 2021
@author: Andy
'''
class SingleArmSampleSM(Behavior):
	'''
	Sample behavior of single arm moving
	'''


	def __init__(self):
		super(SingleArmSampleSM, self).__init__()
		self.name = 'Single Arm Sample'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', True)
		self.add_parameter('pose_1', '0.2, -0.25, -0.5')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:499 y:236, x:567 y:701
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:32 y:153
			OperatableStateMachine.add('init_robot',
										InitRobotState(robot_name=self.robot_name, en_sim=self.en_sim, speed=100),
										transitions={'done': 'fixed_joints_test', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:98 y:41
			OperatableStateMachine.add('fixed_joints_test',
										FixedJointMoveState(robot_name=self.robot_name, en_sim=self.en_sim, speed=100, slide_pos=0, joints=[0, -30, 0, 60, 0, -30, 0]),
										transitions={'done': 'fixed_pose_test', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:295 y:27
			OperatableStateMachine.add('fixed_pose_test',
										FixedPoseMoveState(robot_name=self.robot_name, en_sim=self.en_sim, mode='p2p', speed=100, pos=self.pose_1, euler=[0, 0, 0], phi=0),
										transitions={'done': 'get_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:499 y:34
			OperatableStateMachine.add('get_pose',
										GetPoseState(robot_name=self.robot_name),
										transitions={'done': 'move_robot', 'finish': 'back_home'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:959 y:36
			OperatableStateMachine.add('move_robot',
										customer_flexbe_states__IKMoveState(robot_name=self.robot_name, en_sim=self.en_sim),
										transitions={'done': 'get_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:494 y:156
			OperatableStateMachine.add('back_home',
										FixedJointMoveState(robot_name=self.robot_name, en_sim=self.en_sim, speed=100, slide_pos=0, joints=[0,0,0,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
