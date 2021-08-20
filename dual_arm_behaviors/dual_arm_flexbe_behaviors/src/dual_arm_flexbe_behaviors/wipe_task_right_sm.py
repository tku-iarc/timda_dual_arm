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
from dual_arm_flexbe_states.get_wipe_pose import GetWipePose
from dual_arm_flexbe_states.init_robot import InitRobotState
from dual_arm_flexbe_states.robotiq_2f_gripper_state import Robotiq2FGripperState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 19 2021
@author: Luis
'''
class wipe_task_rightSM(Behavior):
	'''
	wipe task behaviors for dual-arm robot. It can wipe desk
	'''


	def __init__(self):
		super(wipe_task_rightSM, self).__init__()
		self.name = 'wipe_task_right'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1202 y:49, x:404 y:33
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:47 y:115
			OperatableStateMachine.add('right_gripper_reset',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd='reset'),
										transitions={'done': 'init_right_arm'},
										autonomy={'done': Autonomy.Off})

			# x:48 y:350
			OperatableStateMachine.add('approach_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=35, pos=[-0.21, -0.280, -0.550], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'arrive_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:46 y:434
			OperatableStateMachine.add('arrive_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=35, pos=[-0.21, -0.077, -0.810], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'grap_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:769 y:286
			OperatableStateMachine.add('arrive_return_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.21, -0.077, -0.800], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'release_rag', 'failed': 'return_rag'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:454 y:299
			OperatableStateMachine.add('get_wipe_pose',
										GetWipePose(robot_name='right_arm'),
										transitions={'done': 'move_right_arm', 'finish': 'return_rag'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:280 y:436
			OperatableStateMachine.add('grap_rag',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd=180),
										transitions={'done': 'above_rag'},
										autonomy={'done': Autonomy.Off})

			# x:47 y:193
			OperatableStateMachine.add('init_right_arm',
										InitRobotState(robot_name='right_arm', en_sim=self.en_sim),
										transitions={'done': 'right_gripper_active', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:612 y:285
			OperatableStateMachine.add('move_right_arm',
										IKMoveState(robot_name='right_arm', en_sim=self.en_sim),
										transitions={'done': 'get_wipe_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:290 y:280
			OperatableStateMachine.add('rag_out',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.18, -0.26, -0.4200], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'get_wipe_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:947 y:294
			OperatableStateMachine.add('release_rag',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd='open'),
										transitions={'done': 'right_safety_back'},
										autonomy={'done': Autonomy.Off})

			# x:611 y:211
			OperatableStateMachine.add('return_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=35, pos=[-0.21, -0.077, -0.600], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'arrive_return_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:960 y:106
			OperatableStateMachine.add('right_back_home',
										FixedJointMoveState(robot_name='right_arm', en_sim=self.en_sim, speed=20, slide_pos=0, joints=[0,0,0,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:46 y:271
			OperatableStateMachine.add('right_gripper_active',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd='active'),
										transitions={'done': 'approach_rag'},
										autonomy={'done': Autonomy.Off})

			# x:945 y:217
			OperatableStateMachine.add('right_safety_back',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.16, -0.300, -0.600], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'right_back_home', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:287 y:355
			OperatableStateMachine.add('above_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=40, pos=[-0.21, -0.077, -0.500], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'rag_out', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
