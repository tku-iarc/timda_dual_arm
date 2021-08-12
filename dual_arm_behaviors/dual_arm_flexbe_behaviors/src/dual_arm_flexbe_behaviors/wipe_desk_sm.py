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
from dual_arm_flexbe_states.get_spray_alcohol_II import GetSprayAlcoholPoseII
from dual_arm_flexbe_states.get_spray_alcohol_pose import GetSprayAlcoholPose
from dual_arm_flexbe_states.get_wipe_pose import GetWipePose
from dual_arm_flexbe_states.init_robot import InitRobotState
from dual_arm_flexbe_states.robotiq_2f_gripper_state import Robotiq2FGripperState
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
		# x:1036 y:668, x:501 y:670
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:49 y:33
			OperatableStateMachine.add('left_gripper_reset',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd='reset'),
										transitions={'done': 'right_gripper_reset'},
										autonomy={'done': Autonomy.Off})

			# x:215 y:91
			OperatableStateMachine.add('approach_alcohol_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='p2p', speed=20, pos=[-0.1146, 0.2363, -0.600], euler=[-47.024, 3.005, -44.998], phi=0),
										transitions={'done': 'arrive_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:605 y:9
			OperatableStateMachine.add('approach_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.16, -0.300, -0.550], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'arrive_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:222 y:178
			OperatableStateMachine.add('arrive_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='p2p', speed=20, pos=[-0.1146, 0.1282, -0.6680], euler=[-47.024, 3.005, -44.998], phi=0),
										transitions={'done': 'grab_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:550 y:97
			OperatableStateMachine.add('arrive_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.16, -0.097, -0.800], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'grap_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1139 y:153
			OperatableStateMachine.add('arrive_return_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.1146, 0.1582, -0.6710], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'release_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:963 y:399
			OperatableStateMachine.add('arrive_return_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.16, -0.097, -0.800], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'release_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:764 y:9
			OperatableStateMachine.add('get_spray_pose',
										GetSprayAlcoholPose(robot_name='left_arm'),
										transitions={'done': 'move_left_arm', 'finish': 'squeeze_alcohol'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:1124 y:6
			OperatableStateMachine.add('get_spray_pose_II',
										GetSprayAlcoholPoseII(robot_name='left_arm'),
										transitions={'done': 'move_left_arm_II', 'finish': 'get_wipe_pose'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:920 y:87
			OperatableStateMachine.add('get_wipe_pose',
										GetWipePose(robot_name='right_arm'),
										transitions={'done': 'move_right_arm', 'finish': 'return_bottle'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:227 y:281
			OperatableStateMachine.add('grab_bottle',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd=57),
										transitions={'done': 'above_bottle'},
										autonomy={'done': Autonomy.Off})

			# x:529 y:191
			OperatableStateMachine.add('grap_rag',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd=180),
										transitions={'done': 'get_spray_pose'},
										autonomy={'done': Autonomy.Off})

			# x:48 y:193
			OperatableStateMachine.add('init_left_arm',
										InitRobotState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'left_gripper_active', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:46 y:357
			OperatableStateMachine.add('init_right_arm',
										InitRobotState(robot_name='right_arm', en_sim=self.en_sim),
										transitions={'done': 'right_gripper_active', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:960 y:477
			OperatableStateMachine.add('left_back_home',
										FixedJointMoveState(robot_name='left_arm', en_sim=self.en_sim, speed=20, slide_pos=0, joints=[0,0,0,0,0,0,0]),
										transitions={'done': 'right_back_home', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:43 y:279
			OperatableStateMachine.add('left_gripper_active',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd='active'),
										transitions={'done': 'init_right_arm'},
										autonomy={'done': Autonomy.Off})

			# x:1141 y:312
			OperatableStateMachine.add('left_safety_back',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.1250, 0.3163, -0.6710], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'return_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:714 y:192
			OperatableStateMachine.add('move_left_arm',
										IKMoveState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'get_spray_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:699 y:271
			OperatableStateMachine.add('move_left_arm_II',
										IKMoveState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'get_spray_pose_II', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:688 y:342
			OperatableStateMachine.add('move_right_arm',
										IKMoveState(robot_name='right_arm', en_sim=self.en_sim),
										transitions={'done': 'get_wipe_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:1138 y:226
			OperatableStateMachine.add('release_bottle',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd='open'),
										transitions={'done': 'left_safety_back'},
										autonomy={'done': Autonomy.Off})

			# x:1146 y:398
			OperatableStateMachine.add('release_rag',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd='open'),
										transitions={'done': 'right_safety_back'},
										autonomy={'done': Autonomy.Off})

			# x:1135 y:75
			OperatableStateMachine.add('return_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.1146, 0.1582, -0.5500], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'arrive_return_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:967 y:278
			OperatableStateMachine.add('return_rag',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.13, -0.097, -0.600], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'arrive_return_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1145 y:564
			OperatableStateMachine.add('right_back_home',
										FixedJointMoveState(robot_name='right_arm', en_sim=self.en_sim, speed=20, slide_pos=0, joints=[0,0,0,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:42 y:454
			OperatableStateMachine.add('right_gripper_active',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd='active'),
										transitions={'done': 'approach_alcohol_bottle'},
										autonomy={'done': Autonomy.Off})

			# x:47 y:115
			OperatableStateMachine.add('right_gripper_reset',
										Robotiq2FGripperState(robot_name='right_arm', en_sim=self.en_sim, gripper_cmd='reset'),
										transitions={'done': 'init_left_arm'},
										autonomy={'done': Autonomy.Off})

			# x:1147 y:473
			OperatableStateMachine.add('right_safety_back',
										FixedPoseMoveState(robot_name='right_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.16, -0.300, -0.600], euler=[45.000, 0.000, 0.000], phi=0),
										transitions={'done': 'left_back_home', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:925 y:13
			OperatableStateMachine.add('squeeze_alcohol',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd=100),
										transitions={'done': 'get_spray_pose_II'},
										autonomy={'done': Autonomy.Off})

			# x:431 y:15
			OperatableStateMachine.add('above_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.1146, 0.1282, -0.6000], euler=[-47.024, 3.005, -44.998], phi=0),
										transitions={'done': 'approach_rag', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
