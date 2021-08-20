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
from dual_arm_flexbe_states.init_robot import InitRobotState
from dual_arm_flexbe_states.robotiq_2f_gripper_state import Robotiq2FGripperState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 19 2021
@author: Luis
'''
class wipe_task_leftSM(Behavior):
	'''
	wipe task behaviors for dual-arm robot. It can grap alcohol bottle to squeeze
	'''


	def __init__(self):
		super(wipe_task_leftSM, self).__init__()
		self.name = 'wipe_task_left'

		# parameters of this behavior
		self.add_parameter('robot_name', 'left_arm')
		self.add_parameter('en_sim', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1224 y:48, x:502 y:9
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:49 y:33
			OperatableStateMachine.add('left_gripper_reset',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd='reset'),
										transitions={'done': 'init_left_arm'},
										autonomy={'done': Autonomy.Off})

			# x:207 y:240
			OperatableStateMachine.add('approach_alcohol_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='p2p', speed=35, pos=[-0.1146, 0.2363, -0.600], euler=[-47.024, 3.005, -44.998], phi=0),
										transitions={'done': 'arrive_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:208 y:315
			OperatableStateMachine.add('arrive_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='p2p', speed=35, pos=[-0.1146, 0.1192, -0.6750], euler=[-47.024, 3.005, -44.998], phi=0),
										transitions={'done': 'grab_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:783 y:309
			OperatableStateMachine.add('arrive_return_bottle',
										FixedPoseMoveState(robot_name=self.robot_name, en_sim=self.en_sim, mode='line', speed=20, pos=[-0.1146, 0.1462, -0.6710], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'release_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:410 y:463
			OperatableStateMachine.add('get_spray_pose',
										GetSprayAlcoholPose(robot_name='left_arm'),
										transitions={'done': 'move_left_arm', 'finish': 'squeeze_alcohol'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:607 y:398
			OperatableStateMachine.add('get_spray_pose_II',
										GetSprayAlcoholPoseII(robot_name='left_arm'),
										transitions={'done': 'move_left_arm_II', 'finish': 'return_bottle'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:210 y:382
			OperatableStateMachine.add('grab_bottle',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd=59),
										transitions={'done': 'above_bottle'},
										autonomy={'done': Autonomy.Off})

			# x:49 y:120
			OperatableStateMachine.add('init_left_arm',
										InitRobotState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'left_gripper_active', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:935 y:140
			OperatableStateMachine.add('left_back_home',
										FixedJointMoveState(robot_name='left_arm', en_sim=self.en_sim, speed=20, slide_pos=0, joints=[0,0,0,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:49 y:191
			OperatableStateMachine.add('left_gripper_active',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd='active'),
										transitions={'done': 'approach_alcohol_bottle'},
										autonomy={'done': Autonomy.Off})

			# x:938 y:234
			OperatableStateMachine.add('left_safety_back',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=20, pos=[-0.1250, 0.3163, -0.6710], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'left_back_home', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:413 y:388
			OperatableStateMachine.add('move_left_arm',
										IKMoveState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'get_spray_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:605 y:310
			OperatableStateMachine.add('move_left_arm_II',
										IKMoveState(robot_name='left_arm', en_sim=self.en_sim),
										transitions={'done': 'get_spray_pose_II', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_cmd': 'robot_cmd'})

			# x:942 y:310
			OperatableStateMachine.add('release_bottle',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd='open'),
										transitions={'done': 'left_safety_back'},
										autonomy={'done': Autonomy.Off})

			# x:786 y:396
			OperatableStateMachine.add('return_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=35, pos=[-0.1146, 0.1462, -0.5500], euler=[-44.024, -0.005, -44.998], phi=0),
										transitions={'done': 'arrive_return_bottle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:606 y:474
			OperatableStateMachine.add('squeeze_alcohol',
										Robotiq2FGripperState(robot_name='left_arm', en_sim=self.en_sim, gripper_cmd=105),
										transitions={'done': 'get_spray_pose_II'},
										autonomy={'done': Autonomy.Off})

			# x:207 y:454
			OperatableStateMachine.add('above_bottle',
										FixedPoseMoveState(robot_name='left_arm', en_sim=self.en_sim, mode='line', speed=30, pos=[-0.1146, 0.1282, -0.6000], euler=[-47.024, 3.005, -44.998], phi=0),
										transitions={'done': 'get_spray_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
