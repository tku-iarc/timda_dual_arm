#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dual_arm_flexbe_behaviors.single_arm_sample_sm import SingleArmSampleSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 13 2021
@author: Andy Chien
'''
class DualArmSampleSM(Behavior):
	'''
	Sample behavior for dual-arm robot. It can move two arms at the same time.
	'''


	def __init__(self):
		super(DualArmSampleSM, self).__init__()
		self.name = 'Dual Arm Sample'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SingleArmSampleSM, 'Dual Arm Container/Single Arm Sample Left')
		self.add_behavior(SingleArmSampleSM, 'Dual Arm Container/Single Arm Sample Right')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:296 y:78, x:278 y:18
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_dual_arm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('Single Arm Sample Left', 'finished')]),
										('failed', [('Single Arm Sample Left', 'failed')]),
										('finished', [('Single Arm Sample Right', 'finished')]),
										('failed', [('Single Arm Sample Right', 'failed')])
										])

		with _sm_dual_arm_container_0:
			# x:95 y:90
			OperatableStateMachine.add('Single Arm Sample Left',
										self.use_behavior(SingleArmSampleSM, 'Dual Arm Container/Single Arm Sample Left',
											parameters={'robot_name': "left_arm", 'en_sim': True, 'pose_1': "0.2, 0.25, -0.5"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:343 y:89
			OperatableStateMachine.add('Single Arm Sample Right',
										self.use_behavior(SingleArmSampleSM, 'Dual Arm Container/Single Arm Sample Right',
											parameters={'robot_name': "right_arm", 'en_sim': True, 'pose_1': "0.2, -0.25, -0.5"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:93 y:27
			OperatableStateMachine.add('Dual Arm Container',
										_sm_dual_arm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
