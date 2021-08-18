#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dual_arm_flexbe_behaviors.scratch_desk_sm import scratch_deskSM
from dual_arm_flexbe_behaviors.wipe_desk_sm import wipe_deskSM
from dual_arm_flexbe_states.wait_timda_mobile import WaitTimdaMobile
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 18 2021
@author: Luis
'''
class Customer_serviceSM(Behavior):
	'''
	Sample behavior of dual arm customer service
	'''


	def __init__(self):
		super(Customer_serviceSM, self).__init__()
		self.name = 'Customer_service'

		# parameters of this behavior
		self.add_parameter('robot_name', 'right_arm')
		self.add_parameter('en_sim', False)

		# references to used behaviors
		self.add_behavior(scratch_deskSM, 'scratch_desk')
		self.add_behavior(wipe_deskSM, 'wipe_desk')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:825 y:207, x:347 y:545
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:85 y:39
			OperatableStateMachine.add('wait_timda_mobile',
										WaitTimdaMobile(en_sim=self.en_sim),
										transitions={'done': 'scratch_desk', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:484 y:32
			OperatableStateMachine.add('wipe_desk',
										self.use_behavior(wipe_deskSM, 'wipe_desk'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:256 y:35
			OperatableStateMachine.add('scratch_desk',
										self.use_behavior(scratch_deskSM, 'scratch_desk'),
										transitions={'finished': 'wipe_desk', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
