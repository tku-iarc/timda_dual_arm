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
from dual_arm_flexbe_behaviors.wipe_task_left_sm import wipe_task_leftSM
from dual_arm_flexbe_behaviors.wipe_task_right_sm import wipe_task_rightSM
from dual_arm_flexbe_states.wait_timda_mobile import WaitTimdaMobile
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 19 2021
@author: Luis
'''
class CustomerServiceSM(Behavior):
	'''
	dual arm customer service behaviors
	'''


	def __init__(self):
		super(CustomerServiceSM, self).__init__()
		self.name = 'Customer Service'

		# parameters of this behavior
		self.add_parameter('en_sim', False)

		# references to used behaviors
		self.add_behavior(wipe_task_leftSM, 'Container/wipe_task_left')
		self.add_behavior(wipe_task_rightSM, 'Container/wipe_task_right')
		self.add_behavior(scratch_deskSM, 'scratch_desk')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:725 y:300, x:399 y:40
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('wipe_task_left', 'finished'), ('wipe_task_right', 'finished')]),
										('failed', [('wipe_task_left', 'failed'), ('wipe_task_right', 'failed')])
										])

		with _sm_container_0:
			# x:50 y:102
			OperatableStateMachine.add('wipe_task_right',
										self.use_behavior(wipe_task_rightSM, 'Container/wipe_task_right'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:293 y:103
			OperatableStateMachine.add('wipe_task_left',
										self.use_behavior(wipe_task_leftSM, 'Container/wipe_task_left'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('wait_timda_mobile',
										WaitTimdaMobile(en_sim=self.en_sim),
										transitions={'done': 'scratch_desk', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:179 y:152
			OperatableStateMachine.add('scratch_desk',
										self.use_behavior(scratch_deskSM, 'scratch_desk'),
										transitions={'finished': 'Container', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:403 y:236
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
