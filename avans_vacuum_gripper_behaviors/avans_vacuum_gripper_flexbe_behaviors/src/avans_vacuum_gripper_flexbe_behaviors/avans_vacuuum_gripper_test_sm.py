#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from avans_vacuum_gripper_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from avans_vacuum_gripper_flexbe_states.vacuum_gripper_status_state import VacuumGripperStatusState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Apr 27 2022
@author: Gerard Harkema
'''
class avans_vacuuum_gripper_testSM(Behavior):
	'''
	avans_vacuuum_gripper_test
	'''


	def __init__(self):
		super(avans_vacuuum_gripper_testSM, self).__init__()
		self.name = 'avans_vacuuum_gripper_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:853 y:50, x:116 y:192
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('EnableGripperVacuum',
										VacuumGripperControlState(target_time=1.0, gripper_vacuum_enable=True),
										transitions={'done': 'Wait'},
										autonomy={'done': Autonomy.Off})

			# x:388 y:42
			OperatableStateMachine.add('GripperStatus',
										VacuumGripperStatusState(blocking=True, clear=False),
										transitions={'received': 'DisableGripperVacuum', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'vacuum_active': 'vacuum_active'})

			# x:263 y:39
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=5.0),
										transitions={'done': 'GripperStatus'},
										autonomy={'done': Autonomy.Off})

			# x:618 y:40
			OperatableStateMachine.add('DisableGripperVacuum',
										VacuumGripperControlState(target_time=1.0, gripper_vacuum_enable=False),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
