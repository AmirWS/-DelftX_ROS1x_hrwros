#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hrwros_factory_states.compute_grasp_state import ComputeGraspState
from hrwros_factory_states.detect_part_camera_state import DetectPartCameraState
from hrwros_factory_states.moveit_to_joints_dyn_state import MoveitToJointsDynState as hrwros_factory_states__MoveitToJointsDynState
from hrwros_factory_states.set_conveyor_power_state import SetConveyorPowerState
from hrwros_factory_states.vacuum_gripper_control_state import VacuumGripperControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Sep 22 2022
@author: Amir wael
'''
class pickpartfromconveyorSM(Behavior):
	'''
	This is the behavior of hrwros_factory simulation that pick a part from the conveyor belt using Robot1
	'''


	def __init__(self):
		super(pickpartfromconveyorSM, self).__init__()
		self.name = 'pick part from conveyor'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pick_group = 'robot1'
		home1 = [1.57, -1.57, 1.24, -1.57, -1.57, 0]
		gripper1 = 'vacuum_gripper1_suction_cup'
		names1 = ['robot1_elbow_joint', 'robot1_shoulder_lift_joint', 'robot1_shoulder_pan_joint', 'robot1_wrist_1_joint', 'robot1_wrist_2_joint', 'robot1_wrist_3_joint']
		# x:677 y:489, x:606 y:25
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.pick_configuration = home1
		_state_machine.userdata.joint_names = []
		_state_machine.userdata.home1 = home1
		_state_machine.userdata.conveyor_speed = 100

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:107 y:25
			OperatableStateMachine.add('Start Conveyor',
										SetConveyorPowerState(stop=False),
										transitions={'succeeded': 'Detect Part', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'conveyor_speed'})

			# x:127 y:258
			OperatableStateMachine.add('Compute Pick Configuration',
										ComputeGraspState(group=pick_group, offset=0.0, joint_names=names1, tool_link=gripper1, rotation=3.1416),
										transitions={'continue': 'Move robot1 To Pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:109 y:124
			OperatableStateMachine.add('Detect Part',
										DetectPartCameraState(ref_frame='robot1_base', camera_topic='/hrwros/logical_camera_1', camera_frame='logical_camera_1_frame'),
										transitions={'continue': 'Stop Conveyor Belt', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})

			# x:133 y:506
			OperatableStateMachine.add('Move robot1 To Home Configuration',
										hrwros_factory_states__MoveitToJointsDynState(move_group=pick_group, offset=0.0, tool_link=gripper1, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'home1', 'joint_names': 'joint_names'})

			# x:145 y:339
			OperatableStateMachine.add('Move robot1 To Pick',
										hrwros_factory_states__MoveitToJointsDynState(move_group=pick_group, offset=0.0, tool_link=gripper1, action_topic='/move_group'),
										transitions={'reached': 'Activate Gripper', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:123 y:197
			OperatableStateMachine.add('Stop Conveyor Belt',
										SetConveyorPowerState(stop=True),
										transitions={'succeeded': 'Compute Pick Configuration', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'conveyor_speed'})

			# x:151 y:431
			OperatableStateMachine.add('Activate Gripper',
										VacuumGripperControlState(enable=True, service_name='/gripper1/control'),
										transitions={'continue': 'Move robot1 To Home Configuration', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
