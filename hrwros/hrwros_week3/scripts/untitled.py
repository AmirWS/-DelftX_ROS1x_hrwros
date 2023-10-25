#!/usr/bin/env python3
import sys
  
# appending a path
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
  
# importing required module
import rospy
# aceesing its content
rospy.Pub

"""

import importlib.util
spec = importlib.util.spec_from_file_location("rospy", "/opt/ros/noetic/lib/python3/dist-packages/rospy/__init__.py")
foo = importlib.util.module_from_spec(spec)
sys.modules["rospy"] = foo
spec.loader.exec_module(foo)"""



# Cartesian Paths
    # ^^^^^^^^^^^^^^^
    # You can plan a cartesian path directly by specifying a list of waypoints
    # for the end-effector to go through.
    waypoints = []
    # start with the current pose
    current_pose = robot1_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = robot1_group.get_current_pose()

    # create linear offsets to the current pose
    new_eef_pose = geometry_msgs.msg.Pose()

    # Manual offsets because we don't have a camera to detect objects yet.
    new_eef_pose.position.x = current_pose.pose.position.x + 0.10
    new_eef_pose.position.y = current_pose.pose.position.y - 0.20
    new_eef_pose.position.z = current_pose.pose.position.z - 0.20

    # Retain orientation of the current pose.
    new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

    waypoints.append(new_eef_pose)
    waypoints.append(current_pose.pose)

    # We want the cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in cartesian
    # translation.  We will specify the jump threshold as 0.0, effectively
    # disabling it.
    fraction = 0.0
    for count_cartesian_path in range(0, 3):
        if fraction < 1.0:
            (plan_cartesian, fraction) = robot1_group.compute_cartesian_path(
                                                    waypoints,
                                                    0.01,     # eef_step
                                                    0.0)      # jump_threshold
        else:
            break

    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan_cartesian

    robot1_client.send_goal(robot1_goal)
    robot1_client.wait_for_result()