#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs


"""
def location_cb(location):
    plan.
"""
def simple_pick_place():
    # First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)

    # Instantiate a MoveGroupCommander object.  This object is an interface
    # to one group of joints.  In this case the group refers to the joints of
    # robot1. This interface can be used to plan and execute motions on robot1.
    robot1_group = moveit_commander.MoveGroupCommander("robot1")
    # MoveGroup Commander Object for robot2.
    # We're not using it so let's leave it commented out
    # robot2_group = moveit_commander.MoveGroupCommander("robot2")

    # Action clients to the ExecuteTrajectory action server.
    robot1_client = actionlib.SimpleActionClient(
        'execute_trajectory',
        moveit_msgs.msg.ExecuteTrajectoryAction)
    robot1_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for robot1')
    #robot2_client = actionlib.SimpleActionClient(
    #    'execute_trajectory',
    #    moveit_msgs.msg.ExecuteTrajectoryAction)

    #robot2_client.wait_for_server()
    #rospy.loginfo('Execute Trajectory server is available for robot2')

    # Set a named joint configuration as the goal to plan for a move group.
    # Named joint configurations are the robot poses
    # defined via MoveIt! Setup Assistant.
    robot1_group.set_named_target("R1Home")

    # Plan to the desired joint-space goal
    # using the default planner (RRTConnect).
    # Gets only the plan, and discards the other elements returned by plan()
    _, plan, _, _ = robot1_group.plan()
    

    # Create a goal message object for the action server.
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()

    # Update the trajectory in the goal message.
    robot1_goal.trajectory = plan

    # Send the goal to the action server.
    robot1_client.send_goal(robot1_goal)
    robot1_client.wait_for_result()
    #print(plan)
    print(type(plan))
    print("HOOOOOOOOOOOOOOOOOOOOOOOOOOOOOMMMMMMMMMMEEEEEEEEEEEEEEEEEEEEEEEEEEE")

    robot1_group.set_named_target("R1PreGrasp")

    _, plan, _, _ = robot1_group.plan()
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan

    robot1_client.send_goal(robot1_goal)
    robot1_client.wait_for_result()
    print("PREEEEEEEEEEEEEGRAAAAAAaaaaAAAAAAAAAAAAAAAAASP")
    #object_location=rospy.Subscriber("object_location_to_r1",geometry_msgs,location_cb)

    
    print("CAARTEEEEEEEEEEESIIIIIIIIAAAAAAAAAAAAAAAAAAAAAAN")
    robot1_group.set_named_target("R1Place")

    _, plan, _, _ = robot1_group.plan()
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan

    robot1_client.send_goal(robot1_goal)
    robot1_client.wait_for_result()

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        simple_pick_place()
    except rospy.ROSInterruptException:
        pass
