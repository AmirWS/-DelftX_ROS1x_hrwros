#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


	
def publish_fn():
	pub=rospy.Publisher("cmd_vel_mux/input/teleop",Twist,queue_size=1)
	rospy.init_node("drive_turtlebot_circle",anonymous=True)
	rate=rospy.Rate(0.5)
	move=Twist()
	move.linear.x,move.linear.y, move.linear.z = [1,	0	,	0]
	move.angular.x, move.angular.y, move.angular.z = [1	,0	,	0]

	linear_speed="Sending Speeds:\n\tlinear:\n\t\tx:{}\ty:{}\tz:{}".format(move.linear.x,move.linear.y,move.linear.z)
	angular_speed="\tangular:\n\t\tx:{}\ty:{}\tz:{}".format(move.angular.x,move.angular.y,move.angular.z)

	rospy.loginfo(linear_speed+"\n"+angular_speed)

	while not rospy.is_shutdown():
		pub.publish(move)
		rate.sleep()
	else:
		print("\nInterruption is done, Exiting.....")


if __name__=="__main__":
	try:
		publish_fn()
	except rospy.ROSInterruptException:
		pass
