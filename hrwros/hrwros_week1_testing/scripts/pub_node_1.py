#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


	
def publish_fn():
	pub=rospy.Publisher("chatter_1",String,queue_size=10)
	rospy.init_node("node_pub",anonymous=True)
	rate=rospy.Rate(1)
	while not rospy.is_shutdown():
		rospy.loginfo("I SENT A MESSAGE")
		pub.publish("IAM THE MESSAGE")
		rate.sleep()
	else:
		print("\nInterruption is done, Exiting.....")



if __name__=="__main__":
	try:
		publish_fn()
	except rospy.ROSInterruptException:
		pass
