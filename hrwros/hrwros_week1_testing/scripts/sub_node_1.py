#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def sub_cb_fn(message):
	rospy.loginfo(message.data)


def sub_fn():
	sub=rospy.Subscriber("chatter_1",String,sub_cb_fn)
	rospy.init_node("node_sub",anonymous=True)
	rospy.spin()







if __name__ == "__main__":
	
	try:
		sub_fn()
	except	rospy.ROSInterruptException:
		pass