#!/usr/bin/env python3



import rospy
from std_msgs.msg import String

class publisherC(object):
    

    def __init__(self):
        self.node=rospy.init_node("node_name", anonymous=False)
        self.pub=rospy.Publisher("topic_1",String,queue_size=10)
        self.rate=rospy.Rate(1)


    def simplePublisher(self):
        topic1_content = "Welcome to Hello (Real) World with ROS!!!"

        while not rospy.is_shutdown():
            self.pub.publish(topic1_content)
            self.rate.sleep()


if __name__ == '__main__':
    try:

        publishing_class=publisherC()
        publishing_class.simplePublisher()
    except rospy.ROSInterruptException:
        pass
