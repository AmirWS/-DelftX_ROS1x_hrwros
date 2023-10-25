#!/usr/bin/env python3



import rospy
from std_msgs.msg import Float32
import numpy as np

i = 0
class publisherC(object):
    

    def __init__(self):
        self.node=rospy.init_node("node_1", anonymous=False)
        self.pub=rospy.Publisher("topic_1", Float32, queue_size=10)
        self.rate=rospy.Rate(1)


    def simplePublisher(self):
        global i
        while not rospy.is_shutdown():
            self.pub.publish(i)
            rospy.loginfo("Sending value: "+str(i))
            print(np.array[1,2,3])
            i+=1
            self.rate.sleep()


if __name__ == '__main__':
    try:

        publishing_class=publisherC()
        publishing_class.simplePublisher()
       
    except rospy.ROSInterruptException:
        pass