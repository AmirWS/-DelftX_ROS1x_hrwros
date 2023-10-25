#!/usr/bin/env python3



import rospy
from std_msgs.msg import Float32

class publisherC(object):
    

    def __init__(self):

        self.node=rospy.init_node("n2", anonymous=False)
        self.pub=rospy.Publisher("topic_2",Float32,queue_size=10)
        self.rate=rospy.Rate(1)

        self.sub=rospy.Subscriber("topic_1",Float32, callback=self.simpleSubscriber)
        self.data=rospy.wait_for_message("topic_1",Float32,timeout=None)
        

    def simpleSubscriber(self,message):
        self.message="node 1 message is --"+str(message.data)+"--."
        self.data=message.data*10
        #rospy.spin() 
# runs while rospy is running, to avoid exiting, but not used because in goes onto loop and cannot publish
 
    def simplePublisher(self):

        while not rospy.is_shutdown() and self.data!=0:
            i=self.data
            rospy.loginfo(self.message)
            rospy.loginfo("Sending N2 Message")
            self.pub.publish(i)
            print("i is :",i)
            self.rate.sleep()
    



if __name__ == '__main__':
    try:

        publishing_class=publisherC()
        publishing_class.simplePublisher()
    except rospy.ROSInterruptException:
        pass
