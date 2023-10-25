#!/usr/bin/env python3



import rospy
from std_msgs.msg import Float32

class publisherC(object):
    

    def __init__(self):
        self.node=rospy.init_node("n3", anonymous=False)
        self.rate=rospy.Rate(1)
        self.sub=rospy.Subscriber("topic_2",Float32, callback=self.simpleSubscriber)
        self.data=rospy.wait_for_message("topic_2",Float32,timeout=None)
        

    def simpleSubscriber(self,message):

        self.message="node 2 message is --"+str(message.data)+"--."
        rospy.loginfo(self.message)
        #self.rate.sleep()
# runs while rospy is running, to avoid exiting, but not used because in goes onto loop and cannot publish
 
    def simplePublisher(self):
        topic2_content = "\t--N2 MESSAGE--\t"

        while not rospy.is_shutdown() and self.message!=0:
            rospy.loginfo(self.message)
            rospy.loginfo("Sending N2 Message")
            self.pub.publish(topic2_content)
            self.rate.sleep()
    



if __name__ == '__main__':
    try:
        publishing_class=publisherC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
