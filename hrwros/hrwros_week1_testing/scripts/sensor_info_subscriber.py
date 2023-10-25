#!/usr/bin/env python3

import rospy
from hrwros_week1_testing.msg import SensorInformation
from hrwros_utilities.sim_sensor_data import distSensorData as getSensorData

def callback_fn(data):
	message=("Reading from sensor: {}".format(data.sensor_data.range))
	rospy.loginfo(message)





def sensorInfoSubscriber():
	subscriber=rospy.Subscriber("sensor_info",SensorInformation,callback_fn)
	rospy.init_node('sensor_info_subscriber', anonymous=False)
	rospy.spin()






if __name__ == '__main__':
    try:
        sensorInfoSubscriber()
    except rospy.ROSInterruptException:
        pass