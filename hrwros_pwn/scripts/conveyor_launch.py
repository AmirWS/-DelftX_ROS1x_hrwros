#!/usr/bin/env python3
import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import rospy
from hrwros_gazebo.srv import ConveyorBeltControl, ConveyorBeltControlRequest
from hrwros_gazebo.msg import ConveyorBeltState #conveyor speed
from hrwros_gazebo.msg import Proximity

request = ConveyorBeltControlRequest()
conveyor_set_speed = rospy.ServiceProxy('hrwros/conveyor/control', ConveyorBeltControl)

def detection_cb(proximity):
	global request, conveyor_set_speed
	proxmitiy_sensor_state=proximity.object_detected
	# Call the service here.
	if proxmitiy_sensor_state == False:
		request.state.power=float(100)
		print(str(proxmitiy_sensor_state)+"a1a")
	elif proxmitiy_sensor_state == True:
		request.state.power=float(0)
		print(str(proxmitiy_sensor_state)+" 2s2s")
	
	#Setting Speed request
	conveyor_set_speed(request)
	return proxmitiy_sensor_state	

    

if __name__ == "__main__":
    # Initialize the client ROS node.
    rospy.init_node("run_conveyor", anonymous=False)
    rospy.wait_for_service('hrwros/conveyor/control')
        
    
    #Speed setting
    proximity_sensor_state=rospy.Subscriber("break_beam_sensor",Proximity,detection_cb) #Sensed reading from proximity, off
    rospy.spin()
    