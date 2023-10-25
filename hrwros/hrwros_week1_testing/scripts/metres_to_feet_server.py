#!/usr/bin/env python3

from hrwros_week1_testing.srv import ConvertMetresToFeet,ConvertMetresToFeetRequest,ConvertMetresToFeetResponse
import rospy
import numpy as np

def process_service_request(request):
	response=ConvertMetresToFeetResponse()
	if request.distance_metres<0:
		response.success=False
		response.distance_feet=-np.Inf
	else:
		response.distance_feet=request.distance_metres*3.22
		response.success=True
	return response



def metres_to_feet_server():
    service = rospy.Service('metres_to_feet', ConvertMetresToFeet, process_service_request)
    rospy.init_node('metres_to_feet_server', anonymous=False)
    rospy.loginfo('Convert metres to feet service is now available.')
    rospy.spin()



if __name__ == "__main__":
    metres_to_feet_server()
