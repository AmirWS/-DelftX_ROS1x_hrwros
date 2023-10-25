#!/usr/bin/env python3

import rospy
from hrwros_week1_testing.msg import SensorInformation
from hrwros_utilities.sim_sensor_data import distSensorData as getSensorData


def sensorInfoPublisher():
    si_publisher = rospy.Publisher('sensor_info', SensorInformation, queue_size=10)
    rospy.init_node('sensor_info_publisher', anonymous=False)
    rate = rospy.Rate(1)

    # Create a new SensorInformation object and fill in its contents.
    sensor_info = SensorInformation()

    # Fill in the header information.
    sensor_info.sensor_data.header.stamp = rospy.Time.now()
    sensor_info.sensor_data.header.frame_id = 'distance_sensor_frame'

    # Fill in the sensor data information.
    sensor_info.sensor_data.radiation_type = sensor_info.sensor_data.ULTRASOUND
    sensor_info.sensor_data.field_of_view = 0.5  # Field of view of the sensor in rad.
    sensor_info.sensor_data.min_range = 0.02  # Minimum distance range of the sensor in m.
    sensor_info.sensor_data.max_range = 2.0  # Maximum distance range of the sensor in m.

    # Fill in the manufacturer name and part number.
    sensor_info.maker_name = 'The Ultrasound Company'
    sensor_info.part_number = 123456

    while not rospy.is_shutdown():
        # Read the sensor data from a simulated sensor.
        sensor_info.sensor_data.range = getSensorData(sensor_info.sensor_data.radiation_type,
            sensor_info.sensor_data.min_range, sensor_info.sensor_data.max_range)

        # Publish the sensor information on the /sensor_info topic.
        si_publisher.publish(sensor_info)
        # Print log message if all went well.
        rospy.loginfo("All went well. Publishing topic ")
        rate.sleep()


if __name__ == '__main__':
    try:
        sensorInfoPublisher()
    except rospy.ROSInterruptException:
        pass