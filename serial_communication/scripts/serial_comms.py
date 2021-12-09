#!/usr/bin/env python

import serial
import rospy
from serial_communication.msg import SensorData

sensor_data_msg = SensorData()

def serial_comms():
    ser = serial.Serial('/dev/ttyUSB0', 115200)

    rospy.loginfo("Starting up Serial Comms!")

    pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)
    rospy.init_node('serial_comms', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = ser.readline()
        if (data[0:4] == "Vive"):
            parsed_data = data[5:].split(',')
            sensor_data_msg.x1 = int(parsed_data[0])
            sensor_data_msg.y1 = int(parsed_data[1])
            sensor_data_msg.x2 = int(parsed_data[2])
            sensor_data_msg.y2 = int(parsed_data[3])
        if (data[0:6] == "beacon"):
            parsed_data = data[7:].split(',')
            angle = int(parsed_data[0])
            print(angle)
            if angle < -75 or angle > 75:

                sensor_data_msg.beacon_stalled = 1
            else:
                sensor_data_msg.beacon_stalled = 0
            sensor_data_msg.beacon_angle = angle
            
        pub.publish(sensor_data_msg)

if __name__ == '__main__':
    try:
        serial_comms()
    except rospy.ROSInterruptException:
        pass
