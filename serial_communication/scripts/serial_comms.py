#!/usr/bin/env python

import time
#import serial
import rospy
from serial_communication.msg import SensorData
#import socket
import serial

sensor_data_msg = SensorData()

def serial_comms():
    """
    This node reads in data from the ESP32 over serial
    """

    # initialize serial to the ttyUSB0 USB port
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    
    rospy.loginfo("Starting up Serial Comms!")

    # set up publisher on sensor_data topic
    pub = rospy.Publisher('sensor_data', SensorData, queue_size=1)

    rospy.init_node('serial_comms', anonymous=True)
    
    # initialize this node to run at 50 Hz
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        # read in data from serial port
        data = ser.readline()

        # if this is vive data, then parse it
        if (data[0:4] == "Vive"):
            parsed_data = data[5:].split(',')
            try:
                sensor_data_msg.x2 = int(parsed_data[0])
                sensor_data_msg.y2 = int(parsed_data[1])
                sensor_data_msg.x1 = int(parsed_data[2])
                sensor_data_msg.y1 = int(parsed_data[3])
            except:
                pass

        # if it's beacon data, then parse it
        if (data[0:6] == "Beacon"):
            parsed_data = data[7:].split(',')
            sensor_data_msg.beacon_angle = int(parsed_data[0])
        
        # publish latest sensor data message to the sensor_data topic
        pub.publish(sensor_data_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        serial_comms()
    except rospy.ROSInterruptException:
        pass
