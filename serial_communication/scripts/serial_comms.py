#!/usr/bin/env python

import time
#import serial
import rospy
from serial_communication.msg import SensorData
#import socket
import serial

sensor_data_msg = SensorData()

def serial_comms():
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    #sensor_data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #sensor_data_socket.bind(("192.168.0.171", 3510))
    
    rospy.loginfo("Starting up Serial Comms!")

    pub = rospy.Publisher('sensor_data', SensorData, queue_size=1)
    rospy.init_node('serial_comms', anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        data = ser.readline()
        if (data[0:4] == "Vive"):
            parsed_data = data[5:].split(',')
            try:
                sensor_data_msg.x2 = int(parsed_data[0])
                sensor_data_msg.y2 = int(parsed_data[1])
                sensor_data_msg.x1 = int(parsed_data[2])
                sensor_data_msg.y1 = int(parsed_data[3])
            except:
                pass
        if (data[0:6] == "Beacon"):
            parsed_data = data[7:].split(',')
            sensor_data_msg.beacon_angle = int(parsed_data[0])
        """
        data, addr = sensor_data_socket.recvfrom(1024)
        split_data = data.split(';')
        vive_data = split_data[0].split(',')
        beacon_data = int(split_data[1])
        sensor_data_msg.x2 = int(vive_data[0])
        sensor_data_msg.y2 = int(vive_data[1])
        sensor_data_msg.x1 = int(vive_data[2])
        sensor_data_msg.y1 = int(vive_data[3])
        sensor_data_msg.beacon_stalled = beacon_data
        """ 
        pub.publish(sensor_data_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        serial_comms()
    except rospy.ROSInterruptException:
        pass
