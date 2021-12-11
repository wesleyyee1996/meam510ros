#!/usr/bin/env python

import rospy
import socket

from geometry_msgs.msg import Twist
from serial_communication.msg import SensorData
from std_msgs.msg import String

UDP_IP = ""
UDP_PORT_REC = 2522
UDP_PORT_SEND = 2700

class Test:
    def __init__(self):
        self.robot_id = "0"

test = Test()

socket_send_loc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_send_loc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

def send_robot_location_callback(sensor_data):
    to_send = str(test.robot_id) + "," + str(sensor_data.x1) + "," + str(sensor_data.y1)
    to_send = str(to_send)
    rospy.loginfo("Sending robot location "+str(to_send))
    socket_send_loc.sendto(to_send.encode(), ("192.168.0.255", 2510))


def get_robot_id(rob_id):
    test.robot_id = rob_id.data
    #rospy.loginfo("Received robot id "+test.robot_id)

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    #rospy.loginfo("Connected to local IP ", s.getsockname()[0], " on port ", UDP_PORT_REC)
    s.close()

def udp_comms():
    pub = rospy.Publisher('motor_control', Twist, queue_size=10)
    rospy.Subscriber("robot_id", String, get_robot_id)
    rospy.Subscriber("sensor_data", SensorData, send_robot_location_callback)
    rospy.init_node('udp_communication', anonymous=True)
    rate = rospy.Rate(50)

    get_local_ip()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT_REC))

    rospy.loginfo("UDP Communications started. Now listening!")

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)
        motor_drive_data = data.split(',')
        motor_msg = Twist()
        motor_msg.linear.x = float(motor_drive_data[1])
        motor_msg.angular.z = float(motor_drive_data[0])
        pub.publish(motor_msg)
        

if __name__ == '__main__':
    try:
        udp_comms()
    except rospy.ROSInterruptException:
        pass
