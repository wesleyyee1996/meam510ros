#!/usr/bin/env python

import rospy
import socket
import thread

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_msgs.msg import Bool

UDP_IP = ""
UDP_PORT_REC = 2522
UDP_PORT_SEND = 2700

class GetRobotId:
    def __init__(self):
        self.robot_id = "0"

# just a class to hold the robot id
robo_id = GetRobotId()

# socket for sending robot location as broadcast
socket_send_loc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
socket_send_loc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
socket_send_loc.bind((UDP_IP,0))

def send_robot_location_callback(vive_data):
    """
    every time we receive sensor data from the serial_communication
    ROS node, we package it up along with the robot ID and broadcast
    """
    to_send = str(robo_id.robot_id) + "," + str(int(vive_data.twist.linear.x)) + "," + str(int(vive_data.twist.linear.y)) + str(0)

    
    #rospy.loginfo("Sending robot location "+str(to_send))
    socket_send_loc.sendto(to_send.encode(), ("192.168.1.255", 2510))


def get_robot_id(rob_id):
    """
    callback function for robot_id topic, which is received from the
    get_robot_id ROS node
    """
    robo_id.robot_id = rob_id.data
    #rospy.loginfo("Received robot id "+test.robot_id)

def get_local_ip():
    """
    gets the local IP that the raspberry PI was registered under
    for informational purposes only
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    rospy.loginfo("Connected to local IP %s ", s.getsockname()[0])
    s.close()


def udp_comms():

    # set up publisher to motor_control topic
    pub = rospy.Publisher('motor_control', Twist, queue_size=10)

    # set up publisher to led_signal topic
    led_signal_pub = rospy.Publisher('led_signal', Bool, queue_size=10)

    # set up subscriber callback function for robot_id topic
    rospy.Subscriber("robot_id", String, get_robot_id)

    # set up subscriber callback function for sensor_data topic
    rospy.Subscriber("vive_pose", TwistStamped, send_robot_location_callback)
    
    # initialize the udp_communication node
    rospy.init_node('udp_communication', anonymous=True)

    # set the loop rate to 50 Hz
    rate = rospy.Rate(10)

    get_local_ip()

    # setup the socket for receiving commands from computer
    receive_commands_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receive_commands_sock.bind((UDP_IP, UDP_PORT_REC))

    rospy.loginfo("UDP Communications started. Now listening!")

    while not rospy.is_shutdown():
        try:
            # every time we receive command data from the computer,
            # package this up into a geometry_msgs/Twist msg and publish
            # to the motor_control topic
            data, addr = receive_commands_sock.recvfrom(1024)
            motor_drive_data = data.split(',')
            motor_msg = Twist()
            motor_msg.linear.x = float(motor_drive_data[1])
            motor_msg.angular.z = float(motor_drive_data[0])
            pub.publish(motor_msg)

            # also publish to led signal topic
            led_signal_msg = Bool()
            led_signal_msg.data = True
            led_signal_pub.publish(led_signal_msg)
        except socket.timeout:
            rospy.logerr("UDP Comms: Haven't received manual control data from computer! Reconnecting.")
        rate.sleep()
        

if __name__ == '__main__':
    try:
        udp_comms()
    except rospy.ROSInterruptException:
        pass
