#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from serial_communication.msg import SensorData

class Global:
    # create variable to save listened info
    def __init__(self):
        #self.robot_angle = 600
        self.beacon_angle = 600
        self.beacon_sensing = 600
        self.obstacle_distance = 0

    def has_data(self):
        if self.beacon_angle != 600: 
            return True
        return False

g = Global()

#def save_robot_angle(vive_pose):
    # record robot angle
    #g.robot_angle = vive_data.twist.angluar.z 


def save_beacon_info(sensor_data):
    # record beacon info
    g.beacon_angle = sensor_data.beacon_angle
    g.beacon_sensing = sensor_data.beacon_stalled


def save_obstacle_distance(scan_data):
    # record the obstacle distance stright infront of the robot
    for i in range (107,118):
        g.obstacle_distance += scan_data.ranges[i]/11





def move_to_beacon():

    rospy.init_node("move_to_beacon", anonymous = True)
    pub = rospy.Publisher("motor_control", Twist, queue_size = 10)
    #rospy.Subscriber("vive_pose", TwistStamped, save_robot_angle)
    rospy.Subscriber("sensor_data", SensorData, save_beacon_info)
    rospy.Subscriber("scan", LaserScan, save_obstacle_distance)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        if g.has_data():

            motor_msg = Twist()

            if g.beacon_sensing != 0:
                if g.beacon_angle < -10:
                    motor_msg.linear.x = 0
                    motor_msg.angular.z = 0.35 + 0.1*g.beacon_angle/90
                elif g.beacon_angle > 10:
                    motor_msg.linear.x = 0
                    motor_msg.angular.z = -0.35 + 0.1*g.beacon_angle/90
                else:
                    motor_msg.angular.z = 0
                    if g.obstacle_distance > 0.2:
                        motor_msg.linear.x = min(0.35, g.obstacle_distance-0.1)
                    else:
                        motor_msg.linear.x = 0

            else:
                motor_msg.angular.z = 0.35
                motor_msg.linear.x = 0


            pub.publish(motor_msg)

        else:
            pass

        rate.sleep()




if __name__ == "__main__":
    try:
        move_to_beacon()
    except rospy.ROSInterruptException:
        pass
