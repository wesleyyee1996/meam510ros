#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from serial_communication.msg import SensorData
from decision_making.msg import ModeActivation
from std_msgs.msg import Bool

class Global:
    # create variable to save listened info
    def __init__(self):
        #self.robot_angle = 600
        self.beacon_angle = 600
        self.beacon_sensing = 600
        self.obstacle_distance = 0
        self.isActivated = False

    def has_data(self):
        if self.beacon_angle != 600: 
            return True
        return False

g = Global()


def save_beacon_info(sensor_data):
    # record beacon info
    g.beacon_angle = sensor_data.beacon_angle
    g.beacon_sensing = sensor_data.beacon_stalled


def save_obstacle_distance(scan_data):
    # record the obstacle distance stright infront of the robot
    avg = 0
    cnt = 0
    for i in range (110,118):
        if scan_data.ranges[i] > 0:
            avg += scan_data.ranges[i]
            cnt += 1

    if cnt == 0:
        avg = 0
    else:
        avg /= cnt
    g.obstacle_distance = avg

def modeActivationCallback(msg):
    g.isActivated = msg.move_to_beacon_on

def move_to_beacon():
    """
    Functionality for moving to the beacon. We receive messages from the sensor_data topic which contains
    whether beacon is on the left(1), right(2), or is in front(3). If it's to the left, then turn the robot
    to the left. If it's to the right, then turn the robot to the right. If it's to the front, then go straight.
    """

    rospy.init_node("move_to_beacon", anonymous = True)
    pub = rospy.Publisher("motor_control", Twist, queue_size = 1)
    gripper_pub = rospy.Publisher("actuate_gripper", Bool, queue_size = 1)
    rospy.Subscriber("sensor_data", SensorData, save_beacon_info)
    rospy.Subscriber("scan", LaserScan, save_obstacle_distance)
    rospy.Subscriber("mode_activation", ModeActivation, modeActivationCallback)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        
        # check to make sure that this node has been activated
        if g.isActivated:
            if g.has_data():

                # initialize the velocity message to send to the motors
                motor_msg = Twist()

                # if we have detected the beacon, then spin left, right, or go
                # straight depending on where it is
                if g.beacon_angle != 0:
                    if g.beacon_angle == 1:
                        motor_msg.linear.x = 0
                        motor_msg.angular.z = 0.1
                    elif g.beacon_angle == 2:
                        motor_msg.linear.x = 0
                        motor_msg.angular.z = -0.1
                    else:
                        motor_msg.angular.z = 0

                        # if we see that the lidar reading says it's more than 0.2 meters 
                        # in front, then we move at a rate proportional to how far away
                        # we are from it
                        if g.obstacle_distance > 0.2:
                            motor_msg.linear.x = min(0.2, g.obstacle_distance-0.05)

                        # if we're closer than 0.2 meters from it, then stop and tell the gripper
                        # to close
                        else:
                            motor_msg.linear.x = 0
                            actuate_gripper_msg = Bool()
                            actuate_gripper_msg.data = True
                            gripper_pub.publish(actuate_gripper_msg)

                # if the beacon_angle is equal to 0, then we haven't detected it. in that
                # case, then we just keep spinning
                else:
                    motor_msg.angular.z = 0.09
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
