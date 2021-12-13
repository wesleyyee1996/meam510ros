#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Global:
# create variable to save our robot(vive)  and cans coordinate value
    def __init__(self):
        self.vive_x = 0
        self.vive_y = 0
        self.vive_angle = 0
        self.can_x = 0
        self.can_y = 0
        self.last_update_time = 0
        self.obstacle_distance = 0
        self.stop_turning = False

    def has_data(self):
        if self.vive_x != 0 and self.vive_y != 0 and self.can_x != 0 and self.can_y != 0:
            return True
        return False

g = Global()

class Vector:
    """
    Represents the vector from (x1, y1) -> (x2, y2)
    """

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.x2 = x2
        self.x = x2 -x1
        self.y = y2 -y1

    def norm(self):
        # calculate the distance between two point
        return math.sqrt(self.x**2 +self.y**2)

    def isPositive(self):
        # checks to see if the y value of the vector is positive
        if self.y > 0:
            return True
        else:
            return False

    def angle(self):
        # calculate the angl under the world coordinate between two points
        angle = math.acos((self.x2-self.x1)/self.norm());
        if self.isPositive():
            return angle
        else:
            return 2*math.pi-angle


def save_vive_data(vive_data):
    # record the vive data each time when it recieved
    g.vive_x = vive_data.twist.linear.x
    g.vive_y = vive_data.twist.linear.y
    g.vive_angle = vive_data.twist.angular.z
    g.last_update_time = vive_data.header.stamp.to_sec()


def save_can_data(can_data):
    # record the can data each time when it recieved 
    g.can_x = can_data.twist.linear.x
    g.can_y = can_data.twist.linear.y


def save_obstacle_distance(scan_data):
    # record the obstacle distance stright infront of th robote
    for i in range( 107, 118):
        g.obstacle_distance += scan_data.ranges[i]/11



def driving_direction():
    # used to calculate the direction from robot to the can position
    vector = Vector(g.vive_x, g.vive_y, g.can_x, g.can_y)
    distance = vector.norm()
    angle = vector.angle()
    return [distance,angle]

def listener():

    rospy.init_node("move_to_vive_can", anonymous = True)
    pub = rospy.Publisher("motor_control", Twist, queue_size = 10)
    rospy.Subscriber("vive_pose", TwistStamped, save_vive_data)
    rospy.Subscriber("can_location", TwistStamped, save_can_data)
    rospy.Subscriber("scan", LaserScan, save_obstacle_distance)

    rate = rospy.Rate(10)

    turn_on_motors = False

    while not rospy.is_shutdown():
        
        if g.has_data():

            [distance,angle] = driving_direction();

            curr_time = rospy.Time().to_sec()

            motor_msg = Twist()

            #print("Angle ", angle)
            angle_float = float(angle)
            vive_float = float(g.vive_angle)
            angle_diff = angle_float - vive_float
            if angle_diff > 0.3:
                motor_msg.linear.x = 0
                motor_msg.angular.z = min(1.5*angle_diff, 0.4)
            elif angle_diff < -0.3:
                motor_msg.linear.x = 0
                motor_msg.angular.z = max(1.5*angle_diff, -0.4)
            else:
                if angle_diff > 0:
                    motor_msg.angular.z = min(1.5*angle_diff, 0.4)
                else:
                    motor_msg.angular.z = max(1.5*angle_diff, -0.4)
                if distance > 0.1:
                    motor_msg.linear.x = 0.45
                else:
                    motor_msg.linear.x = 0
                    motor_msg.angular.z = 0

            #if (curr_time - prev_time > 0.1):
            if (curr_time - g.last_update_time  > 0.3):
                turn_on_motors = not turn_on_motors
                prev_time = curr_time

            #if (turn_on_motors):
            #motor_msg.angular.z = 0

            pub.publish(motor_msg)
        else:
            #print("Hasn't gotten good data")
            pass

        rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

