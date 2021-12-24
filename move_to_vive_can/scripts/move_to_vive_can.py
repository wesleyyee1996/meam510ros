#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from get_can_locations.msg import ObjectLocation
from std_msgs.msg import Bool
from decision_making.msg import ModeActivation

class Global:
# create variable to save our robot(vive)  and cans coordinate value
    def __init__(self):
        self.vive_x = 0
        self.vive_y = 0
        self.vive_angle = 0
        self.can_x = 0
        self.can_y = 0
        self.can_id = 0
        self.last_update_time = 0
        self.obstacle_distance = 0
        self.stop_turning = False
        self.isActivated = False

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
        angle = math.atan2(self.y,-self.x);
        angle += math.pi
        angle = 2*math.pi - angle

        return angle

def save_vive_data(vive_data):
    # record the vive data each time when it recieved
    g.vive_x = vive_data.twist.linear.x
    g.vive_y = vive_data.twist.linear.y
    g.vive_angle = vive_data.twist.angular.z
    g.last_update_time = vive_data.header.stamp.to_sec()


def save_can_data(can_data):
    # record the can data each time when it recieved 
    print(can_data)
    #if not g.has_data():
    g.can_x = can_data.twist.linear.x
    g.can_y = can_data.twist.linear.y
    g.can_id = can_data.object_id
    #elif g.can_id == can_data.object_id:
    #    g.can_x = can_data.twist.linear.x
    #    g.can_y = can_data.twist.linear.y

def save_obstacle_distance(scan_data):
    # record the obstacle distance stright infront of th robote
    avg = 0
    cnt = 0

    for i in range(110, 118):
        if scan_data.ranges[i] > 0:
            avg += scan_data.ranges[i]
            cnt += 1

    if cnt == 0:
        avg = 0
    else:
        avg /= cnt
            
    g.obstacle_distance = avg



def driving_direction():
    # used to calculate the direction from robot to the can position
    vector = Vector(g.vive_x, g.vive_y, g.can_x, g.can_y)
    distance = vector.norm()
    angle = vector.angle()
    return [distance,angle]

def mode_activation_callback(msg):
    g.isActivated = msg.move_to_vive_can_on

def listener():
    """
    This node moves the robot to the vive can. 
    """

    rospy.init_node("move_to_vive_can", anonymous = True)
    pub = rospy.Publisher("motor_control", Twist, queue_size = 1)
    actuate_gripper_pub = rospy.Publisher("actuate_gripper", Bool, queue_size=1)
    rospy.Subscriber("vive_pose", TwistStamped, save_vive_data)
    rospy.Subscriber("target_can_location", ObjectLocation, save_can_data)
    rospy.Subscriber("scan", LaserScan, save_obstacle_distance)
    rospy.Subscriber("mode_activation", ModeActivation, mode_activation_callback)

    rate = rospy.Rate(50)

    turn_on_motors = False

    straight_shot = False

    while not rospy.is_shutdown():
        
        # first make sure that this node has been activated
        if g.isActivated:
            if g.has_data():

                # calculate the desired orientation to turn to based on the can's location as well 
                # as our location and current orientation. also calculate can's distance from robot
                [distance,angle] = driving_direction()

                curr_time = rospy.Time().to_sec()

                motor_msg = Twist()

                angle_float = float(angle) 
                vive_float = float(g.vive_angle)
                angle_diff = angle_float - vive_float

                # if we're over 1000 vive units away from the can, then keep turn left if our current angle
                # and desired angle is greater than or less than 0.15 radians. Otherwise, move forward
                if distance > 1000:
                    print("outside of radius")
                    if angle_diff > 0.15 and distance > 100:
                        motor_msg.linear.x = 0
                        motor_msg.angular.z = 0.15+0.02*angle_diff/math.pi
                    elif angle_diff < -0.15 and distance > 100:
                        motor_msg.linear.x = 0
                        motor_msg.angular.z = -0.15+0.02*angle_diff/math.pi
                    else:
                        motor_msg.angular.z = 0
                        
                        motor_msg.linear.x = min(0.2, (distance-100)*0.001)
                        # we have reached the can, so grab the gripper 

                # if we're less than 1000 vive units away from the can, then move straight to the can
                else:
                    if angle_diff > 0.1 and not straight_shot:
                        motor_msg.linear.x = 0
                        motor_msg.angular.z = 0.15+0.01*angle_diff/math.pi
                    elif angle_diff < -0.1 and not straight_shot:
                        motor_msg.linear.x = 0
                        motor_msg.angular.z = -0.15+0.01*angle_diff/math.pi
                    else:

                        # now it's right in front. we want to set straight shot to be true since if we're too
                        # close to the can we don't want to accidentally keep turning and knock over the can
                        print("Straight shot")
                        straight_shot = True
                        if g.obstacle_distance > 0.2:
                            motor_msg.linear.x = 0.11
                        else:

                            # once we've reached the can, stop and then instruct the gripper to close
                            motor_msg.linear.x = 0
                            grab_can_msg = Bool()
                            grab_can_msg.data = True
                            actuate_gripper_pub.publish(grab_can_msg)
                print("Lidar dist: ", g.obstacle_distance, ", Vive dist: ", distance, ", Vive angle: ", angle)
                pub.publish(motor_msg)
                
            else:
                pass

        rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

