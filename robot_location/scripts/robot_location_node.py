#!/usr/bin/env python

import rospy
import math
from serial_communication.msg import SensorData
from geometry_msgs.msg import TwistStamped

# pose estimate msg from the Vive
# stamped with a time so we know if it's an old reading or not
robot_location_msg = TwistStamped()
robot_location_msg.header.frame_id = 'vive'

class Vector:
    """
    Represents the vector from (x1, y1) -> (x2, y2)
    """
    def __init__(self, x1, y1, x2, y2):
        self.x = x2 - x1
        self.y = y2 - y1
    
    def norm(self):
        return math.sqrt(self.x**2 + self.y**2)
    
    def isPositive(self):
        """
        checks to see if the y value of the vector is positive
        """
        if self.y > 0:
            return True
        else:
            return False


def dot_product(vec1, vec2):
    return vec1.x * vec2.x + vec1.y * vec2.y

def sensor_data_callback(sensor_data):
    """
    When we receive sensor data from the sensor_data topic that contains the
    Vive coordinates, we need to compute the averaged position and also the 
    orientation. But only do it if the most recent reading was not 0.
    """
    if (sensor_data.x1 != 0 and sensor_data.x2 != 0 and sensor_data.y1 != 0 and sensor_data.y2 != 0):
        
        # set the timestamp of the msg to now
        robot_location_msg.header.stamp = rospy.Time.now()

        # compute the average x and y values
        avg_x = (sensor_data.x1 + sensor_data.x2)/2
        avg_y = (sensor_data.y1 + sensor_data.y2)/2

        robot_location_msg.twist.linear.x = avg_x
        robot_location_msg.twist.linear.y = avg_y

        # compute the orientation, with respect to the X axis of the Vive
        # coordinate system (long side of the arena is the X axis)
        vector = Vector(sensor_data.x1, sensor_data.y1, sensor_data.x2, sensor_data.y2)
        zero_vec = Vector(0, 0, 1, 0)
        dot_prod = dot_product(vector, zero_vec)
        theta = math.atan2(vector.y,vector.x)

        theta += math.pi

        theta = 2*math.pi-theta

        orientation = theta - math.pi/2
        if orientation < 0:
            orientation += 2*math.pi
        
        orientation = 2*math.pi - orientation
        robot_location_msg.twist.angular.z = orientation


def transmit_robot_location():
    rospy.init_node("robot_location_node", anonymous=True)

    rospy.Subscriber("sensor_data", SensorData, sensor_data_callback)

    vive_pose_pub = rospy.Publisher("vive_pose", TwistStamped, queue_size=10)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        vive_pose_pub.publish(robot_location_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        transmit_robot_location()
    except rospy.ROSInterruptException:
        pass
