#!/usr/bin/env python

import rospy
import socket

from get_can_locations.msg import ObjectLocation

def get_can_loc():
    """
    Node to receive the can locations from the broadcasted UDP IP address at port 1510
    """

    # bind to the broadcast IP on port 1510
    can_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    can_socket.bind(("192.168.1.255", 1510))

    # setup publisher to can_location topic
    can_pub = rospy.Publisher('can_location', ObjectLocation, queue_size=10)

    rospy.init_node('get_can_location', anonymous=True)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        try:

            # process the can data messages and publish to the can_location topic
            can_data, addr = can_socket.recvfrom(1024)
            i = 0
            while i < len(can_data) and can_data[i] != ':':
                i += 1
            can_id = int(can_data[:(i)])
            can_data = can_data[(i+1):]
            can_data = can_data.split(',')
            can_msg = ObjectLocation()
            can_msg.object_id = can_id
            can_msg.twist.linear.x = float(can_data[0])
            can_msg.twist.linear.y = float(can_data[1])
            can_pub.publish(can_msg)
        except socket.timeout:
            rospy.logerr("Can data timed out! Reconnecting")
        rate.sleep()

if __name__ == '__main__':
    try:
        get_can_loc()
    except rospy.ROSInterruptException:
        pass
