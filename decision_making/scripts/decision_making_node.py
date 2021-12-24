#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from decision_making.msg import ModeActivation
from get_can_locations.msg import ObjectLocation

# class for holding logic 
class MoveToCanLogic:
    def __init__(self):

        # check to see if we want this node to be activated
        self.isActivated = False

        # hashmap which maps can ID to can positions
        self.cans_on_field = {}

        # flag which tells us if on red or blue side of map
        self.is_lower_side = 0
        self.middle_y_val = 4680

# create instance of movetocanlogic obj
move_to_can_logic = MoveToCanLogic()

def robotSideCallback(robot_side_data):
    """
    Update which side of the field the robot is on
    """
    move_to_can_logic.is_lower_side = robot_side_data.data


def objectLocationCallback(object_data):
    """
    Update object locations in object location dictionary
    """
    if (move_to_can_logic.is_lower_side and object_data.twist.linear.y < move_to_can_logic.middle_y_val)or (not move_to_can_logic.is_lower_side and object_data.twist.linear.y > move_to_can_logic.middle_y_val):
        move_to_can_logic.cans_on_field[object_data.object_id] = (object_data.twist.linear.x, object_data.twist.linear.y)

def modeActivationCallback(mode_activation_msg):
    move_to_can_logic.isActivated = mode_activation_msg.move_to_vive_can_on

def determineClosestCanToCenter():
    """
    Determine which can is the closest to the center. This is the one which we will go for
    """
    min_can_id = -1
    for can_id, can_data in move_to_can_logic.cans_on_field.items():
        if min_can_id == -1:
            min_can_id = can_id
        else:
            if move_to_can_logic.is_lower_side and can_data[1] < move_to_can_logic.cans_on_field[min_can_id][1]:
                min_can_id = can_id
            elif not move_to_can_logic.is_lower_side and can_data[1] > move_to_can_logic.cans_on_field[min_can_id][1]:
                min_can_id = can_id

    return min_can_id


def make_decision():
    rospy.init_node("make_decisions", anonymous=True)
    rate = rospy.Rate(5)

    pub = rospy.Publisher("target_can_location", ObjectLocation, queue_size=1)

    rospy.Subscriber("can_location", ObjectLocation, objectLocationCallback)
    rospy.Subscriber("robot_side", Bool, robotSideCallback)
    rospy.Subscriber("mode_activations", ModeActivation, modeActivationCallback)

    target_can_msg = ObjectLocation()

    while not rospy.is_shutdown():

        # if this node has been activated, then do stuff
        if move_to_can_logic.isActivated:

            # calc which can we want to go for
            target_can_id = determineClosestCanToCenter()

            rospy.loginfo("Target Can ID: %i", target_can_id)
            target_can_msg.object_id = target_can_id
            target_can_msg.twist.linear.x = move_to_can_logic.cans_on_field[target_can_id][0]
            target_can_msg.twist.linear.y = move_to_can_logic.cans_on_field[target_can_id][1]

            # publish the target can as a geometry_msgs/Twist msg to the target_can_location topic
            pub.publish(target_can_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        make_decision()
    except rospy.ROSInterruptException:
        pass
