#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from decision_making.msg import ModeActivation
from get_can_locations.msg import ObjectLocation

class MoveToCanLogic:
    def __init__(self):
        self.isActivated = False
        self.cans_on_field = {}
        self.is_lower_side = 0
        self.middle_y_val = 4680

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
        if move_to_can_logic.isActivated:

            target_can_id = determineClosestCanToCenter()

            rospy.loginfo("Target Can ID: %i", target_can_id)
            target_can_msg.object_id = target_can_id
            target_can_msg.twist.linear.x = move_to_can_logic.cans_on_field[target_can_id][0]
            target_can_msg.twist.linear.y = move_to_can_logic.cans_on_field[target_can_id][1]

            pub.publish(target_can_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        make_decision()
    except rospy.ROSInterruptException:
        pass
