#!/usr/bin/env python

import rospy

from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from decision_making.msg import ModeActivation
from std_msgs.msg import Bool

hostname = "192.168.0.171"
port = 8080

class ModeMsgClass:
    def __init__(self):
        self.mode_active_msg = ModeActivation()
        self.grabbed_can = False
mode = ModeMsgClass()


def build_webpage():
    body = "<html><body>"
    body += "<div><a href=\"/toggle_gripper?"
    if mode.mode_active_msg.gripper_closed:
        body += str(0)
    else:
        body += str(1)
    body += "\">Toggle Gripper</a></div>"
    body += "<div><a href=\"/wall_following?"
    if mode.mode_active_msg.wall_following_on:
        body += str(0)
    else:
        body += str(1)
    body += "\">Toggle Wall Following</a></div>"
    body += "<div><a href=\"/move_to_beacon?"
    if mode.mode_active_msg.move_to_beacon_on:
        body += str(0)
    else:
        body += str(1)
    body += "\">Toggle Move To Beacon</a></div>"
    body += "<div><a href=\"/move_to_vive_can?"
    if mode.mode_active_msg.move_to_vive_can_on:
        body += str(0)
    else:
        body += str(1)
    body += "\">Toggle Move to Vive Can</a></div>"
    body += "<div>Wall Following Status: "
    if mode.mode_active_msg.wall_following_on:
        body += "ON"
    else:
        body += "OFF"
    body += "</div>"
    body += "<div>Gripper Status: "
    if mode.mode_active_msg.gripper_closed:
        body += "ON" 
    else:
        body += "OFF"
    body += "<div>Go to beacon status: "
    if mode.mode_active_msg.move_to_beacon_on:
        body += "ON"
    else:
        body += "OFF"
    body += "<div>Go to Vive Can status: "
    if mode.mode_active_msg.move_to_vive_can_on:
        body += "ON"
    else:
        body += "OFF"
    body += "</div>"
    body += "<div>Has grabbed can?: "
    if mode.grabbed_can:
        body += "YES"
    else:
        body += "NO"
    body += "</div>"
    body += "</body></html>"
    return body

class MyServer(BaseHTTPRequestHandler):

    def do_GET(self):
        if '/toggle_gripper' in self.path:
            is_on = int(self.path[-1])
            print("Gripper: ", is_on)
            if is_on == 1:
                print("Setting gripper to true")
                mode.mode_active_msg.gripper_closed= True
            else:
                mode.mode_active_msg.gripper_closed = False
        elif '/wall_following' in self.path:
            is_on = int(self.path[-1])
            if is_on == 1:
                mode.mode_active_msg.wall_following_on = True
                mode.mode_active_msg.move_to_beacon_on = False
                mode.mode_active_msg.move_to_vive_can_on = False
            else:
                mode.mode_active_msg.wall_following_on = False
        elif '/move_to_beacon' in self.path:
            is_on = int(self.path[-1])
            if is_on == 1:
                mode.mode_active_msg.move_to_beacon_on = True
                mode.mode_active_msg.wall_following_on = False
                mode.mode_active_msg.move_to_vive_can_on = False
            else:
                mode.mode_active_msg.move_to_beacon_on = False
        elif '/move_to_vive_can' in self.path:
            is_on = int(self.path[-1])
            if is_on == 1:
                mode.mode_active_msg.move_to_vive_can_on = True
                mode.mode_active_msg.wall_following_on = False
                mode.mode_active_msg.move_to_beacon_on = False

        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(build_webpage())
        mode_pub.publish(mode.mode_active_msg)

def grabbedCanCallback(msg):
    mode.grabbed_can = msg.data

def run_web_server():
    rospy.init_node("website_node", anonymous=True)

    global mode_pub
    mode_pub = rospy.Publisher("mode_activation", ModeActivation, queue_size=1)
    gripper_sub = rospy.Subscriber("has_grabbed_can", Bool, grabbedCanCallback)

    webServer = HTTPServer((hostname, port), MyServer)

    rospy.loginfo("Server started!")
    try:
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("test")
        rate.sleep()

    webServer.server_close()

if __name__ == '__main__':
    try:
        run_web_server()
    except rospy.ROSInterruptException:
        pass
