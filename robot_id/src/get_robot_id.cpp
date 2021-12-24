#include <ros/ros.h>
#include <wiringPi.h>
#include "std_msgs/String.h"
#include <string>

// This node just reads the ID pins for the robot and then 
// publishes them to the robot_id topic as a string

int id_pin_1 = 10; // pin 19, GPIO 10
int id_pin_2 = 9; // pin 21, GPIO 9
int id_pin_3 = 11; // pin 23, GPIO 11
int id_pin_4 = 25; // pin 22, GPIO 25

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_id");
	ros::NodeHandle nh;
	ROS_INFO("robot_id node starting");
	ros::Publisher pub = nh.advertise<std_msgs::String>("robot_id", 1000);

	ros::Rate loop_rate(1);

	// Set up GPIO stuff to read values as input
	wiringPiSetupGpio();
	pinMode(id_pin_1, INPUT);
	pinMode(id_pin_2, INPUT);
	pinMode(id_pin_3, INPUT);
	pinMode(id_pin_4, INPUT);

	while (ros::ok()) {

		// read in the input values
		int pin1_val = digitalRead(id_pin_1);
		int pin2_val = digitalRead(id_pin_2);
		int pin3_val = digitalRead(id_pin_3);
		int pin4_val = digitalRead(id_pin_4);

		int robot_id = 0;

		// set robot id based on input pins
		if (pin1_val == 1) {
			robot_id = 1;
		} else if (pin2_val == 1) {
			robot_id = 2;
		} else if (pin3_val == 1) {
			robot_id = 3;
		} else if (pin4_val == 1) {
			robot_id = 4;
		}

		std_msgs::String msg;
	        msg.data=std::to_string(robot_id);

		pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();


	}

	return 0;
}
