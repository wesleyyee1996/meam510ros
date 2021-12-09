#include <ros/ros.h>
#include <wiringPi.h>
#include "std_msgs/String.h"
#include <string>

int id_pin_1 = 10; // pin 19
int id_pin_2 = 9; // pin 21
int id_pin_3 = 11; // pin 23
int id_pin_4 = 25; // pin 22

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_id");
	ros::NodeHandle nh;
	ROS_INFO("robot_id node starting");
	ros::Publisher pub = nh.advertise<std_msgs::String>("robot_id", 1000);

	ros::Rate loop_rate(10);

	wiringPiSetupGpio();
	pinMode(id_pin_1, INPUT);
	pinMode(id_pin_2, INPUT);
	pinMode(id_pin_3, INPUT);
	pinMode(id_pin_4, INPUT);

	while (ros::ok()) {
		int pin1_val = digitalRead(id_pin_1);
		int pin2_val = digitalRead(id_pin_2);
		int pin3_val = digitalRead(id_pin_3);
		int pin4_val = digitalRead(id_pin_4);

		int robot_id = pin1_val*8 + pin2_val*4 + pin3_val*2 + pin4_val*1;

		std_msgs::String msg;
	        msg.data=std::to_string(robot_id);

		pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();


	}

	return 0;
}
