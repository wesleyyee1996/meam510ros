#include <ros/ros.h>
#include <wiringPi.h>
#include <std_msgs/Bool.h>

// This node checks to see if we have received a UDP command messages from the computer
// If we haven't received any 5 seconds or more, then we turn on the green LED

ros::Time last_update_time;

void ledSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data == true) {
		last_update_time = ros::Time::now();		
	}
}

int main(int argc, char **argv) {

	// setup GPIO pins
	wiringPiSetupGpio();

	// setup led signal pin to GPIO 21
	int led_signal_pin = 21; // pin 40

	pinMode(led_signal_pin, OUTPUT);

	ros::init(argc, argv, "led_no_input_signal");
	ros::NodeHandle nh;
	ROS_INFO("LED No Input Signal node starting up!");
	ros::Subscriber sub = nh.subscribe("led_signal", 1, ledSignalCallback);

	last_update_time = ros::Time::now();

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		
		ros::Duration elapsed_time = ros::Time::now() - last_update_time;
		// if we haven't received any command data from computer in 5 seconds, then turn on LED
		if (elapsed_time.toSec() > 5.0) {
			digitalWrite(led_signal_pin, HIGH);
		}
		// otherwise turn it off
		else {
			digitalWrite(led_signal_pin, LOW);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	digitalWrite(led_signal_pin, LOW);
	return 0;

}
