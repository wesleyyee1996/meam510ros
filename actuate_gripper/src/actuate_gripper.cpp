#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <std_msgs/Int8.h>

#define GRIPPER_PWM_PIN 20 // GPIO 20, pin 38
#define MAX_DUTY 255

void actuateGripperCallback(const std_msgs::Int8::ConstPtr& msg) {
	
}

in main(int argc, char **argv) {
	ros::init(argc, argv, "actuate_gripper");
	ros::NodeHandle nh;
	ROS_INFO("Actuate gripper node starting up!");

	ros::Subscriber sub = nh.subscribe("actuate_gripper", 1, actuateGripperCallback);
	wiringPiSetupGpio();

	// setup PWM 
	softPwmCreate(GRIPPER_PWM_PIN, 0, MAX_DUTY);

	ros::Rate loop_rate(20);
	ros::spin();
		
	return 0;
}
