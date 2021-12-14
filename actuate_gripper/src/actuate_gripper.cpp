#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <std_msgs/Bool.h>

#define GRIPPER_PWM_PIN 20 // GPIO 20, pin 38
#define MAX_DUTY 255
#define CLOSE_POSITION_PWM .052*MAX_DUTY  
#define OPEN_POSITION_PWM .082*MAX_DUTY 
#define HAS_GRABBED_CAN_PIN 16 // GPIO 16, pin 36


bool close_gripper = false;

void actuateGripperCallback(const std_msgs::Bool::ConstPtr& msg) {	
	close_gripper = msg->data;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "actuate_gripper");
	ros::NodeHandle nh;
	ROS_INFO("Actuate gripper node starting up!");

	ros::Subscriber sub = nh.subscribe("actuate_gripper", 1, actuateGripperCallback);
	ros::Publisher has_grabbed_can_pub = nh.advertise<std_msgs::Bool>("has_grabbed_can", 1);
	
	wiringPiSetupGpio();

	// setup PWM 
	softPwmCreate(GRIPPER_PWM_PIN, 0, MAX_DUTY);

	// setup grabbed can pin
	pinMode(HAS_GRABBED_CAN_PIN, INPUT);

	std_msgs::Bool has_grabbed_can_msg;
	ros::Rate loop_rate(50);
	while (ros::ok()) {
		if (close_gripper) {
			softPwmWrite(GRIPPER_PWM_PIN, CLOSE_POSITION_PWM);
			ROS_INFO("Gripper closing!");

			if (!digitalRead(HAS_GRABBED_CAN_PIN)){
				has_grabbed_can_msg.data = true;
			}

		}
		else {
			softPwmWrite(GRIPPER_PWM_PIN, OPEN_POSITION_PWM);
			ROS_INFO("Gripper opening!");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
		
	return 0;
}
