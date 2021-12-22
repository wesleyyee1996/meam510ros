#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <std_msgs/Bool.h>
#include <decision_making/ModeActivation.h>

#define GRIPPER_PWM_PIN 26 // GPIO 26, pin 37
#define MAX_DUTY 255
#define CLOSE_POSITION_PWM 120/4095*MAX_DUTY  
#define OPEN_POSITION_PWM 100/4095*MAX_DUTY 
#define HAS_GRABBED_CAN_PIN 16 // GPIO 16, pin 36


bool close_gripper = false;
bool isActivated = false;

void actuateGripperCallback(const std_msgs::Bool::ConstPtr& msg) {	
	close_gripper = msg->data;
	
}

void modeActivationCallback(const decision_making::ModeActivation::ConstPtr& mode_active_msg) {
	isActivated = mode_active_msg->gripper_closed;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "actuate_gripper");
	ros::NodeHandle nh;
	ROS_INFO("Actuate gripper node starting up!");

	ros::Subscriber sub = nh.subscribe("actuate_gripper", 1, actuateGripperCallback);
	ros::Publisher has_grabbed_can_pub = nh.advertise<std_msgs::Bool>("has_grabbed_can", 1);
	
	ros::Subscriber mode_active_sub = nh.subscribe("mode_activation", 1, modeActivationCallback);
	wiringPiSetupGpio();

	// setup PWM 
	softPwmCreate(GRIPPER_PWM_PIN, 0, MAX_DUTY);

	// setup grabbed can pin
	pinMode(HAS_GRABBED_CAN_PIN, INPUT);

	std_msgs::Bool has_grabbed_can_msg;
	ros::Rate loop_rate(50);
	while (ros::ok()) {
		if (isActivated) {
			softPwmWrite(GRIPPER_PWM_PIN, 12.5);
			//ROS_INFO("Gripper closing!");
			//std::cout << CLOSE_POSITION_PWM << std::endl;

			if (!digitalRead(HAS_GRABBED_CAN_PIN)){
				has_grabbed_can_msg.data = true;
			}

		}
		else {
			softPwmWrite(GRIPPER_PWM_PIN, 7.5);
			//ROS_INFO("Gripper opening!");
			//std::cout << OPEN_POSITION_PWM << std::endl;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
		
	return 0;
}
