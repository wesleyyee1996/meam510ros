#include <ros/ros.h>
#include "robot_control/MotorDrive.h"
#include <cmath>

class WallFollow {

	public:
		int DESIRED_DISTANCE_FROM_WALL = 200;
		int lower_angle = 0;
		int upper_angle = 60;

		float Dt = 0;
		float L = 200;
		float alpha = 0;
		float Dt_future = 0;
		int steering_angle = 0;

		int error = 0;
		int prev_error = 0;
		int integral = 0;

		float Kp = 0.3;

		int stop_distance = 300;


	void wall_follow_step(ros::Publisher &pub) {
		int dist_front = getAvgDistanceAtAngle(90,4);
		int dist_front_left_1 = getAvgDistanceAtAngle(100,4);
		int dist_front_left_2 = getAvgDistanceAtAngle(110,4);

		robot_control::MotorDrive motor_drive_msg;

		if (dist_front < stop_distance && dist_front_left_1 < sqrt(pow(stop_distance,2)+pow(sin(10)*stop_distance,2)) && dist_front_left_2 < sqrt(pow(stop_distance,2))+pow(sin(20)*stop_distance,2)) {
			motor_drive_msg.y_speed = 128;
			pub.publish(
		}

	}

	int pid_control(float error) {
		steering_angle = -Kp*error;

		if (steering_angle < 0) {
			steering_angle = max(-128, steering_angle);
		}
		else {
			steering_angle = max(128, steering_angle);
		}

		prev_error = error;
		return steering_angle;
	}

	int getAvgDistanceAtAngle(int angle, int samples) {

	}

	float calculateFutureDistance(float dist_a, float dist_b, float theta) {
		alpha = atan((dist_a*cos(theta/57.2958) - dist_b)/(dist_a*sin(theta/57.2958)));
		Dt = dist_b * cos(alpha);
		return Dt + L*sin(alpha);
	}	
};



int main(int argc, char **argv) {
	ros::init(argc, argv, "wall_follow");
	ros::NodeHandle nh;
	ROS_INFO("Starting up Wall Following node!");
	ros::Publisher wall_follow_pub = nh.advertise<robot_control::MotorDrive>("motor_control", 100);
	ros::Rate loop_rate(10);

	while (ros::ok()) {


	}
}
