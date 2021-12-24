#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "decision_making/ModeActivation.h"

// Node for doing wall following

class WallFollow {

	public:
		float DESIRED_DISTANCE_FROM_WALL =0.2; // meters
		int lower_angle = 0;
		int upper_angle = 60;
		int theta = 60;

		float Dt = 0; // estimated distance in the future at distance L based on current angle alpha
		float L = 0.2; // lookahead distance
		float alpha = 0; // current angle from the wall
		float Dt_future = 0;
		int steering_angle = 0; // desired steering angle for error correction

		float error = 0;
		float prev_error = 0;
		int integral = 0;

		float Kp = 0.6; // Kp


		float stop_distance = 0.4; // stop distance from the wall for us to turn left
		float min_valid_distance = 0.1; // minimum valid distance that we read from the lidar
		float max_angular_vel = 0.15; // the maximum angular velocity that we will tell the robot to turn
		float default_linear_vel = 0.2; // default linear velocity to go forward

		int ydlidar_scan_range = 448; // size of the lidar scan array

		int wall_following_on = 0; 

		ros::Publisher* wall_follow_pub;

		bool isActivated = false;

	
	void save_pub(ros::Publisher* pub_ptr) {
		wall_follow_pub = pub_ptr;
	}


	void wall_follow_step(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

		// first check to see if we have been activated or not
		if (isActivated) {
		
			float dist_front = getAvgDistanceAtAngle(90,4, scan_msg);
			float dist_front_left_1 = getAvgDistanceAtAngle(100,4, scan_msg);
			float dist_front_left_2 = getAvgDistanceAtAngle(110,4, scan_msg);

			geometry_msgs::Twist drive_msg;

			// if we need to turn at a corner
			if (dist_front < stop_distance && dist_front_left_1 < sqrt(pow(stop_distance,2)+pow(sin(10)*stop_distance,2)) && dist_front_left_2 < sqrt(pow(stop_distance,2))+pow(sin(20)*stop_distance,2)) {
				drive_msg.angular.z = -0.15;
				wall_follow_pub->publish(drive_msg);
				ROS_INFO("Turning left! Dist front: %f", dist_front);
			}
			// if we are at a straight section of the wall
			else {
				float dist_b = getAvgDistanceAtAngle(lower_angle, 4, scan_msg);
				
				float dist_a = getAvgDistanceAtAngle(upper_angle, 4, scan_msg);
				Dt_future = calculateFutureDistance(dist_a, dist_b, theta);
				float error = DESIRED_DISTANCE_FROM_WALL - Dt_future;
				float angular_vel = p_control(error);
				drive_msg.linear.x = default_linear_vel;
				drive_msg.angular.z = angular_vel;
				wall_follow_pub->publish(drive_msg);
			}
		}
	}

	// proportional control
	float p_control(float error) {
		float angular_vel = -Kp*error;

		if (angular_vel < 0) {
			angular_vel = std::max(-max_angular_vel, angular_vel);
		}
		else {
			angular_vel = std::min(max_angular_vel, angular_vel);
		}

		prev_error = error;
		return angular_vel;
	}

	// convert clockwise angle to counter clockwise angle
	// since the lidar scan starts at 0 degrees from x axis
	// and goes clockwise. want to get corresponding angle in
	// counter clockwise direction.
	// also multiply by a scaling factor since the scan range is
	// about 450 instead of 360
	int getScanIdx(int clockwise_angle) {
		float scaling_factor = ydlidar_scan_range/360; 
		int ccw_angle = clockwise_angle;
		int scan_idx = ccw_angle * scaling_factor;
		return scan_idx;
	}

	float getAvgDistanceAtAngle(int angle, int samples, const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
		float avg = 0;
		int start_idx = angle - floor(samples/2);
		int end_idx = angle + floor(samples/2);
		int cnt = 0;
		float curr_data = 0;
		for (int i = start_idx; i < end_idx; i++) {
			
			int idx = getScanIdx(i);
			if (i < 0) {
				curr_data = scan_msg->ranges[idx];
			}
			else {
				curr_data = scan_msg->ranges[idx];
			}
			if (curr_data > min_valid_distance) {
				avg += curr_data;
				cnt++;
			}
		}
		if (cnt == 0) {
			return 0;
		}
		//ROS_INFO("start_idx %i, end_idx %i, avg %f, cnt %i", start_idx, end_idx, avg, cnt);
		return avg/cnt;

	}

	float calculateFutureDistance(float dist_a, float dist_b, float theta) {
		alpha = atan((dist_a*cos(theta/57.2958) - dist_b)/(dist_a*sin(theta/57.2958)));
		Dt = dist_b * cos(alpha);
		return Dt + L*sin(alpha);
	}	
};

WallFollow wall_follow_obj;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
	wall_follow_obj.wall_follow_step(scan_msg);
}

void modeActivationCallback(const decision_making::ModeActivation::ConstPtr& msg) {
	wall_follow_obj.isActivated = msg->wall_following_on;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "wall_follow");
	ros::NodeHandle nh;
	ROS_INFO("Starting up Wall Following node!");
	ros::Subscriber laser_scan_sub = nh.subscribe("scan", 100, scanCallback);
	ros::Publisher wall_follow_pub = nh.advertise<geometry_msgs::Twist>("motor_control", 100);
	ros::Subscriber mode_active_sub = nh.subscribe("mode_activation", 1, modeActivationCallback);
	wall_follow_obj.save_pub(&wall_follow_pub);

	ros::spin();
}
