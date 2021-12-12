#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <cmath>
#include "sensor_msgs/LaserScan.h"

class WallFollow {

	public:
		float DESIRED_DISTANCE_FROM_WALL =0.2; // meters
		int lower_angle = 0;
		int upper_angle = 60;
		int theta = 60;

		float Dt = 0;
		float L = 0.2;
		float alpha = 0;
		float Dt_future = 0;
		int steering_angle = 0;

		float error = 0;
		float prev_error = 0;
		int integral = 0;

		float Kp = 0.6;


		float stop_distance = 0.24;
		float min_valid_distance = 0.1;
		float max_angular_vel = 0.5;
		float default_linear_vel = 0.24;

		int ydlidar_scan_range = 448;

		int wall_following_on = 0;

		ros::Publisher* wall_follow_pub;

	void save_pub(ros::Publisher* pub_ptr) {
		wall_follow_pub = pub_ptr;
	}

	void wall_follow_step(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
		float dist_front = getAvgDistanceAtAngle(90,4, scan_msg);
	        float dist_front_left_1 = getAvgDistanceAtAngle(100,4, scan_msg);
       		float dist_front_left_2 = getAvgDistanceAtAngle(110,4, scan_msg);

	        geometry_msgs::Twist drive_msg;

		//ROS_INFO("Dist front: {%f}: ", dist_front);

	        // if we need to turn at a corner
	        if (dist_front < stop_distance && dist_front_left_1 < sqrt(pow(stop_distance,2)+pow(sin(10)*stop_distance,2)) && dist_front_left_2 < sqrt(pow(stop_distance,2))+pow(sin(20)*stop_distance,2)) {
        	        drive_msg.angular.z = -0.5;
                	wall_follow_pub->publish(drive_msg);
			ROS_INFO("Turning left! Dist front: %f", dist_front);
        	}
        	// if we are at a straight section of the wall
        	else {
                	float dist_b = getAvgDistanceAtAngle(lower_angle, 4, scan_msg);
			
	                float dist_a = getAvgDistanceAtAngle(upper_angle, 4, scan_msg);
	                Dt_future = calculateFutureDistance(dist_a, dist_b, theta);
	                ROS_INFO("Future distance: %f", Dt_future);
			float error = DESIRED_DISTANCE_FROM_WALL - Dt_future;
	                float angular_vel = p_control(error);
			drive_msg.linear.x = default_linear_vel;
	                drive_msg.angular.z = angular_vel;
        	        wall_follow_pub->publish(drive_msg);
			ROS_INFO("Straight wall, angular vel: {%f}", angular_vel);
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
		int ccw_angle = 360 - clockwise_angle;
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


int main(int argc, char **argv) {
	ros::init(argc, argv, "wall_follow");
	ros::NodeHandle nh;
	ROS_INFO("Starting up Wall Following node!");
	ros::Subscriber laser_scan_sub = nh.subscribe("scan", 100, scanCallback);
	ros::Publisher wall_follow_pub = nh.advertise<geometry_msgs::Twist>("motor_control", 100);
	wall_follow_obj.save_pub(&wall_follow_pub);

	ros::spin();
}
