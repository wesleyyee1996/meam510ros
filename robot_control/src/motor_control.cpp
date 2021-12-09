
#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "robot_control/MotorDrive.h"

class MotorControl {
	public:
float led_freq = 50; // frequency for driving the motors
    int led_channel_1 = 1; // PWM channel set to 1 for motor 1, and 2 for motor 2
    int led_channel_2 = 2;
    int MAX_DUTY = 255;
    int pwm_1 = 12; // pwm pin for motor 1, pin 32
    int pwm_2 = 13; // pwm pin for motor 2, pin 33 
    int motor_drive_1a = 27; // motor drive 1, pin 13
    int motor_drive_1b = 22; // pin 15
    int motor_drive_2a = 23; // motor drive 2, pin 16
    int motor_drive_2b = 24; // pin 18
    int pwm_resolution = 8; // 8-bit resolution, which is 255
    int steering_angle = 0;
    int left_out = 0;
    int right_out = 0;

    MotorControl() {
	    wiringPiSetupGpio();
	    softPwmCreate(pwm_1, 0, MAX_DUTY);
	    softPwmCreate(pwm_2, 0, MAX_DUTY);
	    pinMode(motor_drive_1a, OUTPUT);
	    pinMode(motor_drive_2a, OUTPUT);
	    pinMode(motor_drive_1b, OUTPUT);
	    pinMode(motor_drive_2b, OUTPUT);
    }

void setMotor1_Forward() {
  digitalWrite(motor_drive_1a, HIGH);
  digitalWrite(motor_drive_1b, LOW);
}

void setMotor2_Forward() {
  digitalWrite(motor_drive_2a, HIGH);
  digitalWrite(motor_drive_2b, LOW);
}

void setMotor1_Backward() {
  digitalWrite(motor_drive_1a, LOW);
  digitalWrite(motor_drive_1b, HIGH);
}

void setMotor2_Backward() {
  digitalWrite(motor_drive_2a, LOW);
  digitalWrite(motor_drive_2b, HIGH);
}

// handle left motor, controls are using arcade drive
void handle_left(int y_val, int steering_angle) {
  left_out = (y_val + steering_angle);

  if (left_out < 0) {
    setMotor1_Backward();
  }
  else {
    setMotor1_Forward();
  }
  softPwmWrite(pwm_1, left_out);
}

// handle right motor, controls are using arcade drive
void handle_right(int y_val, int steering_angle) {
  right_out = (y_val - steering_angle);

  if (right_out < 0) {
    setMotor2_Backward();
  }
  else {
    setMotor2_Forward();
  }
  softPwmWrite(pwm_2, right_out);
}
};

MotorControl motor_control_obj;

void motorControlCallback(const robot_control::MotorDrive::ConstPtr& msg) {
	motor_control_obj.handle_left(msg->y_speed, msg->x_speed);
	motor_control_obj.handle_right(msg->y_speed, msg->x_speed);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_control");
	ros::NodeHandle nh;
	ROS_INFO("Hello!");
	ros::Subscriber sub = nh.subscribe("motor_control", 1000, motorControlCallback);
	ros::spin();
      	return 0;
}



