
#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <geometry_msgs/Twist.h>

// This node is the driver for the motors. It takes in desired velocity messages and translates 
// them into motor pwm signals. It doesn't use PID control though since we didn't have encoders

class MotorControl {
	public:
float led_freq = 50; // frequency for driving the motors
    int led_channel_1 = 1; // PWM channel set to 1 for motor 1, and 2 for motor 2
    int led_channel_2 = 2;
    int MAX_DUTY = 255;
    int pwm_2 = 12; // pwm pin for motor 1, pin 32
    int pwm_1 = 13; // pwm pin for motor 2, pin 33 
    int motor_drive_2a = 27; // motor drive 1, pin 13
    int motor_drive_2b = 22; // pin 15
    int motor_drive_1a = 23; // motor drive 2, pin 16
    int motor_drive_1b = 24; // pin 18
    int pwm_resolution = 8; // 8-bit resolution, which is 255
    int steering_angle = 0;

    float vel_l;
    float vel_r;

    ros::Time last_update_time;

    MotorControl() {
      // Constructor for the motor control object. Initializes all of the GPIO stuff
	    wiringPiSetupGpio();
	    softPwmCreate(pwm_1, 0, MAX_DUTY);
	    softPwmCreate(pwm_2, 0, MAX_DUTY);
	    pinMode(motor_drive_1a, OUTPUT);
	    pinMode(motor_drive_2a, OUTPUT);
	    pinMode(motor_drive_1b, OUTPUT);
	    pinMode(motor_drive_2b, OUTPUT);
    }

// Sets the left motor to go forward
void setMotor1_Forward() {
  digitalWrite(motor_drive_1a, HIGH);
  digitalWrite(motor_drive_1b, LOW);
}

// Sets the right motor to go forward
void setMotor2_Forward() {
  digitalWrite(motor_drive_2a, HIGH);
  digitalWrite(motor_drive_2b, LOW);
}

// Sets the left motor to go backward
void setMotor1_Backward() {
  digitalWrite(motor_drive_1a, LOW);
  digitalWrite(motor_drive_1b, HIGH);
}

// Sets the right motor to go backward
void setMotor2_Backward() {
  digitalWrite(motor_drive_2a, LOW);
  digitalWrite(motor_drive_2b, HIGH);
}

// handle left motor
void handle_left(float left_val) {

  if (left_val < 0) {
    setMotor1_Backward();
  }
  else {
    setMotor1_Forward();
  }
  softPwmWrite(pwm_1, abs(int(left_val)));
}

// handle right motor
void handle_right(float right_val) {

  if (right_val < 0) {
    setMotor2_Backward();
  }
  else {
    setMotor2_Forward();
  }
  softPwmWrite(pwm_2, abs(int(right_val)));
}
};

MotorControl motor_control_obj;

// Callback fxn for motor control messages. We use arcade drive and then multiply the 
// desired velocity by 512 to get the desired PWM to input to the motors
void motorControlCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	float lin_vel = msg->linear.x;
	float ang_vel = msg->angular.z;
	
	motor_control_obj.vel_l = 512*(lin_vel - ang_vel);
	motor_control_obj.vel_r = 512*(lin_vel + ang_vel);

	motor_control_obj.last_update_time = ros::Time::now();

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_control");
	ros::NodeHandle nh;
	ROS_INFO("Motor control starting up!");

  // subscribe to the motor_control topic
	ros::Subscriber sub = nh.subscribe("motor_control", 1, motorControlCallback);

	ros::Rate loop_rate(50);
	motor_control_obj.last_update_time = ros::Time::now();

	while (ros::ok()) {

		ros::Duration elapsed_time = ros::Time::now() - motor_control_obj.last_update_time;
		if (elapsed_time.toSec() > 1) {
			motor_control_obj.vel_l = 0;
			motor_control_obj.vel_r = 0;
		}
    
    motor_control_obj.handle_left(motor_control_obj.vel_l);
    motor_control_obj.handle_right(motor_control_obj.vel_r);
    loop_rate.sleep();
    ros::spinOnce();
	}

	// when we exit, turn off the motors
	;
	softPwmWrite(motor_control_obj.pwm_1, 0);
	softPwmWrite(motor_control_obj.pwm_2, 0);
      	return 0;
}



