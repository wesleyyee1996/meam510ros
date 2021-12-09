
#pragma once
#include <wiringPi.h>
#include <softPwm.h>

class MotorControl {
  public: 
    float led_freq = 50; // frequency for driving the motors
    int led_channel_1 = 1; // PWM channel set to 1 for motor 1, and 2 for motor 2
    int led_channel_2 = 2;
    int MAX_DUTY = 255;
    int pwm_1 = 13; // pwm pin for motor 1
    int pwm_2 = 12; // pwm pin for motor 2
    int motor_drive_1a = 27; // motor drive 1
    int motor_drive_1b = 22; 
    int motor_drive_2a = 23; // motor drive 2
    int motor_drive_2b = 24;
    int pwm_resolution = 8; // 8-bit resolution, which is 255
    int steering_angle = 0;
    int left_out = 0;
    int right_out = 0;

    void setup_motors();

    void setMotor1_Forward();

    void setMotor2_Forward();

    void setMotor1_Backward();

    void setMotor2_Backward();

    void handle_left(int y_val, int steering_angle);

    void handle_right(int y_val, int steering_angle);
};
