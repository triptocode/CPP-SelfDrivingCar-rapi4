// motor_control.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#define IN1 22 // GPIO pin for left motor input 1
#define IN2 23 // GPIO pin for left motor input 2
#define IN3 17 // GPIO pin for right motor input 3
#define IN4 18 // GPIO pin for right motor input 4
#define ENABLE 12 // GPIO pin for motor enable (PWM)
#define ENB 13 // GPIO pin for motor enable (PWM)


void Driving_forward(int in1, int in2, int in3, int in4, int speed);
void Driving_backward(int in1, int in2, int in3, int in4, int speed);
void Driving_left(int in1, int in2, int in3, int in4, int speed);
void Driving_right(int in1, int in2, int in3, int in4, int speed);
void Driving_stop(int in1, int in2, int in3, int in4, int speed);

#endif
