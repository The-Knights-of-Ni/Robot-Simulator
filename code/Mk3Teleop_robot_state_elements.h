#ifndef MK3TELEOP_ROBOT_STATE_ELEMENTS
#define MK3TELEOP_ROBOT_STATE_ELEMENTS

#include "robot_util.h"

double time = 0.0;
  
int right_drive_encoder;
int left_drive_encoder;
int winch_encoder;
int shoulder_encoder;
int elbow_potentiometer;
float heading;
float tilt;
float roll;
float x_velocity;
float y_velocity;
  
float left_drive;
float right_drive;
float winch;
float shoulder;
float intake;
float hand;
float slide;

//float hand_print_position; //for checking servo values
float shoulder_print_theta;
float forearm_print_theta;
  
gamepad gamepad1;
gamepad gamepad2;

JNIEnv * env;
jobject self;

void waitForStart()
{
    
}

void initJNI()
{
    
}

void cleanupJNI()
{
    
}

typedef int jthrowable;
jthrowable updateRobot()
{
    gamepad1.buttons = 0;
    gamepad2.buttons = 0;
    return 1;
}
#endif
