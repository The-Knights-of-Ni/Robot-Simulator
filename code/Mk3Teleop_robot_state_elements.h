/*
Handling most of the JNI stuff right now, needs to be tossed out
*/
#ifndef MK3TELEOP_ROBOT_STATE_ELEMENTS
#define MK3TELEOP_ROBOT_STATE_ELEMENTS

#include "robot_util.h"

double timeSim = 0.0;
//
// int right_drive_encoder;
// int left_drive_encoder;
// int winch_encoder;
// int shoulder_encoder;
// int elbow_potentiometer;
// float heading;
// float tilt;
// float roll;
// float x_velocity;
// float y_velocity;
//
// float left_drive;
// float right_drive;
// float winch;
// float shoulder;
// float intake;
// float hand;
// float slide;
//
//float hand_print_position; //for checking servo values
float shoulder_print_theta;
float forearm_print_theta;

gamepad simulatorgamepad1;
gamepad simulatorgamepad2;

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
    simulatorgamepad1.buttons = 0;
    simulatorgamepad2.buttons = 0;
    return 1;
}

float dtSim;
float old_time = timeSim;

//just because the simulator doesn't prefectly treat it like real code yet
void simulatorStartMain()
{
    // shoulder_omega = 0;
    // target_arm_theta = pi*7/12;
    // target_shoulder_theta = pi*150/180;
    // target_inside_elbow_theta = pi/6;
}

extern "C"
void JNI_main(JNIEnv * _env, jobject _self)
{
    env = _env;
    self = _self;

    initJNI();

    waitForStart();

    do
    {
        dtSim = timeSim-old_time;
        if(dtSim == 0) break;//SIMULATOR THING
        old_time = timeSim;
    } while(updateRobot() == 0);

    cleanupJNI();
}

#endif
