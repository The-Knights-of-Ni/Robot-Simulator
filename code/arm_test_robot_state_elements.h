#ifndef ARM_ROBOT_STATE_ELEMENTS
#define ARM_ROBOT_STATE_ELEMENTS

#include "robot_util.h"

float arm_winch_power = 0.0f;
float arm_shoulder_power = 0.0f;
float winch_encoder = 0.0f;
float shoulder_encoder = 0.0f;
float elbow_potentiometer = 0.0f;
gamepad gamepad1 = {};
gamepad gamepad2 = {};

void waitForStart()
{
    
}

void initJNI(JNIEnv * env, jobject self)
{
    
}

void cleanupJNI(JNIEnv * env, jobject self)
{
    
}

typedef int jthrowable;
jthrowable updateRobot(JNIEnv * env, jobject self)
{
    return 0;
}
#endif
