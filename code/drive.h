#ifndef DRIVE
#define DRIVE

#include "robotics.h"
//TODO: Traction control
//TODO: Negative Inertia
//TODO: D-PAD Turn macros (90, 180, 270, simulated shift)
//TODO: Drive on course (turn by bearing)
//TODO: Stabilized driving (if we get slammed, we should correct)
//TODO: Delete this comment

#define threshold 0.

#define sprocket_pitch_radius 3.13 //Inches
#define encoderticks_per_radian 1440.0/(2*pi)
#define encoderticks_per_inch sprocket_pitch_radius*encoderticks_per_radian
#define encoderticks_per_cm sprocket_pitch_radius*2.54*encoderticks_per_radian
#define acceptableAngleError 2

void deadZone(v2f &stick)
{
    float stick_norm = norm(stick);
    if (stick_norm < threshold)
    {
        stick.data[0] = 0;
        stick.data[1] = 0;
    }
    else
    {
        stick * ((stick_norm - threshold) / (1.0f - threshold)) /
        stick_norm;
    }//Remap non-deadzone to full range. Unnecessary if we can't move at 10% pwm
}

void squareDeadZone(v2f &stick)
{
    if (fabs(stick.data[0]) < threshold)
    {
        stick.data[0] = 0;
    }
    if (fabs(stick.data[1]) < threshold)
    {
        stick.data[1] = 0;
    }
}

//this doesn't compile right now
#if 0
//TODO: go 0-100 for vis instead of 0-1, for clarity
void driveDistIn(float dist, float vIs)
{
    if (dist < 0)
    {
        dist = fabs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_inch ||
           left_enc_net < dist * encoderticks_per_inch)
    {
        if (doInit)
        {
            updateRobot(env, self); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;

        if (right_enc_net < dist * encoderticks_per_inch)
        {
            right_drive = vIs;
        }
        if (left_enc_net < dist * encoderticks_per_inch)
        {
            left_drive = vIs;
        }

        updateRobot(env, self);
    }
}

void driveDistCm(float dist, float vIs)
{
    if (dist < 0)
    {
        dist = abs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_cm || left_enc_net < dist * encoderticks_per_cm)
    {
        if (doInit)
        {
            updateRobot(env, self); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;

        if (right_enc_net < dist * encoderticks_per_cm)
        {
            right_drive = vIs;
        }
        if (left_enc_net < dist * encoderticks_per_cm)
        {
            left_drive = vIs;
        }

        updateRobot(env, self);
    }
}

#define side_slowing_constant 5
void driveOnCourseIn(float dist, float vIs,
                     float target_heading) //Assuming we start facing 180 degrees (intake side)
{
    if (dist < 0)
    {
        dist = abs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_inch ||
           left_enc_net < dist * encoderticks_per_inch)
    {
        if (doInit)
        {
            updateRobot(env, self); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;
        if (tolerantEquals(heading, target_heading, acceptableAngleError))
        {
            //TODO: Recovery code that makes sure we don't gain ticks by getting hit
            if (right_enc_net < dist * encoderticks_per_inch)
            {
                right_drive = vIs;
            }
            if (left_enc_net < dist * encoderticks_per_inch)
            {
                left_drive = vIs;
            }
        }
        else if(isAngleGreater(heading, target_heading)) //TODO: make it so it ramps up if we're running at less than 100
        {
            left_drive = vIs;
            right_drive -= side_slowing_constant;
        }
        else
        {
            left_drive -= side_slowing_constant;
            right_drive = vIs;
        }
        updateRobot(env, self);
    }
}

void driveOnCourseCm(float dist, float vIs,
                     float target_heading) //Assuming we start facing 180 degrees (intake side)
{
    if (dist < 0)
    {
        dist = abs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_cm ||
           left_enc_net < dist * encoderticks_per_cm)
    {
        if (doInit)
        {
            updateRobot(env, self); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;
        if (tolerantEquals(heading, target_heading, acceptableAngleError))
        {
            //TODO: Recovery code that makes sure we don't gain ticks by getting hit
            if (right_enc_net < dist * encoderticks_per_cm)
            {
                right_drive = vIs;
            }
            if (left_enc_net < dist * encoderticks_per_cm)
            {
                left_drive = vIs;
            }
        }
        else if(isAngleGreater(heading, target_heading)) //TODO: make it so it ramps up if we're running at less than 100
        {
            left_drive = vIs;
            right_drive -= side_slowing_constant;
        }
        else
        {
            left_drive -= side_slowing_constant;
            right_drive = vIs;
        }
        updateRobot(env, self);
    }
}

void turnRelDeg(float angle, float vIs)
{
    updateRobot(env, self);
    while(!tolerantEquals(heading, heading + angle, acceptableAngleError))
    {
        if(isAngleGreater(heading, heading + angle))
        {
            left_motor = vIs;
            right_motor = -vIs;
        }
        else
        {
            left_motor = -vIs;
            right_motor = vIs;
        }
        updateRobot(env, self);
    }
}
#endif

#define Px_0 0
#define Px_1 0.2 //Set this to something the driver likes
#define Px_2 1.0
#define Py_0 0
#define Py_1 0.8 //Set this to something the driver likes
#define Py_2 1

//Bounds and Smooths joystick values for better handling.
void smoothJoysticks(v2f *stick) //TODO: Move to <robot_name>.h and have custom constants
{
    stick->data[0] = bound(stick->data[0], -1,
                           1);//Clamp between -1 and 1, might just build the deadzone into here
    stick->data[1] = bound(stick->data[1], -1, 1);
    stick->data[0] = (stick->data[0] < 0 ? -1 : 1) *
                     ((1 - stick->data[0]) * ((1 - stick->data[0]) * Px_0 + stick->data[0] * Px_1) +
                      stick->data[0] * ((1 - stick->data[0]) * Px_1 + stick->data[0] *
                                                                      Px_2)); //Quadratic Bezier, might want to make this a separate definition later
    stick->data[1] = (stick->data[1] < 0 ? -1 : 1) *
                     ((1 - stick->data[1]) * ((1 - stick->data[1]) * Py_0 + stick->data[1] * Py_1) +
                      stick->data[1] * ((1 - stick->data[1]) * Py_1 + stick->data[1] * Py_2));
}

float raw_x;
float raw_y;

#define smoothConstant 0.6 //Set this to something the driver likes

v2f smoothJoysticks254Style(v2f stick)//TODO: Fix this
{
    v2f smoothed;
    raw_x = bound(raw_x, -1, 1);//Clamp between -1 and 1
    raw_y = bound(raw_y, -1, 1);
    smoothed.data[0] = sin((pi * smoothConstant * raw_x / 2.0) /
                           (pi / 2.0)); //Sin wave: https://www.desmos.com/calculator/4hd9ovg7el
    smoothed.data[1] = sin((pi * smoothConstant * raw_y / 2.0) / (pi / 2.0));
    return smoothed;//Give back smooth x and y values
}

#endif //DRIVE
