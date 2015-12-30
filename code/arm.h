/*
  robot_state_elements
  {
  gamepad{float joystick_1_x, float joystick_1_y, float joystick_2_x, float joystick_2_y};
  
  float arm_winch_power;
  float arm_shoulder_power;
  
  gamepad gamepad1;
  }
 */

float g = 384; //gravity in " per sec

float winch_pulley_r = 1.0f;
float shoulder_pulley_r = 1.0f;
float elbow_pulley_r = 2.0f;

float winch_I = 100.0; //the rotational inertia of the forearm

float shoulder_I = 81.0f*80; //the rotational inertia of the shoulder
float shoulder_m = 80.0f;
float shoulder_gear_ratio = 6.75;

float forearm_I = 81.0f*200; //the rotational inertia of the forearm
float forearm_m = 200.0f;

float forearm_length = 17.0f;
float forearm_cm_dist = 9.0f;
float shoulder_length = 16.5f;
float shoulder_cm_dist = 9.0f;

float spring_force = 2*2802000*0;

float k_string = 100000000*0;

float dc_motor_voltage = 14.0f;

float neverest_max_torque = 4334000; //in g in^2/s^2
float neverest_max_speed = 13.51; //in rad/s

float neverest_k_i = dc_motor_voltage/neverest_max_speed;
float neverest_k_t_over_R = neverest_max_torque/dc_motor_voltage;

float string_length_0 = 29.908846;

struct gamepad
{
    v2f left_stick;
    v2f right_stick;
};

#include "arm_robot_state_elements.h"

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    static float time = 0;
    time += 0.0005;
    
    float x_pos = 0.0;
    //for ever
    {
        ////////////////Control Map////////////////
        //arm_winch_power = -0.0;//gamepad1.left_stick.y-gamepad1.left_stick.x;
        x_pos = 0.0;
        //TODO: make ui use a render queue so buttons can be called when the shader isn't running
        if(doButtonNW("winch in", x_pos, 1.0, 4, 2))
        {
            arm_winch_power = -1.0;
        }
        x_pos += getTextWidthInWindowUnits("winch in")+(2*4)*wx_scale;
        if(doButtonNW("winch off", x_pos , 1.0, 4, 2))
        {
            arm_winch_power = 0.0;
        }
        x_pos += getTextWidthInWindowUnits("winch off")+(2*4)*wx_scale;
        if(doButtonNW("winch out", x_pos, 1.0, 4, 2))
        {
            arm_winch_power = 1.0;
        }
        //arm_shoulder_power = 0;//sin(time*10);//gamepad1.left_stick.y+gamepad1.left_stick.x;

        x_pos = 0.0;
        if(doButtonNW("shoulder in", x_pos, -0.8, 4, 2))
        {
            arm_shoulder_power = 1.0;
        }
        x_pos += getTextWidthInWindowUnits("shoulder in")+(2*4)*wx_scale;
        if(doButtonNW("shoulder off", x_pos , -0.8, 4, 2))
        {
            arm_shoulder_power = 0.0;
        }
        x_pos += getTextWidthInWindowUnits("shoulder off")+(2*4)*wx_scale;
        if(doButtonNW("shoulder out", x_pos, -0.8, 4, 2))
        {
            arm_shoulder_power = -1.0;
        }
        ///////////////////////////////////////////
        
        updateRobot(env, self);
    }
    cleanupJNI(env, self);
}
