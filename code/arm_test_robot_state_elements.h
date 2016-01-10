#ifndef ARM_ROBOT_STATE_ELEMENTS
#define ARM_ROBOT_STATE_ELEMENTS

#include "robot_util.h"
#include "arm.h"

static arm_state prs = {1.0, 0.0,
                        0.0, 0.0,
                        0.0, 0.0,};

float arm_winch_power = 0.0f;
float arm_shoulder_power = 0.0f;
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

float play = 0;

arm_state linear_integrate(arm_state s, arm_derivatives d, float dt)
{
    for(int i = 0; i < sizeof(s)/sizeof(s.values[0]); i++)
    {
        s.values[i] += d.values[i]*dt;
    }
    
    //s.shoulder_omega = clamp(s.shoulder_omega, -10, 10);
    //s.winch_omega = clamp(s.winch_omega, -10, 10);
    //s.forearm_omega = clamp(s.forearm_omega, -10, 10);
    return s;
}

arm_derivatives evaluate(arm_state s, arm_derivatives d, float dt)
{
    s = linear_integrate(s, d, dt);
    arm_derivatives o = getArmDerivatives(s, arm_shoulder_power, arm_winch_power);
    return o;
}

bool virtual_right_stick_held = false;
void simulateAndRender()
{
    ////////////////Control Map////////////////
    //TODO: make ui use a render queue so buttons can be called when the shader isn't running
    float x_pos = 0;
    #if 0
    if(doButtonNW("winch in", x_pos, 1.0, 4, 2))
    {
        gamepad2.right_stick.x = -1.0;
    }
    x_pos += getTextWidthInWindowUnits("winch in")+(2*4)*wx_scale;
    if(doButtonNW("winch off", x_pos , 1.0, 4, 2))
    {
        gamepad2.right_stick.x = 0.0;
    }
    x_pos += getTextWidthInWindowUnits("winch off")+(2*4)*wx_scale;
    if(doButtonNW("winch out", x_pos, 1.0, 4, 2))
    {
        gamepad2.right_stick.x = 1.0;
    }
    //arm_shoulder_power = 0;//sin(time*10);//gamepad1.left_stick.y+gamepad1.left_stick.x;
        
    x_pos = 0.0;
    if(doButtonNW("shoulder in", x_pos, -0.8, 4, 2))
    {
        gamepad2.right_stick.y = 1.0;
    }
    x_pos += getTextWidthInWindowUnits("shoulder in")+(2*4)*wx_scale;
    if(doButtonNW("shoulder off", x_pos , -0.8, 4, 2))
    {
        gamepad2.right_stick.y = 0.0;
    }
    x_pos += getTextWidthInWindowUnits("shoulder off")+(2*4)*wx_scale;
    if(doButtonNW("shoulder out", x_pos, -0.8, 4, 2))
    {
        gamepad2.right_stick.y = -1.0;
    }
    #else    
    virtual_joystick virtual_right_stick = doVirtualJoystickNW(virtual_right_stick_held, 0.5, -0.5, 100, 100);
    gamepad2.right_stick = virtual_right_stick.joystick;
    virtual_right_stick_held = virtual_right_stick.held;
    #endif
    ///////////////////////////////////////////
    
    //TODO: make it work for larger timesteps
    float dt = 0.0005;
    
    x_pos = 0;
    if(doButtonNW("pause", x_pos, -0.5, 4, 2))
    {
        play = 0;
    }
    x_pos += getTextWidthInWindowUnits("pause")+(2*4)*wx_scale;
    if(doButtonNW("slow", x_pos, -0.5, 4, 2))
    {
        play = 0.1;
    }
    x_pos += getTextWidthInWindowUnits("slow")+(2*4)*wx_scale;
    if(doButtonNW("play", x_pos, -0.5, 4, 2))
    {
        play = 1;
    }
    x_pos += getTextWidthInWindowUnits("play")+(2*4)*wx_scale;
    dt *= play;
    
    {
        { //RK4 integrator
            arm_derivatives k_1 = getArmDerivatives(prs, arm_shoulder_power, arm_winch_power);
            arm_derivatives k_2 = evaluate(prs, k_1, dt*0.5f);
            arm_derivatives k_3 = evaluate(prs, k_2, dt*0.5f);
            arm_derivatives k_4 = evaluate(prs, k_3, dt*1.0f);
            
            arm_derivatives d;
            #define final_derivative(value) 1.0/6.0*(k_1.value+2.0f*(k_2.value+k_3.value)+k_4.value)
            for(int i = 0; i < sizeof(d)/sizeof(d.values[0]); i++)
            {
                d.values[i] = final_derivative(values[i]);
            }
            #undef final_derivative
            
            prs = linear_integrate(prs, d, dt);
        }
    }
    
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {shoulder_length/2*cos(prs.shoulder_theta),
                                               -20.0,
                                               shoulder_length/2*sin(prs.shoulder_theta)};
    render_list[n_to_render].orientation = (v4f) {0.0, -sin(prs.shoulder_theta/2), 0.0, cos(prs.shoulder_theta/2)};
    n_to_render++;
    
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {shoulder_length*cos(prs.shoulder_theta)+forearm_length/2*cos(prs.forearm_theta),
                                               -20.0,
                                               shoulder_length*sin(prs.shoulder_theta)+forearm_length/2*sin(prs.forearm_theta)};
    render_list[n_to_render].orientation = (v4f) {0.0, -sin(prs.forearm_theta/2), 0.0, cos(prs.forearm_theta/2)};
    n_to_render++;
    
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {0.0,
                                               -20.0,
                                               0.0};
    render_list[n_to_render].orientation = (v4f) {0.0, -sin(prs.winch_theta/2), 0.0, cos(prs.winch_theta/2)};
    n_to_render++;
}

typedef int jthrowable;
jthrowable updateRobot(JNIEnv * env, jobject self)
{
    return 0;
}

#endif
