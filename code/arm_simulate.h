#ifndef ARM_SIMULATE
#define ARM_SIMULATE

//#include "arm_robot_state_elements.h"
#include "ui.h"
#include "arm.h"
#include "Button.h"

static arm_state prs = {pi*150/180, 0.0,
                        pi*150/180-pi+pi/6, 0.0,
                        0.0, 0.0,};

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
    arm_derivatives o = getArmDerivatives(s, shoulder, winch);
    return o;
}

bool virtual_right_stick_held = false;
bool virtual_left_stick_held = false;
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
    gamepad2.joystick2 = virtual_right_stick.joystick;
    virtual_right_stick_held = virtual_right_stick.held;

    virtual_joystick virtual_left_stick = doVirtualJoystickNW(virtual_left_stick_held, -0.5, -0.5, 100, 100);
    gamepad2.joystick1 = virtual_left_stick.joystick;
    virtual_left_stick_held = virtual_left_stick.held;    
    #endif
    
    x_pos = -0.5;
    gamepad2.left_trigger = doButtonNW("intake position", x_pos, -0.3, 4, 2);
    x_pos += getTextWidthInWindowUnits("intake position")+(2*4)*wx_scale;
    gamepad2.right_trigger = doButtonNW("score position", x_pos, -0.3, 4, 2);
    x_pos += getTextWidthInWindowUnits("score position")+(2*4)*wx_scale;
    gamepad2.buttons |= (doButtonNW("manual control", x_pos, -0.3, 4, 2) << DPAD_UP);
    x_pos += getTextWidthInWindowUnits("manual control")+(2*4)*wx_scale;
    gamepad2.buttons |= (doButtonNW("slow model", x_pos, -0.3, 4, 2) << A);

    if(joystick)
    {
        gamepad2.joystick1 = (v2f){(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_LEFTX)+0.5)/32767.5,
                                   -(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_LEFTY)+0.5)/32767.5};
        
        gamepad2.joystick2 = (v2f){(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTX)+0.5)/32767.5,
                                   -(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTY)+0.5)/32767.5};

        gamepad2.left_trigger = (SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERLEFT)+0.5)/32767.5;
        gamepad2.right_trigger = (SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERRIGHT)+0.5)/32767.5;

        gamepad2.buttons |= (SDL_JoystickGetButton(joystick, 0) << DPAD_UP);
        gamepad2.buttons |= (SDL_JoystickGetButton(joystick, 1) << DPAD_DOWN);
        gamepad2.buttons |= (SDL_JoystickGetButton(joystick, 10) << A);
        gamepad2.buttons |= (SDL_JoystickGetButton(joystick, 13) << Y);
    }
    else
    {
        printf("no controller\n");
    }
    
    ///////////////////////////////////////////
    
    float potentiometer_range = 333.33333333333333333333333333333333333f;
    
    shoulder_encoder = (prs.shoulder_theta-pi*150.0/180.0)*shoulder_gear_ratio*encoder_ticks_per_radian;
    shoulder_encoder = ((int) shoulder_encoder); //TODO: real floor function, casting to int and back is inefficient
    winch_encoder = prs.winch_theta*winch_gear_ratio*encoder_ticks_per_radian;
    winch_encoder = ((int) winch_encoder);
    elbow_potentiometer = ((-180+potentiometer_range*0.5-(((pi+prs.forearm_theta-prs.shoulder_theta)*180.0/pi)-360)-12)/potentiometer_range)*1023;
    elbow_potentiometer = ((int) elbow_potentiometer);
    
    x_pos = -1;
    
    char * left_joystick_string = (char *) malloc(100);
    sprintf(left_joystick_string, "left joystick (%f, %f)",
            gamepad2.joystick1.x, gamepad2.joystick1.y);
    doHoldButtonNW(left_joystick_string, x_pos, 0.4, 4, 2);

    char * right_joystick_string = (char *) malloc(100);
    sprintf(right_joystick_string, "right joystick (%f, %f)",
            gamepad2.joystick2.x, gamepad2.joystick2.y);
    doHoldButtonNW(right_joystick_string, x_pos, 0.3, 4, 2);
    
    char * time_string = (char *) malloc(100);
    sprintf(time_string, "time %f", time);
    doHoldButtonNW(time_string, x_pos, 0.2, 4, 2);
    
    char * potentiometer_string = (char *) malloc(100);
    sprintf(potentiometer_string, "elbow_potentiometer %d", elbow_potentiometer);
    doHoldButtonNW(potentiometer_string, x_pos, 0.1, 4, 2);

    char * shoulder_string = (char *) malloc(100);
    sprintf(shoulder_string, "shoulder_theta %f", shoulder_print_theta);
    doHoldButtonNW(shoulder_string, x_pos, 0.0, 4, 2);

    char * forearm_string = (char *) malloc(100);
    sprintf(forearm_string, "forearm_theta %f", forearm_print_theta);
    doHoldButtonNW(forearm_string, x_pos, -0.1, 4, 2);
    
    //TODO: make it work for larger timesteps
    float dt = 0.00075;
    
    time += dt;
    
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
            arm_derivatives k_1 = getArmDerivatives(prs, shoulder, winch);
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

    #if 0
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {shoulder_length/2*cos(prs.shoulder_theta),
                                               -20.0,
                                               shoulder_length/2*sin(prs.shoulder_theta)};
    render_list[n_to_render].orientation = (v4f) {0.0, -sin(prs.shoulder_theta/2), 0.0, cos(prs.shoulder_theta/2)};
    n_to_render++;
    
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {shoulder_length*cos(prs.shoulder_theta)
                                               +((forearm_length-shoulder_length)/2+forearm_length/2)*cos(prs.forearm_theta),
                                               -20.0,
                                               shoulder_length*sin(prs.shoulder_theta)+((forearm_length-shoulder_length)/2+forearm_length/2)*sin(prs.forearm_theta)};
    render_list[n_to_render].orientation = (v4f) {0.0, -sin(prs.forearm_theta/2), 0.0, cos(prs.forearm_theta/2)};
    n_to_render++;
    
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {0.0,
                                               -20.0,
                                               0.0};
    render_list[n_to_render].orientation = (v4f) {0.0, -sin(prs.winch_theta/2), 0.0, cos(prs.winch_theta/2)};
    n_to_render++;
    #else //render mirrored
        render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {-shoulder_length/2*cos(prs.shoulder_theta),
                                               -20.0,
                                               shoulder_length/2*sin(prs.shoulder_theta)};
    render_list[n_to_render].orientation = (v4f) {0.0, sin(prs.shoulder_theta/2), 0.0, cos(prs.shoulder_theta/2)};
    n_to_render++;
    
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {-shoulder_length*cos(prs.shoulder_theta)
                                               -((forearm_length-shoulder_length)/2+forearm_length/2)*cos(prs.forearm_theta),
                                               -20.0,
                                               shoulder_length*sin(prs.shoulder_theta)+((forearm_length-shoulder_length)/2+forearm_length/2)*sin(prs.forearm_theta)};
    render_list[n_to_render].orientation = (v4f) {0.0, sin(prs.forearm_theta/2), 0.0, cos(prs.forearm_theta/2)};
    n_to_render++;
    
    render_list[n_to_render].model = 0;
    render_list[n_to_render].position = (v3f) {0.0,
                                               -20.0,
                                               0.0};
    render_list[n_to_render].orientation = (v4f) {0.0, sin(prs.winch_theta/2), 0.0, cos(prs.winch_theta/2)};
    n_to_render++;
    #endif
}

#endif
