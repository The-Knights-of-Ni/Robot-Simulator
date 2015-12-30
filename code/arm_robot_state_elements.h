#ifndef ARM_ROBOT_STATE_ELEMENTS
#define ARM_ROBOT_STATE_ELEMENTS

#include "robot_util.h"

struct physical_robot_state
{
    float winch_theta;
    float forearm_theta;
    float shoulder_theta;
    
    float winch_omega;
    float forearm_omega;
    float shoulder_omega;   
};

static physical_robot_state prs = {0.0, 0.0, 1.0};

float arm_winch_power = 0.0f;
float arm_shoulder_power = 0.0f;
gamepad gamepad1 = {};

void waitForStart()
{
    
}

void initJNI(JNIEnv * env, jobject self)
{
    
}

void cleanupJNI(JNIEnv * env, jobject self)
{
    
}

bool8 play = 0;

void updateRobot(JNIEnv * env, jobject self)
{
    
    float dt = 0.0005*1;
    
    float x_pos = 0;
    if(doButtonNW("pause", x_pos, -0.5, 4, 2))
    {
        play = 0;
    }
    x_pos += getTextWidthInWindowUnits("pause")+(2*4)*wx_scale;
    if(doButtonNW("play", x_pos, -0.5, 4, 2))
    {
        play = 1;
    }
    x_pos += getTextWidthInWindowUnits("play")+(2*4)*wx_scale;
    if(!play) dt = 0;
    
    {
        /* while(prs.forearm_theta > 2*pi) prs.forearm_theta -= 2*pi; */
        /* while(prs.forearm_theta < -2*pi) prs.forearm_theta += 2*pi; */

        /* while(prs.shoulder_theta > 2*pi) prs.shoulder_theta -= 2*pi; */
        /* while(prs.shoulder_theta < -2*pi) prs.shoulder_theta += 2*pi; */
        
        float inside_elbow_theta = prs.forearm_theta+(pi-prs.shoulder_theta);
        
        float string_length;
        float string_theta;
        
        bool8 winch_mode = inside_elbow_theta < (acos((elbow_pulley_r-shoulder_pulley_r)/shoulder_length)+acos(elbow_pulley_r/forearm_length));
        if(winch_mode)
        {
            doButtonNW("winch mode", 0.0, 0.5, 4, 2);
            float shoudler_axis_to_end_sq = sq(forearm_length)+sq(shoulder_length)
                -2*forearm_length*shoulder_length*cos(inside_elbow_theta);
            
            string_theta = prs.shoulder_theta
                -asin(forearm_length/sqrt(shoudler_axis_to_end_sq)*sin(inside_elbow_theta))
                +asin(shoulder_pulley_r/sqrt(shoudler_axis_to_end_sq));
            
            string_length =
                sqrt(shoudler_axis_to_end_sq
                     -sq(shoulder_pulley_r))
                +string_theta*shoulder_pulley_r;
            /* printf("string_length_0 = %f\n", string_length); */
            /* play = 0; */
        }
        else
        {
            if(doButtonNW("not winch mode", 0.0, 0.5, 4, 2))
            {
                prs.forearm_theta = -1.0;
            }
            //TODO: I'm not sure if this works when elbow_pulley_r < shoulder_pulley_r
            //      might need to add or subtract 2*pi or something
            float shoulder_string_theta = prs.shoulder_theta-asin((elbow_pulley_r-shoulder_pulley_r)/shoulder_length);
            string_theta = prs.forearm_theta+asin(elbow_pulley_r/forearm_length);
            
            string_length = sqrt(sq(shoulder_length)-sq(elbow_pulley_r-shoulder_pulley_r))
                +(string_theta-shoulder_string_theta)*elbow_pulley_r
                +sqrt(sq(forearm_length)-sq(elbow_pulley_r))
                +shoulder_string_theta*shoulder_pulley_r;
        }
        
        float string_tension = k_string*(-(prs.winch_theta*winch_pulley_r)+string_length-string_length_0);
        if(string_tension < 0)
        {
            string_tension = 0;
            doButtonNW("string not engaged", 0.0, 0.0, 4, 2);
        }
        
        float shoulder_motor_voltage = (arm_shoulder_power*dc_motor_voltage + neverest_k_i*prs.shoulder_omega*shoulder_gear_ratio);
        float shoulder_motor_tau = shoulder_motor_voltage*neverest_k_t_over_R*shoulder_gear_ratio*0;
        
        //TODO: add friction
        //all of the torques not including the axle
        
        float shoulder_tau_other = shoulder_motor_tau - shoulder_cm_dist*shoulder_m*g*cos(prs.shoulder_theta)
            -spring_force*elbow_pulley_r;//TODO: think more carefully if this is correct
        
        float forearm_tau_other = -string_tension*(forearm_length-forearm_cm_dist)*sin(string_theta-prs.forearm_theta)
            +spring_force*(forearm_cm_dist)*sin(inside_elbow_theta); //TODO: this is very approximate, might need to fix
        
        v2f forearm_F_other = {-spring_force*cos(prs.shoulder_theta)-string_tension*cos(string_theta)
                               +forearm_m*(forearm_cm_dist*sq(prs.forearm_omega)*cos(prs.forearm_theta)
                                           +shoulder_length*sq(prs.shoulder_omega)*cos(prs.shoulder_theta)),
                               -forearm_m*g-spring_force*sin(prs.shoulder_theta)-string_tension*sin(string_theta)
                               +forearm_m*(forearm_cm_dist*sq(prs.forearm_omega)*sin(prs.forearm_theta)
                                           +shoulder_length*sq(prs.shoulder_omega)*sin(prs.shoulder_theta))};
        
        m4x5f axle_force_equations = {
            shoulder_I, 0, shoulder_length*sin(prs.shoulder_theta), -shoulder_length*cos(prs.shoulder_theta), shoulder_tau_other,
            0, forearm_I, -forearm_cm_dist*sin(prs.forearm_theta), forearm_cm_dist*cos(prs.forearm_theta), forearm_tau_other,
            -forearm_m*shoulder_length*sin(prs.shoulder_theta), -forearm_m*forearm_cm_dist*sin(prs.forearm_theta), -1, 0, forearm_F_other.x,
            forearm_m*shoulder_length*cos(prs.shoulder_theta), forearm_m*forearm_cm_dist*cos(prs.forearm_theta), 0, -1, forearm_F_other.y,
        };
        
        v4f axle_force_solutions = solve(axle_force_equations);
        assert(axle_force_solutions[0] == axle_force_solutions[0]);
        assert(axle_force_solutions[1] == axle_force_solutions[1]);
        assert(axle_force_solutions[2] == axle_force_solutions[2]);
        assert(axle_force_solutions[3] == axle_force_solutions[3]);
                
        v2f axle_force = {axle_force_solutions[2], axle_force_solutions[3]};
        
        float shoulder_alpha = axle_force_solutions[0];
        
        float forearm_alpha = axle_force_solutions[1];
        
        prs.shoulder_omega += shoulder_alpha*dt;
        prs.shoulder_theta += prs.shoulder_omega*dt;
        /* if(prs.shoulder_theta < 0.0f) */
        /* { */
        /*     prs.shoulder_theta = 0.0f; */
        /*     prs.shoulder_omega = 0; */
        /* } */
        /* if(prs.shoulder_theta > pi) */
        /* { */
        /*     prs.shoulder_theta = pi; */
        /*     prs.shoulder_omega = 0; */
        /* } */
        
        prs.forearm_omega += forearm_alpha*dt;
        prs.forearm_theta += prs.forearm_omega*dt;

        float new_inside_elbow_theta = prs.forearm_theta-prs.shoulder_theta;
        while(new_inside_elbow_theta < -pi) new_inside_elbow_theta += 2*pi;
        while(new_inside_elbow_theta > pi) new_inside_elbow_theta -= 2*pi;
        if(new_inside_elbow_theta > pi*5/6)
        {//TODO: this does not apply a force to the shoulder bar like it would in real life
            //play = 0;
            //printf("greater: %f, %f\b", prs.forearm_theta, prs.shoulder_theta);
            prs.forearm_theta = prs.shoulder_theta+pi*5/6;
            while(prs.forearm_theta < -pi) prs.forearm_theta += 2*pi;
            while(prs.forearm_theta > pi) prs.forearm_theta -= 2*pi;
            prs.forearm_omega = (shoulder_I*prs.shoulder_omega+forearm_I*prs.forearm_omega)/(shoulder_I+forearm_I);//TODO: this is fake
            prs.shoulder_omega = (shoulder_I*prs.shoulder_omega+forearm_I*prs.forearm_omega)/(shoulder_I+forearm_I);
        }
        if(new_inside_elbow_theta < -pi*5/6)
        {
            //play = 0;
            //printf("less: %f, %f\n", prs.forearm_theta, prs.shoulder_theta);
            prs.forearm_theta = prs.shoulder_theta-pi*5/6;
            while(prs.forearm_theta < -pi) prs.forearm_theta += 2*pi;
            while(prs.forearm_theta > pi) prs.forearm_theta -= 2*pi;
            prs.forearm_omega = (shoulder_I*prs.shoulder_omega+forearm_I*prs.forearm_omega)/(shoulder_I+forearm_I);//TODO: this is fake
            prs.shoulder_omega = (shoulder_I*prs.shoulder_omega+forearm_I*prs.forearm_omega)/(shoulder_I+forearm_I);
        }
        
        float winch_motor_voltage = (arm_winch_power*dc_motor_voltage - neverest_k_i*prs.winch_omega*2.0);
        float winch_motor_tau = winch_motor_voltage*neverest_k_t_over_R*2*2.0;
        float winch_tau = winch_motor_tau+string_tension*winch_pulley_r;
        prs.winch_omega += winch_tau/winch_I*dt;
        prs.winch_theta += prs.winch_omega*dt;
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

#endif
