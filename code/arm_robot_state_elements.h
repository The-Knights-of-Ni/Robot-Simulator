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

void updateRobot(JNIEnv * env, jobject self)
{

    float dt = 0.0005;
    
    {
        float inside_elbow_theta = prs.forearm_theta+(pi-prs.shoulder_theta);
        float string_length;
        float string_theta;        
        
        bool8 winch_mode = 0;
        if(winch_mode)
        {
            float shoudler_axis_to_end_sq = sq(forearm_length)+sq(shoulder_length)
                -2*forearm_length*shoulder_length*cos(inside_elbow_theta);
            
            string_theta = prs.shoulder_theta
                -asin(forearm_length/sqrt(shoudler_axis_to_end_sq)*sin(inside_elbow_theta))
                -asin(shoulder_pulley_r/sqrt(shoudler_axis_to_end_sq))
                +pi/2;
            
            string_length =
                sqrt(shoudler_axis_to_end_sq
                     -sq(shoulder_pulley_r))
                +string_theta*shoulder_pulley_r;
        }
        else
        {
            //TODO: I'm not sure if this works when elbow_pulley_r < shoulder_pulley_r
            //      might need to add or subtract 2*pi or something
            float shoulder_string_theta = prs.shoulder_theta+asin((elbow_pulley_r-shoulder_pulley_r)/shoulder_length);
            string_theta = prs.forearm_theta+acos((elbow_pulley_r-shoulder_pulley_r)/elbow_pulley_r)-pi/2;
            
            string_length = sqrt(sq(shoulder_length)-sq(elbow_pulley_r-shoulder_pulley_r))
                +(string_theta-shoulder_string_theta)*elbow_pulley_r
                +sqrt(sq(forearm_length)-sq(elbow_pulley_r));
        }
        
        float string_tension = k_string*(-(prs.winch_theta*winch_pulley_r)+string_length-string_length_0);
        if(string_tension < 0) string_tension = 0;
        
        float shoulder_motor_voltage = (arm_shoulder_power*dc_motor_voltage - neverest_k_i*prs.shoulder_omega);
        float shoulder_motor_tau = shoulder_motor_voltage*neverest_k_t_over_R;
        
        //TODO: add friction
        //all of the torques not including the axle
        float shoulder_tau_other = shoulder_motor_tau - shoulder_cm_dist*shoulder_m*g*cos(prs.shoulder_theta);
        
        float forearm_tau_other = string_tension*(forearm_length-forearm_cm_dist)*sin(string_theta-prs.forearm_theta)
            +spring_force*elbow_pulley_r
            -forearm_cm_dist*forearm_m*g*cos(prs.forearm_theta);
        
        float shoulder_tau = shoulder_tau_other;
        
        float forearm_tau = forearm_tau_other-1000000*prs.forearm_omega;
        
        prs.shoulder_omega += shoulder_tau/shoulder_I*dt;
        prs.shoulder_theta += prs.shoulder_omega*dt;
        if(prs.shoulder_theta < 0.0f)
        {
            prs.shoulder_theta = 0.0f;
            prs.shoulder_omega = -prs.shoulder_omega;
        }
        
        prs.forearm_omega += forearm_tau/forearm_I*dt;
        prs.forearm_theta += prs.forearm_omega*dt;
        
        float winch_motor_voltage = (arm_winch_power*dc_motor_voltage - neverest_k_i*prs.winch_omega);
        float winch_motor_tau = winch_motor_voltage*neverest_k_t_over_R;
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
}

#endif
