#ifndef ARM_ROBOT_STATE_ELEMENTS
#define ARM_ROBOT_STATE_ELEMENTS

#include "robot_util.h"

union physical_robot_state
{
    struct
    {
        float shoulder_theta;
        float shoulder_omega;   
    
        float forearm_theta;
        float forearm_omega;

        float winch_theta;
        float winch_omega;
    };
    float values[];
};

static physical_robot_state prs = {1.0, 0.0,
                                   0.0, 0.0,
                                   0.0, 0.0,};

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

float play = 0;

union physical_robot_derivatives
{
    struct
    {
        float shoulder_omega;
        float shoulder_alpha;
    
        float forearm_omega;
        float forearm_alpha;

        float winch_omega;
        float winch_alpha;
    };
    float values[];
};

physical_robot_derivatives getDerivatives(physical_robot_state s)
{
    float inside_elbow_theta = s.forearm_theta+(pi-s.shoulder_theta);
    
    float string_length;
    float string_theta;
    float string_moment_arm;
    
    bool8 winch_mode = inside_elbow_theta < (acos((elbow_pulley_r-shoulder_pulley_r)/shoulder_length)+acos(elbow_pulley_r/forearm_length));
    if(winch_mode)
    {   
        doButtonNW("winch mode", 0.0, 0.5, 4, 2);
        float shoudler_axis_to_end_sq = sq(forearm_length)+sq(shoulder_length)
            -2*forearm_length*shoulder_length*cos(inside_elbow_theta);
        
        string_theta = s.shoulder_theta
            -asin(forearm_length/sqrt(shoudler_axis_to_end_sq)*sin(inside_elbow_theta))
            +asin(shoulder_pulley_r/sqrt(shoudler_axis_to_end_sq));
        
        string_length =
            sqrt(shoudler_axis_to_end_sq
                 -sq(shoulder_pulley_r))
            +string_theta*shoulder_pulley_r;
        
        string_moment_arm = (forearm_length-forearm_cm_dist)*sin(string_theta-s.forearm_theta);
    }
    else
    {        
        if(doButtonNW("not winch mode", 0.0, 0.5, 4, 2))
        {
            s.forearm_theta = -1.0;
        }
        //TODO: I'm not sure if this works when elbow_pulley_r < shoulder_pulley_r
        //      might need to add or subtract 2*pi or something
        string_theta = s.shoulder_theta-asin((elbow_pulley_r-shoulder_pulley_r)/shoulder_length);
        float forearm_string_theta = s.forearm_theta+asin(elbow_pulley_r/forearm_length);
        
        string_length = sqrt(sq(shoulder_length)-sq(elbow_pulley_r-shoulder_pulley_r))
            +(forearm_string_theta-string_theta)*elbow_pulley_r
            +sqrt(sq(forearm_length)-sq(elbow_pulley_r))
            +string_theta*shoulder_pulley_r;
        
        string_moment_arm = forearm_cm_dist*sin(s.forearm_theta-string_theta)+elbow_pulley_r;
    }
    
    float string_tension = k_string*(-(s.winch_theta*winch_pulley_r)+string_length-string_length_0);
    if(string_tension < 0)
    {
        string_tension = 0;
        doButtonNW("string not engaged", 0.0, 0.0, 4, 2);
    }
    
    float shoulder_motor_voltage = (arm_shoulder_power*dc_motor_voltage - neverest_k_i*s.shoulder_omega*shoulder_gear_ratio);
    float shoulder_motor_tau = shoulder_motor_voltage*neverest_k_t_over_R*shoulder_gear_ratio;
    
    //TODO: add friction to winch
    //all of the torques not including the axle
    
    float shoulder_tau_other = shoulder_motor_tau - shoulder_cm_dist*shoulder_m*g*cos(s.shoulder_theta)
        -spring_force*elbow_pulley_r
        -(s.shoulder_omega>0.1 ? 100000 :
          (s.shoulder_omega<-0.1 ?-100000:0))
        +(s.forearm_omega-s.shoulder_omega>0.1 ? 100000 :
          (s.forearm_omega-s.shoulder_omega<-0.1 ?-100000:0));
    
    float forearm_tau_other = -string_tension*string_moment_arm
        +spring_force*elbow_pulley_r
        -(s.forearm_omega-s.shoulder_omega>0.1 ? 100000 :
          (s.forearm_omega-s.shoulder_omega<-0.1 ?-100000:0));
    
    {
        float new_inside_elbow_theta = s.forearm_theta-s.shoulder_theta;
        while(new_inside_elbow_theta < -pi) new_inside_elbow_theta += 2*pi;
        while(new_inside_elbow_theta > pi) new_inside_elbow_theta -= 2*pi;
        
        if(new_inside_elbow_theta > pi*5/6)
        {
            //TODO: assume collision with stop is not elastic
            forearm_tau_other -= 1000000000*(new_inside_elbow_theta-pi*5/6);
            shoulder_tau_other += 1000000000*(new_inside_elbow_theta-pi*5/6);
        }
        
        if(new_inside_elbow_theta < -pi*5/6)
        {
            forearm_tau_other += 1000000000*(-pi*5/6-new_inside_elbow_theta);
            shoulder_tau_other -= 1000000000*(-pi*5/6-new_inside_elbow_theta);
        }
    }
    
    //NOTE: the centripital acceleration probably makes more sense in the matrix, but this is easier to work with visually
    v2f forearm_F_other = {-string_tension*cos(string_theta)
                           +forearm_m*(forearm_cm_dist*sq(s.forearm_omega)*cos(s.forearm_theta)
                                       +shoulder_length*sq(s.shoulder_omega)*cos(s.shoulder_theta)),
                           -forearm_m*g
                           -string_tension*sin(string_theta)
                           +forearm_m*(forearm_cm_dist*sq(s.forearm_omega)*sin(s.forearm_theta)
                                       +shoulder_length*sq(s.shoulder_omega)*sin(s.shoulder_theta))};
    
    m4x5f axle_force_equations = {
        shoulder_I, 0, -shoulder_length*sin(s.shoulder_theta), shoulder_length*cos(s.shoulder_theta), shoulder_tau_other,
        0, forearm_I, -forearm_cm_dist*sin(s.forearm_theta), forearm_cm_dist*cos(s.forearm_theta), forearm_tau_other,
        -forearm_m*shoulder_length*sin(s.shoulder_theta), -forearm_m*forearm_cm_dist*sin(s.forearm_theta), -1, 0, forearm_F_other.x,
        forearm_m*shoulder_length*cos(s.shoulder_theta), forearm_m*forearm_cm_dist*cos(s.forearm_theta), 0, -1, forearm_F_other.y,
    };
    
    v4f axle_force_solutions = solve(axle_force_equations);
    assert(axle_force_solutions[0] == axle_force_solutions[0]);
    assert(axle_force_solutions[1] == axle_force_solutions[1]);
    assert(axle_force_solutions[2] == axle_force_solutions[2]);
    assert(axle_force_solutions[3] == axle_force_solutions[3]);
        
    v2f axle_force = {axle_force_solutions[2], axle_force_solutions[3]};
        
    float shoulder_alpha = axle_force_solutions[0];
        
    float forearm_alpha = axle_force_solutions[1];
    
    float winch_motor_voltage = (arm_winch_power*dc_motor_voltage - neverest_k_i*prs.winch_omega*2.0);
    float winch_motor_tau = winch_motor_voltage*neverest_k_t_over_R*2*2.0;
    float winch_tau = winch_motor_tau+string_tension*winch_pulley_r;
    float winch_alpha = winch_tau/winch_I;
    
    physical_robot_derivatives out = {s.shoulder_omega, shoulder_alpha,
                                      s.forearm_omega, forearm_alpha,
                                      s.winch_omega, winch_alpha};
    return out;
}

physical_robot_state linear_integrate(physical_robot_state s, physical_robot_derivatives d, float dt)
{
    for(int i = 0; i < sizeof(s)/sizeof(s.values[0]); i++)
    {
        s.values[i] += d.values[i]*dt;
    }
    return s;
}

physical_robot_derivatives evaluate(physical_robot_state s, physical_robot_derivatives d, float dt)
{
    s = linear_integrate(s, d, dt);
    physical_robot_derivatives o = getDerivatives(s);
    return o;
}

void updateRobot(JNIEnv * env, jobject self)
{
    
    float dt = 0.0005;
    
    float x_pos = 0;
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
            physical_robot_derivatives k_1 = getDerivatives(prs);
            physical_robot_derivatives k_2 = evaluate(prs, k_1, dt*0.5f);
            physical_robot_derivatives k_3 = evaluate(prs, k_2, dt*0.5f);
            physical_robot_derivatives k_4 = evaluate(prs, k_3, dt*1.0f);
            
            physical_robot_derivatives d;
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

#endif
