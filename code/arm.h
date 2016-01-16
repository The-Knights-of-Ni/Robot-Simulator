#ifndef ROBOT_ARM //because it is possible that some compilers will have ARM predefined for the chip archutecture
#define ROBOT_ARM

union arm_derivatives
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

union arm_state
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

arm_derivatives getArmDerivatives(arm_state s, float shoulder_power, float winch_power)
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
    
    float shoulder_motor_voltage = (shoulder_power*dc_motor_voltage - neverest_k_i*s.shoulder_omega*shoulder_gear_ratio);
    float shoulder_motor_tau = shoulder_motor_voltage*neverest_k_t_over_R*shoulder_gear_ratio;
    
    //TODO: add friction to winch
    //all of the torques not including the axle
    
    float shoulder_tau_other = shoulder_motor_tau - shoulder_cm_dist*shoulder_m*g*cos(s.shoulder_theta)
        -spring_force*elbow_pulley_r
        -(s.shoulder_omega>0.1 ? 100000 :
          (s.shoulder_omega<-0.1 ?-100000:0))
        +(s.forearm_omega-s.shoulder_omega>0.1 ? 100000 :
          (s.forearm_omega-s.shoulder_omega<-0.1 ?-100000:0));
    
    //TODO: add shoulder stop
    if(s.shoulder_theta < 0)
    {
        shoulder_tau_other -= 1000000000*s.shoulder_theta;
    }
    if(s.shoulder_theta > pi)
    {
        shoulder_tau_other -= 1000000000*(s.shoulder_theta-pi);
    }
    
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
    
    float winch_motor_voltage = (winch_power*dc_motor_voltage - neverest_k_i*s.winch_omega*winch_gear_ratio);
    float winch_motor_tau = winch_motor_voltage*neverest_k_t_over_R*2*winch_gear_ratio;
    float winch_tau = winch_motor_tau+string_tension*winch_pulley_r
        -(s.winch_omega>0.1 ? 100000 :
          (s.winch_omega<-0.1 ?-100000 : 0));
    float winch_alpha = winch_tau/winch_I;
    
    arm_derivatives out = {s.shoulder_omega, shoulder_alpha,
                                      s.forearm_omega, forearm_alpha,
                                      s.winch_omega, winch_alpha};
    return out;
}

float armHeuristic(arm_state s, float shoulder_power, float winch_power,
                   float target_shoulder_alpha, float target_winch_alpha)
{
    arm_derivatives d = getArmDerivatives(s, shoulder_power, winch_power);
    //TODO: the actual robot code uses operator overloaded
    //      vectors, should probably make the simulator match
    
    float heuristic = -sq(target_shoulder_alpha-d.shoulder_alpha)-sq(target_winch_alpha-d.winch_alpha);
    return heuristic;
}

void armAtVelocity(v2f target_velocity, arm_state s, bool8 score_mode, float dt)
{
    float inside_elbow_theta = s.forearm_theta+(pi-s.shoulder_theta);
    bool8 winch_mode = inside_elbow_theta < (acos((elbow_pulley_r-shoulder_pulley_r)/shoulder_length)+acos(elbow_pulley_r/forearm_length));
    
    ////////////////////////////////////////////////////
    float string_moment_arm;
    
    if(winch_mode)
    {
        float shoudler_axis_to_end_sq = sq(forearm_length)+sq(shoulder_length)
            -2*forearm_length*shoulder_length*cos(inside_elbow_theta);
        
        float string_theta = s.shoulder_theta
            -asin(forearm_length/sqrt(shoudler_axis_to_end_sq)*sin(inside_elbow_theta))
            +asin(shoulder_pulley_r/sqrt(shoudler_axis_to_end_sq));
        
        string_moment_arm = forearm_length*sin(string_theta-s.forearm_theta);
    }
    else
    {
        string_moment_arm = elbow_pulley_r;
    }
    
    float target_forearm_omega;
    float target_shoulder_omega;
    
    //TODO: fix divides by zero
    if(score_mode)
    {
        if(inside_elbow_theta < pi-0.1)
        {
            target_forearm_omega =
                (target_velocity.x/sin(s.shoulder_theta)+target_velocity.y/cos(s.shoulder_theta))
                /(forearm_length*(-sin(s.forearm_theta)/sin(s.shoulder_theta)+cos(s.forearm_theta)/cos(s.shoulder_theta)));
            target_shoulder_omega =
                (target_velocity.x/sin(s.forearm_theta)+target_velocity.y/cos(s.forearm_theta))
                /(shoulder_length*(-sin(s.shoulder_theta)/sin(s.forearm_theta)+cos(s.shoulder_theta)/cos(s.forearm_theta)));
        }
        else
        {
            target_forearm_omega =
                -target_velocity.x*sin(s.shoulder_theta)+target_velocity.y*cos(s.shoulder_theta);
            target_shoulder_omega =
                -target_velocity.x*sin(s.shoulder_theta)+target_velocity.y*cos(s.shoulder_theta);
            target_forearm_omega -= 10;
            target_shoulder_omega += 0;
        }
    }
    else
    {
        if(inside_elbow_theta > pi+0.1)
        {
            target_forearm_omega =
                (target_velocity.x/sin(s.shoulder_theta)+target_velocity.y/cos(s.shoulder_theta))
                /(forearm_length*(-sin(s.forearm_theta)/sin(s.shoulder_theta)+cos(s.forearm_theta)/cos(s.shoulder_theta)));
            target_shoulder_omega =
                (target_velocity.x/sin(s.forearm_theta)+target_velocity.y/cos(s.forearm_theta))
                /(shoulder_length*(-sin(s.shoulder_theta)/sin(s.forearm_theta)+cos(s.shoulder_theta)/cos(s.forearm_theta)));
        }
        else
        {
            target_forearm_omega =
                -target_velocity.x*sin(s.shoulder_theta)+target_velocity.y*cos(s.shoulder_theta);
            target_shoulder_omega =
                -target_velocity.x*sin(s.shoulder_theta)+target_velocity.y*cos(s.shoulder_theta);
            target_forearm_omega += 10;
            target_shoulder_omega -= 10;
        }
    }
    
    arm_winch_power =
        winch_gear_ratio/neverest_max_speed
        *((target_forearm_omega-target_shoulder_omega)*string_moment_arm/winch_pulley_r
          +target_shoulder_omega);
    arm_shoulder_power =
        shoulder_gear_ratio/neverest_max_speed
        *target_shoulder_omega;
    
    //clamp while keeping ratio constant
    if(arm_winch_power > 1.0)
    {
        arm_shoulder_power *= 1.0/arm_winch_power;
        arm_winch_power = 1.0;
    }
    if(arm_winch_power < -1.0)
    {
        arm_shoulder_power *= -1.0/arm_winch_power;
        arm_winch_power = -1.0;
    }
    if(arm_shoulder_power > 1.0)
    {
        arm_winch_power *= 1.0/arm_shoulder_power;
        arm_shoulder_power = 1.0;
    }
    if(arm_shoulder_power < -1.0)
    {
        arm_winch_power *= -1.0/arm_shoulder_power;
        arm_shoulder_power = -1.0;
    }
}

void armToState(arm_state stable_s, arm_state s, bool8 score_mode, float dt)
{
    if(!score_mode)
    {
        float target_shoulder_alpha = 0.0;//-10000*s.shoulder_omega+2000*(stable_s.shoulder_theta-s.shoulder_theta);
        float target_winch_alpha = 0.0;//-10000*s.forearm_omega+10*(stable_s.winch_theta-s.winch_theta);
        
        float winch_power_step = 0.1;
        float shoulder_power_step = 0.1;
        
        float best_heuristic;
        float best_shoulder_power;
        float best_winch_power;
        {
            float shoulder_power = 0.0;
            float winch_power = 0.0;
            
            float heuristic = armHeuristic(s, shoulder_power, winch_power, target_shoulder_alpha, target_winch_alpha);
            
            best_heuristic = heuristic;
            best_shoulder_power = shoulder_power;
            best_winch_power = arm_winch_power;
        }
        
        //TODO: this is really SIMDable/multithreadable
        for(float winch_power = -1.0; winch_power < 1.0; winch_power += winch_power_step)
        {
            for(float shoulder_power = -1.0; shoulder_power < 1.0; shoulder_power += shoulder_power_step)
            {
                float heuristic = armHeuristic(s, shoulder_power, winch_power, target_shoulder_alpha, target_winch_alpha);
                if(heuristic > best_heuristic)
                {
                    best_heuristic = heuristic;
                    best_shoulder_power = shoulder_power;
                    best_winch_power = winch_power;
                }
            }
        }
        arm_shoulder_power = lerp(best_shoulder_power, arm_shoulder_power, exp(-100*dt));
        arm_winch_power = lerp(best_winch_power, arm_winch_power, exp(-100*dt));
        
        arm_shoulder_power += 5*(stable_s.shoulder_theta-s.shoulder_theta);//-s.shoulder_omega;
        arm_winch_power += 1*(stable_s.winch_theta-s.winch_theta);//-s.winch_omega;
    }
    else
    {
        arm_winch_power = 10*(stable_s.winch_theta-s.winch_theta);
        arm_shoulder_power = 50*(stable_s.shoulder_theta-s.shoulder_theta);
    }
}
#endif
