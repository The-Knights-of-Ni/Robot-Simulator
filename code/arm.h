#ifndef ROBOT_ARM //because it is possible that some compilers will have ARM predefined for the chip archutecture
#define ROBOT_ARM

//TODO: tune values
float g = 384; //gravity in " per sec

float winch_pulley_r = 1.0f;
float shoulder_pulley_r = 1.0f;
float elbow_pulley_r = 2.0f;

float winch_I = 1000.0*10; //the rotational inertia of the forearm
float winch_gear_ratio = 2.0;

float shoulder_I = 81.0f*80; //the rotational inertia of the shoulder
float shoulder_m = 80.0f;
float shoulder_gear_ratio = 6.75;

float forearm_I = 81.0f*20; //the rotational inertia of the forearm
float forearm_m = 200.0f;

float forearm_length = 11.5f;
float forearm_cm_dist = 6.0f;
float shoulder_length = 16.5f;
float shoulder_cm_dist = 9.0f;

float spring_force = 2*2802000;

float k_string = 10000000;

float dc_motor_voltage = 14.0f;

float neverest_max_torque = 4334000; //in g in^2/s^2
float neverest_max_speed = 13.51; //in rad/s

float neverest_k_i = dc_motor_voltage/neverest_max_speed;
float neverest_k_t_over_R = neverest_max_torque/dc_motor_voltage;

float string_length_0 = 7;

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
        -(s.winch_omega>0.1 ? 1000000 :
          (s.winch_omega<-0.1 ?-1000000 : 0));
    float winch_alpha = winch_tau/winch_I;

    //TEMP
    #if 0
    s.shoulder_omega = shoulder_power/shoulder_gear_ratio*neverest_max_speed;
    s.winch_omega = winch_power/winch_gear_ratio*neverest_max_speed;
    #endif
    //END TEMP
    
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

float filterArmJoystick(float a)
{
    a = deadzoneAdjust(a);
    if(a == 0) return 0;
    return cubicBezier(a*0.5+0.5, -1.0, 0.0, 0.0, 1.0);
}

void armAtVelocity(float & out_shoulder_power, float & out_winch_power,
                   float & target_arm_theta, float & target_inside_elbow_theta, v2f target_velocity,
                   float shoulder_theta, float inside_elbow_theta, float shoulder_omega, float dt)
{
    bool8 winch_mode = inside_elbow_theta < (acos((elbow_pulley_r-shoulder_pulley_r)/shoulder_length)+acos(elbow_pulley_r/forearm_length));
    
    ////////////////////////////////////////////////////
    float string_moment_arm;
    
    if(winch_mode)
    {
        float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        
        float string_theta =
            asin(shoulder_length/shoudler_axis_to_end*sin(inside_elbow_theta))
            +asin(shoulder_pulley_r/shoudler_axis_to_end);
        
        string_moment_arm = forearm_length*sin(string_theta);
    }
    else
    {
        string_moment_arm = elbow_pulley_r;
    }
    
    //TODO: fix divides by zero
    float target_winch_omega = 0;
    
    target_velocity.y = filterArmJoystick(target_velocity.y);
    if(target_velocity.y != 0)
    {
        target_winch_omega = target_velocity.y;
        target_inside_elbow_theta = inside_elbow_theta;
    }
    
    float inside_elbow_omega = target_winch_omega*winch_pulley_r/string_moment_arm;
        
    float shoudler_axis_to_end_sq = sq(forearm_length)+sq(shoulder_length)
        -2*forearm_length*shoulder_length*cos(inside_elbow_theta);
    float dshoudler_axis_to_end_sq =
        2*forearm_length*shoulder_length*sin(inside_elbow_theta)*inside_elbow_omega;
    
    float target_shoulder_omega =
        +(  invSqrt(1-sq(forearm_length/sqrt(shoudler_axis_to_end_sq)*sin(inside_elbow_theta)))
            *(  forearm_length/sqrt(shoudler_axis_to_end_sq)*cos(inside_elbow_theta)*inside_elbow_omega
                -1.0/2.0*forearm_length*pow(shoudler_axis_to_end_sq, -3.0/2.0)*dshoudler_axis_to_end_sq*sin(inside_elbow_theta)))
        +1/2*invSqrt(1-sq(shoulder_pulley_r*invSqrt(shoudler_axis_to_end_sq)))*shoulder_pulley_r*pow(shoudler_axis_to_end_sq, -3.0/2.0)*dshoudler_axis_to_end_sq;

    target_velocity.x = filterArmJoystick(target_velocity.x);
    if(target_velocity.x != 0)
    {
        float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
        
        target_shoulder_omega += target_velocity.x;
    }
    
    /* if(inside_elbow_theta > pi-0.3) */
    /* { */
    /*     target_winch_omega += 60*(pi-0.3-inside_elbow_theta); */
    /* } */
    /* if(inside_elbow_theta > pi-0.25) */
    /* { */
    /*     //target_winch_omega -= 1000; */
    /*     target_shoulder_omega = target_velocity.x; */
    /*     target_winch_omega = 60*(pi-0.25-inside_elbow_theta)-target_shoulder_omega; */
    /* } */
    
    out_winch_power = target_winch_omega+target_shoulder_omega;
    out_shoulder_power =
        shoulder_gear_ratio/winch_gear_ratio
        *(target_shoulder_omega)/* +1.06 */;
    
    //clamp while keeping ratio constant
    if(out_winch_power > 1.0)
    {
        out_shoulder_power *= 1.0/out_winch_power;
        out_winch_power = 1.0;
    }
    if(out_winch_power < -1.0)
    {
        out_shoulder_power *= -1.0/out_winch_power;
        out_winch_power = -1.0;
    }
    if(out_shoulder_power > 1.0)
    {
        out_winch_power *= 1.0/out_shoulder_power;
        out_shoulder_power = 1.0;
    }
    if(out_shoulder_power < -1.0)
    {
        out_winch_power *= -1.0/out_shoulder_power;
        out_shoulder_power = -1.0;
    }
}

void armJointsAtVelocity(float & out_shoulder_power, float & out_winch_power,
                         float & target_shoulder_theta, float & target_inside_elbow_theta, v2f target_velocity,
                         float shoulder_theta, float inside_elbow_theta, float shoulder_omega, float dt)
{
    //TODO: this should probably be a different function
    float target_winch_omega = 0;

    target_velocity.y = filterArmJoystick(target_velocity.y);
    if(target_velocity.y != 0)
    {
        target_winch_omega = target_velocity.y;
        target_inside_elbow_theta = inside_elbow_theta;
    }
    
    float target_shoulder_omega = 0;
    
    target_velocity.x = filterArmJoystick(target_velocity.x);
    if(target_velocity.x != 0) //TODO: make the deadzone a constant
    {
        target_shoulder_theta = shoulder_theta;
        
        target_shoulder_omega = target_velocity.x;
    }
    
    out_winch_power = target_winch_omega+target_shoulder_omega;
    out_shoulder_power =
        shoulder_gear_ratio/winch_gear_ratio
        *(target_shoulder_omega);
}

void armToAngle(float & out_shoulder_power, float & out_winch_power,
                float target_arm_theta, float target_inside_elbow_theta,
                float shoulder_theta,   float inside_elbow_theta,
                bool8 score_mode, float dt)
{
    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
    
    float arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
    
    out_shoulder_power += 10*(target_arm_theta-arm_theta);
    out_winch_power += 10*(target_inside_elbow_theta-inside_elbow_theta);
}

void armJointsToAngle(float & out_shoulder_power, float & out_winch_power,
                      float target_shoulder_theta, float target_inside_elbow_theta,
                      float shoulder_theta,        float inside_elbow_theta,
                      bool8 score_mode, float dt)
{
    out_shoulder_power += 10*(target_shoulder_theta-shoulder_theta);
    out_winch_power += 10*(target_inside_elbow_theta-inside_elbow_theta);
}

/*
  inputs: hand[0] and hand[1] are the coordinates of the wanted hand position in inches, relative to the robot
  hand will be clamped if it is outside of the range of motion of the arm
*/
v2f getArmTargetsRectangular(v2f hand, bool mode)
{
    v2f arm_targets; //0 -> shoulder, 1 -> elbow

    float dist = norm(hand);
    //clamp hand motion
    if (dist > forearm_length + shoulder_length)
    {
        hand * ((0.9 * (forearm_length + shoulder_length)) * invSqrt(dist));
    }
    if (dist < forearm_length - shoulder_length)
    {
        hand * ((0.9 * (forearm_length - shoulder_length)) * invSqrt(dist));
    }
    //TODO: clamp when the arm will hit the frame

    arm_targets[0] = atan2(hand[1], hand[0]);
    if (arm_targets[0] < 0.0f)
        arm_targets[0] += 2.0f * pi;

    float shoulder_offset = acos((sq(shoulder_length) + sq(dist) - sq(forearm_length)) / (2.0f * dist * shoulder_length));
    //from law of cosines, forearm_length^2 = shoulder_length^2 + dist^2 - 2*dist*shoulder_length*cos(shoulder_offset)

    arm_targets[1] = acos(
        (sq(shoulder_length) + sq(forearm_length) - sq(dist)) / (2.0f * shoulder_length * forearm_length));
    if (mode)//shoulder_target < shoulder_max)
    { //pulley case

        arm_targets[0] = arm_targets[0] - shoulder_offset;
        arm_targets[1] = 2.0f * pi - arm_targets[1];
    }
    else
    { //winch case
        arm_targets[0] = arm_targets[0] + shoulder_offset;
    }
    /*
      outputs: arm_target[1] and arm_targets[0] are the rotations of
      the elbow(from the potentiometer) and shoulder outputs in radians, respectively
    */
    return arm_targets;
}

/*
  inputs: hand[0] and hand[1] are the polar coordinates of the wanted hand position in inches and radians, relative to the robot
  hand will be clamped if it is outside of the range of motion of the arm
*/
v2f getArmTargetsPolar(v2f hand, bool mode)
{
    v2f arm_targets; //[0] -> shoulder, [1] -> elbow

    //clamp hand motion
    if (hand[0] > forearm_length + shoulder_length)
        hand[0] = forearm_length + shoulder_length;
    if (hand[0] < fabs(forearm_length - shoulder_length))
        hand[0] = fabs(forearm_length - shoulder_length);
    float dist = hand[0];
    //TODO: clamp when the arm will hit the frame

    float shoulder_offset = acos(
        (sq(shoulder_length) + sq(dist) - sq(forearm_length)) / (2.0f * dist * shoulder_length));
    //from law of cosines, forearm_length^2 = shoulder_length^2 + dist^2 - 2*dist*shoulder_length*cos(shoulder_offset)

    arm_targets[1] = acos((sq(shoulder_length) + sq(forearm_length) - sq(dist)) / (2.0f * shoulder_length * forearm_length));
    if (mode)//shoulder_target < shoulder_max)
    { //pulley case
        arm_targets[0] = hand[1] - shoulder_offset;
        arm_targets[1] = 2.0f * pi- arm_targets[1];
    }
    else
    { //winch case
        arm_targets[0] = hand[1] + shoulder_offset;
    }

    /*
      outputs: arm_targets[1] and arm_targets[0] are the rotations of
      the elbow(from the potentiometer) and shoulder outputs in radians, respectively
    */
    return arm_targets;
}
#endif
