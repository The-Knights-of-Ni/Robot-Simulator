/*
  robot_state_elements
  {
  gamepad{float joystick_1_x, float joystick_1_y, float joystick_2_x, float joystick_2_y};
  
  float arm_winch_power;
  float arm_shoulder_power;
  
  gamepad gamepad1;
  }
 */

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

float forearm_length = 17.0f;
float forearm_cm_dist = 9.0f;
float shoulder_length = 16.5f;
float shoulder_cm_dist = 9.0f;

float spring_force = 2*2802000;

float k_string = 10000000;

float dc_motor_voltage = 14.0f;

float neverest_max_torque = 4334000; //in g in^2/s^2
float neverest_max_speed = 13.51; //in rad/s

float neverest_k_i = dc_motor_voltage/neverest_max_speed;
float neverest_k_t_over_R = neverest_max_torque/dc_motor_voltage;

float string_length_0 = 29.908846+1;

struct gamepad
{
    v2f left_stick;
    v2f right_stick;
};

#include "arm.h"
#include "arm_test_robot_state_elements.h"
#include "meth.h"

float armHeuristic(arm_state s, float shoulder_power, float winch_power, v2f target_acceleration, float r, float dr, float theta_off, float dtheta)
{
    arm_derivatives d = getArmDerivatives(s, shoulder_power, winch_power);
    //TODO: the actual robot code uses operator overloaded
    //      vectors, should probably make the simulator match
    
    v2f linear_acceleration = {
        //x
        -shoulder_length*d.shoulder_alpha*sin(s.shoulder_theta) //tangential acceleration
        -shoulder_length*sq(d.shoulder_omega)*cos(s.shoulder_theta) //centripital acceleration
        -forearm_length*d.forearm_alpha*sin(s.forearm_theta)
        -forearm_length*sq(d.forearm_omega)*cos(s.forearm_theta),
        //y
        +shoulder_length*d.shoulder_alpha*cos(s.shoulder_theta) //tangential acceleration
        -shoulder_length*sq(d.shoulder_omega)*sin(s.shoulder_theta) //centripital acceleration
        +forearm_length*d.forearm_alpha*cos(s.forearm_theta)
        -forearm_length*sq(d.forearm_omega)*sin(s.forearm_theta)
    };
    
    float ddr = //TODO: fix for chain rule
        + 2*shoulder_length*forearm_length*sin(pi+s.forearm_theta-s.shoulder_theta)*(d.forearm_alpha-d.shoulder_alpha)
        + 2*shoulder_length*forearm_length*cos(pi+s.forearm_theta-s.shoulder_theta)*sq(d.forearm_omega-d.shoulder_omega);
    
    float ddtheta = dot((v2f){-sin(s.shoulder_theta-theta_off), cos(s.shoulder_theta-theta_off)},
                            linear_acceleration);
    
    bool8 winch_mode = s.forearm_theta+pi-s.shoulder_theta < (acos((elbow_pulley_r-shoulder_pulley_r)/shoulder_length)+acos(elbow_pulley_r/forearm_length));
    
    v2f polar_acceleration = {
        //r
        winch_mode ? ((d.winch_alpha-d.shoulder_alpha)*winch_pulley_r) : ddr,
        //theta
        ddtheta
    };
    
    //target_acceleration.y = -(d.shoulder_omega-dtheta_off);
    
    float heuristic =
        //-100*fabs(0.0005*d.shoulder_alpha*d.shoulder_omega)-fabs(0.0005*d.winch_alpha+d.winch_omega);
        /* dot(polar_acceleration, */
        /*     (v2f){10*(1+target_acceleration.x)-r-d.winch_omega, */
        /*             1+target_acceleration.y-s.shoulder_theta+theta_off -0.1*(d.shoulder_omega+dr)}); */
        -sq(polar_acceleration.x-target_acceleration.x)-100*sq(polar_acceleration.y-target_acceleration.y);
        //-normsq(sub(polar_acceleration, target_acceleration));
        //dot(polar_acceleration, target_acceleration);
    return heuristic;
}

arm_state s = {};

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    
    float x_pos = 0.0;
    
    //TODO: make this run in a seperate thread, so it can run in a
    //      loop without the init code running every frame
    float dt = 0.005;
    do
    {
        ////////////////Control Map////////////////
        #if 0
        arm_shoulder_power = gamepad2.left_stick.y;
        arm_winch_power = gamepad2.right_stick.y;
        #else
        //TODO: treat it like it's from an encoder
        {
            s.shoulder_omega = lerp((shoulder_encoder-s.shoulder_theta)/dt, s.shoulder_omega, exp(-10*dt));
            s.forearm_omega = lerp((elbow_potentiometer+shoulder_encoder-pi-s.forearm_theta)/dt, s.forearm_omega, exp(-10*dt));
            s.winch_omega = lerp((winch_encoder-s.winch_theta)/dt, s.winch_omega, exp(-10*dt));
            
            s.shoulder_theta = shoulder_encoder;
            s.forearm_theta = elbow_potentiometer+shoulder_encoder-pi;
            s.winch_theta = winch_encoder;
            
            v2f target_velocity = gamepad2.right_stick;
            if(normsq(target_velocity) > 0.0)
            {
                //TODO: possibly itterate over a smaller and smaller region for better
                //performance and precission

                float winch_power_step = 0.1;
                float shoulder_power_step = 0.1;
                //loop over a range of motor powers to find the closest one
                
                v2f linear_velocity = {
                    //x
                    -shoulder_length*s.shoulder_omega*sin(s.shoulder_theta)
                    -forearm_length*s.forearm_omega*sin(s.forearm_theta)
                    //y
                    +shoulder_length*s.shoulder_omega*cos(s.shoulder_theta)
                    +forearm_length*s.forearm_omega*cos(s.forearm_theta)
                };
                
                float r = sqrt(sq(shoulder_length)+sq(forearm_length)-2*shoulder_length*forearm_length*cos(pi+s.forearm_theta-s.shoulder_theta));
                float dr =
                    1/2*invsqrt(sq(shoulder_length)+sq(forearm_length)-2*shoulder_length*forearm_length*cos(pi+s.forearm_theta-s.shoulder_theta))
                    *2*shoulder_length*forearm_length*sin(pi+s.forearm_theta-s.shoulder_theta)*(s.forearm_omega-s.shoulder_omega);
                    
                float theta_off = asin(forearm_length/r*sin(pi+s.forearm_theta-s.shoulder_theta));
                float dtheta = dot((v2f){-sin(s.shoulder_theta-theta_off), cos(s.shoulder_theta-theta_off)},
                                       linear_velocity);
                
                v2f polar_velocity = {
                    //r
                    dr,
                    //theta
                    dtheta
                };

                v2f target_polar_velocity = {
                    target_velocity.x*cos(s.shoulder_theta)-target_velocity.y*sin(s.shoulder_theta),
                    target_velocity.x*sin(s.shoulder_theta)+target_velocity.y*cos(s.shoulder_theta)
                };
                
                //TODO: properly convert polar speeds
                target_polar_velocity = scale(target_polar_velocity, 100);
                //TODO: clamp acceleration based on frame rate so it doesn't overshoot
                v2f target_acceleration = scale(sub(target_polar_velocity, polar_velocity), 100);
                
                float best_heuristic;
                float best_shoulder_power;
                float best_winch_power;
                {
                    float winch_power = 0.0;
                    float shoulder_power = 0.0;
                    
                    float heuristic = armHeuristic(s, shoulder_power, winch_power, target_acceleration,
                                                   r, dr, theta_off, dtheta);
                    best_heuristic = heuristic;
                    best_shoulder_power = shoulder_power;
                    best_winch_power = winch_power;
                }
                //TODO: this is really SIMDable/multithreadable
                for(float winch_power = -1.0; winch_power < 1.0; winch_power += winch_power_step)
                {
                    for(float shoulder_power = -1.0; shoulder_power < 1.0; shoulder_power += shoulder_power_step)
                    {
                        float heuristic = armHeuristic(s, shoulder_power, winch_power, target_acceleration,
                                                       r, dr, theta_off, dtheta);
                        if(heuristic > best_heuristic)
                        {
                            best_heuristic = heuristic;
                            best_shoulder_power = shoulder_power;
                            best_winch_power = winch_power;
                        }
                    }
                }
                arm_shoulder_power = best_shoulder_power;
                arm_winch_power = best_winch_power;
                
                /* arm_winch_power += 10*(10*(target_velocity.x-0.5)-(s.winch_theta-s.shoulder_theta)*winch_pulley_r) */
                /*     -0*s.winch_omega*winch_pulley_r; */
                /* arm_shoulder_power += 3*(1+target_velocity.y-s.shoulder_theta+theta_off) -0.1*(s.shoulder_omega+dr); */
            }
            else
            {
                arm_shoulder_power = 0;
                arm_winch_power = 0;
                //TODO: remember and hold position if joystick is 0
            }
        }
        /* arm_shoulder_power = -gamepad2.right_stick.x; */
        /* arm_winch_power = gamepad2.right_stick.y; */
        #endif
        ///////////////////////////////////////////
        
        arm_shoulder_power = clamp(arm_shoulder_power, -1.0, 1.0);
        arm_winch_power = clamp(arm_winch_power, -1.0, 1.0);
    } while(updateRobot(env, self) != 0);
    cleanupJNI(env, self);
}
