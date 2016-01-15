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
    float left_trigger;
    float right_trigger;
};

#include "arm.h"
#include "arm_test_robot_state_elements.h"
#include "meth.h"

float armHeuristic(arm_state s, float shoulder_power, float winch_power,
                   float target_shoulder_alpha, float target_winch_alpha)
{
    arm_derivatives d = getArmDerivatives(s, shoulder_power, winch_power);
    //TODO: the actual robot code uses operator overloaded
    //      vectors, should probably make the simulator match
    
    float heuristic = -sq(target_shoulder_alpha-d.shoulder_alpha)-sq(target_winch_alpha-d.winch_alpha);
    return heuristic;
}

arm_state s = {};
arm_state stable_s = {};
bool8 score_mode = true;

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    
    //TODO: make this run in a seperate thread, so it can run in a
    //      loop without the init code running every frame
    float dt = 0.005;
    do
    {
        ////////////////Control Map////////////////
        bool8 intake_mode_button = gamepad1.left_trigger > 0.5;
        bool8 score_mode_button = gamepad1.right_trigger > 0.5;
        
        #if 0
        arm_shoulder_power = gamepad2.left_stick.y;
        arm_winch_power = gamepad2.right_stick.y;
        #else
        {
            s.shoulder_omega = lerp((shoulder_encoder-s.shoulder_theta)/dt, s.shoulder_omega, exp(-0.1*dt));
            s.forearm_omega = lerp((elbow_potentiometer+shoulder_encoder-pi-s.forearm_theta)/dt, s.forearm_omega, exp(-0.1*dt));
            s.winch_omega = lerp((winch_encoder-s.winch_theta)/dt, s.winch_omega, exp(-0.1*dt));
            
            s.shoulder_theta = shoulder_encoder;
            s.forearm_theta = elbow_potentiometer+shoulder_encoder-pi;
            s.winch_theta = winch_encoder;
            
            v2f target_velocity = scale(gamepad2.right_stick, 40.0);
            if(intake_mode_button)
            {
                score_mode = false;
                stable_s.shoulder_theta = 2.75;
                stable_s.forearm_theta = pi;
                stable_s.winch_theta = 5.5;
            }
            if(score_mode_button)
            {
                score_mode = true;
                stable_s.shoulder_theta = 0.75;
                stable_s.forearm_theta = 0;
                stable_s.winch_theta = 1;
            }

            float inside_elbow_theta = s.forearm_theta+(pi-s.shoulder_theta);
            bool8 winch_mode = inside_elbow_theta < (acos((elbow_pulley_r-shoulder_pulley_r)/shoulder_length)+acos(elbow_pulley_r/forearm_length));                        
            if(normsq(target_velocity) > 0.0)
            {
                stable_s = s;
                
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
            else
            {
                if(!score_mode)
                {
                    float target_shoulder_alpha = -10000*s.shoulder_omega;
                    float target_winch_alpha = -10000*s.forearm_omega;
                    
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

                    arm_winch_power += 1*(stable_s.winch_theta-s.winch_theta);//-s.winch_omega;
                    arm_shoulder_power += 5*(stable_s.shoulder_theta-s.shoulder_theta);//-s.shoulder_omega;
                }
                else
                {
                    arm_winch_power = 10*(stable_s.winch_theta-s.winch_theta);
                    arm_shoulder_power = 50*(stable_s.shoulder_theta-s.shoulder_theta);
                }
                

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
