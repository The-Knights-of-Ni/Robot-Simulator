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

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    
    float x_pos = 0.0;
    
    //TODO: make this run in a seperate thread, so it can run in a
    //      loop without the init code running every frame
    do
    {
        ////////////////Control Map////////////////
        #if 0
        arm_shoulder_power = gamepad2.left_stick.y;
        arm_winch_power = gamepad2.right_stick.y;
        #else
        //TODO: treat it like it's from an encoder
        {
            arm_state s = prs;
            v2f target_velocity_dir = gamepad2.right_stick;
            if(normsq(target_velocity_dir) > 0.0)
            {
                //TODO: possibly itterate over a smaller and smaller region for better
                //performance and precission

                float w_step = 0.02;
                float s_step = 0.02;
                //loop over a range of motor powers to find the closest one
                v2f linear_velocity = {
                    //x
                    -shoulder_length*s.shoulder_omega*sin(s.shoulder_theta)
                    -forearm_length*s.forearm_omega*sin(s.forearm_theta)
                    //y
                    +shoulder_length*s.shoulder_omega*cos(s.shoulder_theta)
                    +forearm_length*s.forearm_omega*cos(s.forearm_theta)
                };
                
                float linear_speed = norm(linear_velocity);
                v2f target_linear_velocity = normalizeScale(target_velocity_dir, linear_speed);
                v2f target_acceleration_dir = sub(target_linear_velocity, linear_velocity);

                #define projection_weight 1
                #define rejection_weight 1
                
                float best_heuristic = 0;
                float best_shoulder_power = 0;
                float best_winch_power = 0;
                //TODO: this is really SIMDable/multithreadable if it's running slowly
                for(float winch_power = -1.0; winch_power < 1.0; winch_power += w_step)
                {
                    for(float shoulder_power = -1.0; shoulder_power < 1.0; shoulder_power += w_step)
                    {
                        arm_derivatives d = getArmDerivatives(s, shoulder_power, winch_power);
                        //TODO: the actual robot code uses operator overloaded
                        //      vectors, should probably make simulator match
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
                        
                        float heuristic =
                            projection_weight*projDist(linear_acceleration, target_acceleration_dir)
                            - rejection_weight*rejDist(linear_acceleration, target_acceleration_dir);
                        
                        if((winch_power == -1.0 && shoulder_power == -1.0) || heuristic > best_heuristic)
                        {
                            best_heuristic = heuristic;
                            best_shoulder_power = shoulder_power;
                            best_winch_power = winch_power;
                        }
                    }
                }
                arm_shoulder_power = best_winch_power;
                arm_winch_power = best_winch_power;
            }
            else
            {
                //TODO: remember and hold position if joystick is 0
            }
        }
        /* arm_shoulder_power = -gamepad2.right_stick.x; */
        /* arm_winch_power = gamepad2.right_stick.y; */
        #endif
        ///////////////////////////////////////////
    } while(updateRobot(env, self) != 0);
    cleanupJNI(env, self);
}
