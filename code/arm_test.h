/*
  robot_state_elements
  {
  gamepad{float joystick_1_x, float joystick_1_y, float joystick_2_x, float joystick_2_y, float left_trigger, float right_trigger};
  
  float arm_winch_power;
  float arm_shoulder_power;
  
  gamepad gamepad1;
  }
 */

struct gamepad
{
    v2f left_stick;
    v2f right_stick;
    float left_trigger;
    float right_trigger;
};

#include "arm_test_robot_state_elements.h"
//TODO: add include guards to generator so arm_test_robot_state_elements.h can be included in arm.h
#include "arm.h"
#include "meth.h"

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
        v2f target_velocity = gamepad2.right_stick*40.0;
        #endif
        ///////////////////////////////////////////
        
        s.shoulder_omega = lerp((shoulder_encoder-s.shoulder_theta)/dt, s.shoulder_omega, exp(-0.1*dt));
        s.forearm_omega = lerp((elbow_potentiometer+shoulder_encoder-pi-s.forearm_theta)/dt, s.forearm_omega, exp(-0.1*dt));
        s.winch_omega = lerp((winch_encoder-s.winch_theta)/dt, s.winch_omega, exp(-0.1*dt));
        
        s.shoulder_theta = shoulder_encoder;
        s.forearm_theta = elbow_potentiometer+shoulder_encoder-pi;
        s.winch_theta = winch_encoder;
        
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
        
        if(normSq(target_velocity) > 1.0)
        {
            stable_s = s;
            armAtVelocity(target_velocity, s, score_mode, dt);
        }
        else
        {
            armToState(stable_s, s, score_mode, dt);
        }
        
        arm_shoulder_power = clamp(arm_shoulder_power, -1.0, 1.0);
        arm_winch_power = clamp(arm_winch_power, -1.0, 1.0);
    } while(updateRobot(env, self) != 0);
    cleanupJNI(env, self);
}
