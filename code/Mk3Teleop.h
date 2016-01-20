//This is not a comment
/*
  robot_state_elements
  {
  gamepad{float joystick1_x, float joystick1_y, float joystick2_x, float joystick2_y, float trigger1, float trigger2, int buttons};
  
  double time;
  
  int right_drive_encoder;
  int left_drive_encoder;
  int winch_encoder;
  int shoulder_encoder;
  int elbow_potentiometer;
  float heading;
  float tilt;
  float roll;
  float x_velocity;
  float y_velocity;
  
  float left_drive;
  float right_drive;
  float winch;
  float shoulder;
  float intake;
  float hand;
  float slide;

  //float hand_print_position; //for checking servo values
  float shoulder_print_theta;
  float forearm_print_theta;
  
  gamepad gamepad1;
  gamepad gamepad2;
  
  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

#include "drive.h"
#include "arm.h"
#include "Button.h"

#include "Mk3Teleop_robot_state_elements.h"

//TODO: Get RED/BLUE Status
#define current_color 0 //0 = red, 1 = blue

#define slide_rotations 20 //TODO: Move this to <robotname>.h
float hand_blue_position = 0.3;
float hand_red_position = 0.9;
float hand_level_position = 0.6;
#define slide_speed 5
#define slide_blue_position 1
#define slide_red_position -1
#define slide_stored_position 0
//KEYBINDS

//Drive
#define drive_stick pad1stick1

//Hopper
#define intake_toggle pad1.toggle(LEFT_BUMPER)
#define intake_reverse pad1.press(RIGHT_BUMPER)
#define hopper_tilt pad2.toggle(RIGHT_BUMPER) //Might make this a stick or something

//Arm
#define shoulder_manual pad2stick1
#define elbow_manual pad2stick2
#define arm_stick ((v2f){-pad2stick1.y, pad2stick2.y})
#define arm_manual_toggle pad2.toggle(Y)
#define arm_score_mode_button pad2.press(DPAD_UP)
#define arm_intake_mode_button pad2.press(DPAD_DOWN)
#define precision_mode pad2.toggle(A)

//Slide
#define slide_toggle pad1.toggle(Y)
#define slide_right pad1.press(DPAD_RIGHT)
#define slide_left pad1.press(DPAD_LEFT)

Button pad1 = {};
Button pad2 = {};
v2f pad1stick1;
v2f pad1stick2;
v2f pad2stick1;
v2f pad2stick2;
    
float shoulder_theta = 0;
float inside_elbow_theta = 0;
float winch_theta = 0;
bool8 score_mode = true;
float shoulder_omega = 0.0;
float winch_omega = 0.0;

float target_arm_theta;
float target_shoulder_theta;
float target_inside_elbow_theta;

float dt;
float old_time = time;
    
float elbow_potentiometer_angle = 0.0;

//just because the simulator doesn't prefectly treat it like real code yet
void simulatorStartMain()
{
    shoulder_omega = 0;
    target_arm_theta = pi*7/12;
    target_shoulder_theta = pi*150/180;
    target_inside_elbow_theta = pi/6;
}

extern "C"
void JNI_main(JNIEnv * _env, jobject _self)
{
    env = _env;
    self = _self;
    
    initJNI();
    
    waitForStart();
    
    do
    {
        dt = time-old_time;
        if(dt == 0) break;//SIMULATOR THING
        old_time = time;
        
//============================ Controls ==========================
        
        pad1stick1.x = gamepad1.joystick1.x; pad1stick1.y = gamepad1.joystick1.y;
        pad1stick2.x = gamepad1.joystick2.x; pad1stick2.y = gamepad1.joystick2.y;
        pad2stick1.x = gamepad2.joystick1.x; pad2stick1.y = gamepad2.joystick1.y;
        pad2stick2.x = gamepad2.joystick2.x; pad2stick2.y = gamepad2.joystick2.y;
        
//============================= Drive ============================
        deadZone(drive_stick);
        //smoothJoysticks(&drive_stick);
        left_drive = drive_stick.y - drive_stick.x;
        right_drive = drive_stick.y +  drive_stick.x;
        left_drive = clamp(left_drive, -1.0, 1.0);
        right_drive = clamp(right_drive, -1.0, 1.0);
        //Might need to add additional bounding in as a safety
        
//============================== Arm =============================            
        float potentiometer_range = 333.33333333333333333333333333333333333f;
        //TODO: correctly convert to angle
        elbow_potentiometer_angle = lerp(
            (360-((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))+12.0f))*pi/180.0f,
            elbow_potentiometer_angle,
            exp(-20.0*dt));

        float new_shoulder_theta = shoulder_encoder/shoulder_gear_ratio/encoder_ticks_per_radian+pi*150/180.0;
        float new_inside_elbow_theta = elbow_potentiometer_angle;
        float new_winch_theta = winch_encoder/winch_gear_ratio/encoder_ticks_per_radian;
        shoulder_omega = lerp((new_shoulder_theta-shoulder_theta)/dt, shoulder_omega, 0.1);
        winch_omega = lerp((new_winch_theta-winch_theta)/dt, winch_omega, 0.1);
        float inside_elbow_omega = (new_inside_elbow_theta-inside_elbow_theta)/dt;
        
        shoulder_print_theta = new_shoulder_theta;
        forearm_print_theta = new_inside_elbow_theta;
        
        shoulder_theta = new_shoulder_theta;
        inside_elbow_theta = new_inside_elbow_theta;
        winch_theta = new_winch_theta;
        
        if(!arm_manual_toggle) //IK
        {
            v2f target_arm_velocity = arm_stick;

            if(arm_score_mode_button)
            {
                score_mode = true;
                //target_shoulder_theta = 0.75;
                target_inside_elbow_theta = pi*1/3;
                
                shoulder = -clamp(2-0.6*(inside_elbow_theta-target_inside_elbow_theta), 0.0, 1.0);
                float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                
                winch = shoulder*winch_gear_ratio/shoulder_gear_ratio;
            }
            else if(arm_intake_mode_button)
            {
                score_mode = false;
                //target_shoulder_theta = 2.75;
                target_inside_elbow_theta = pi*4/3;
                
                shoulder = clamp(2-0.6*(target_inside_elbow_theta-inside_elbow_theta), 0.0, 1.0);
                target_shoulder_theta = shoulder_theta;
                
                winch = shoulder*winch_gear_ratio/shoulder_gear_ratio;
            }
            else
            {
                if(score_mode)
                {
                    armAtVelocity(shoulder, winch,
                                  target_arm_theta, target_inside_elbow_theta, target_arm_velocity,
                                  shoulder_theta, inside_elbow_theta,
                                  shoulder_omega, dt);
                    
                    target_shoulder_theta = shoulder_theta;
                }
                else
                {
                    armJointsAtVelocity(shoulder, winch,
                                        target_shoulder_theta, target_inside_elbow_theta, target_arm_velocity,
                                        shoulder_theta, inside_elbow_theta,
                                        shoulder_omega, dt);
                
                    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                    target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                }
            }
            
            if(score_mode)
            {
                armToAngle(shoulder, winch,
                           target_arm_theta, target_inside_elbow_theta,
                           shoulder_theta, inside_elbow_theta,
                           score_mode, dt);
            }
            else
            {
                armJointsToAngle(shoulder, winch,
                                 target_shoulder_theta, target_inside_elbow_theta,
                                 shoulder_theta, inside_elbow_theta,
                                 score_mode, dt);
            }
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
        }
        else //Manual
        {
            float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                              -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
            target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
            target_shoulder_theta = shoulder_theta;
            target_inside_elbow_theta = inside_elbow_theta;
            
            deadZone(elbow_manual);
            deadZone(shoulder_manual);
            // smoothJoysticks(&elbow_manual);//This needs to be fixed.
            // smoothJoysticks(&shoulder_manual);
            shoulder = shoulder_manual.y*(precision_mode ? 0.6 : 1);
            winch = elbow_manual.y*(precision_mode ? 0.6 : 1);
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
        }
        
//TODO: Feed-Forward
//TODO: Position macros (DPAD on second controller)
//TODO: Precision mode
//TODO: Rope tension
//TODO: Locks
//============================= Hopper ===========================
        if(intake_toggle) intake = intake_reverse ? -1 : 1;
        else intake = 0;
        if(hopper_tilt)
        {
            if(current_color)
                hand = hand_blue_position;
            else
                hand = hand_red_position;
        }
        else
            hand = hand_level_position;
        
        hand = clamp(hand, 0.0, 1.0);

        //for finding servo values
        // if(pad1.singlePress(B)) hand_level_position += 0.1;
        // if(pad1.singlePress(X)) hand_level_position -= 0.1;
        // hand_print_position = hand_level_position;
        
//TODO: Auto-score
//TODO: Block count
        //TODO: Tilt
//============================ Slide ===========================
        if(slide_toggle)
        {
            if(current_color)
                slide = slide_blue_position;
            else
                slide = slide_red_position;
        }
        else
            slide = slide_stored_position;
        
        //These aren't working yet, I'll need to write a release condition (the toggle is forcing it closed)
        if(slide_right)
            slide += slide_speed;
        if(slide_left)
            slide -= slide_speed;
        
        slide = clamp(slide, 0.0, 1.0);
//============================ Updates ===========================
        pad1.updateButtons(gamepad1.buttons);
        pad2.updateButtons(gamepad2.buttons);
    } while(updateRobot() == 0);
    
    cleanupJNI();
}
