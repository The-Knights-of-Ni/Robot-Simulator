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
#define hopper_tilt pad1.toggle(A) //Might make this a stick or something

//Arm
#define shoulder_manual pad2stick1
#define elbow_manual pad2stick2
#define arm_stick pad2stick1
#define arm_manual_toggle pad2.toggle(DPAD_UP)
#define arm_intake_mode_button (gamepad2.left_trigger > 0.5)
#define arm_score_mode_button (gamepad2.right_trigger > 0.5)
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
    
arm_state s = {};
arm_state stable_s = {};
bool8 score_mode = true;

float dt;
float old_time = time;
    
float elbow_potentiometer_angle = 0.0;

//just because the simulator doesn't prefectly treat it like real code yet
void simulatorStartMain(){
    stable_s.shoulder_theta = pi/2;
    stable_s.forearm_theta = 0;
    stable_s.winch_theta = 0;
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
        float new_forearm_theta = elbow_potentiometer_angle+new_shoulder_theta-pi;
        float new_winch_theta = winch_encoder/winch_gear_ratio/encoder_ticks_per_radian;
        s.shoulder_omega = lerp((new_shoulder_theta-s.shoulder_theta)/dt, s.shoulder_omega, exp(-0.1*dt));
        s.forearm_omega = lerp((new_forearm_theta-s.forearm_theta)/dt, s.forearm_omega, exp(-0.1*dt));
        s.winch_omega = lerp((new_winch_theta-s.winch_theta)/dt, s.winch_omega, exp(-0.1*dt));
        
        shoulder_print_theta = new_shoulder_theta;
        forearm_print_theta = new_forearm_theta;
        
        s.shoulder_theta = new_shoulder_theta;
        s.forearm_theta = new_forearm_theta;
        s.winch_theta = new_winch_theta;

        if(stable_s.forearm_theta > stable_s.shoulder_theta-0.25) stable_s.forearm_theta = stable_s.shoulder_theta-0.25;
        if(arm_manual_toggle) //IK
        {
            
            //deadZone(arm_stick);
            v2f target_arm_velocity = arm_stick*40;
                        
            if(arm_intake_mode_button)
            {
                score_mode = false;
                stable_s.shoulder_theta = 2.75;
                stable_s.forearm_theta = pi;
                stable_s.winch_theta = 5.5;
            }
            if(arm_score_mode_button)
            {
                score_mode = true;
                stable_s.shoulder_theta = 0.75;
                stable_s.forearm_theta = 0;
                stable_s.winch_theta = 1;
            }
            
            if(normSq(target_arm_velocity) > 1.0)
            {
                stable_s = s;
                stable_s.shoulder_theta = clamp(stable_s.shoulder_theta, pi/6, 5*pi/6);
                armAtVelocity(shoulder, winch, target_arm_velocity, s, score_mode, dt);
            }
            else
            {
                armToState(shoulder, winch, stable_s, s, score_mode, dt);
            }
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
        }
        else //Manual
        {
            stable_s = s;
            stable_s.shoulder_theta = clamp(stable_s.shoulder_theta, pi/6, 5*pi/6);
            
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
