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

#include "native_path.h"

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
