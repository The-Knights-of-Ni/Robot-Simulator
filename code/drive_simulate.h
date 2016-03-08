/*
Drive_simulate.h - Interface between the rendering and the code, as well as stick inputs
*/
#ifndef DRIVE_SIMULATE
#define DRIVE_SIMULATE

#include "native_path.h"
#include "ui.h"
#include "Mk3Teleop_robot_state_elements.h"
#include "robot.h"
#include "gl_renderer.h"

bool virtual_right_stick_held = false;
bool virtual_left_stick_held = false;

v2f drivepos = (v2f) {10.0,10.0};
bool initPosition = false;
float driveangle = 0.0;
float driveforward = 0.0;
drivebase rabbit((v3f) {17.0, 17.0, 5.0}, 0, (v3f) {10, 10, -20}, drive_base, 10.0, (v3f) {8.5, 8.5, 2.5}, 1.0);
void simulateAndRender()
{
	if(!initPosition)
	{
		initPosition = true;
	}
	float x_pos = 0; //Using this for GUI offsets
	//Draw Virtual Sticks on the screen and map them to real sticks
	virtual_joystick virtual_right_stick = doVirtualJoystickNW(virtual_right_stick_held, 0.5, -0.5, 100, 100);
    simulatorgamepad2.joystick2 = virtual_right_stick.joystick;
    virtual_right_stick_held = virtual_right_stick.held;

    virtual_joystick virtual_left_stick = doVirtualJoystickNW(virtual_left_stick_held, -0.5, -0.5, 100, 100);
    simulatorgamepad2.joystick1 = virtual_left_stick.joystick;
    virtual_left_stick_held = virtual_left_stick.held;

	//Joystick button assignments. Converting between SDL and our button handler.
	static bool joystick_exists = true;
	if(joystick)
    {
		//TODO: Make this for the drive instead of the arm, or just a function for every button/stick/trigger.
    	simulatorgamepad2.joystick1 = (v2f){(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_LEFTX)+0.5)/32767.5,
                                   -(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_LEFTY)+0.5)/32767.5};

        simulatorgamepad2.joystick2 = (v2f){(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTX)+0.5)/32767.5,
                                   -(SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTY)+0.5)/32767.5};

        simulatorgamepad2.left_trigger = (SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERLEFT)+0.5)/32767.5;
        simulatorgamepad2.right_trigger = (SDL_JoystickGetAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERRIGHT)+0.5)/32767.5;

        simulatorgamepad2.buttons |= (SDL_JoystickGetButton(joystick, 0) << DPAD_UP);
        simulatorgamepad2.buttons |= (SDL_JoystickGetButton(joystick, 1) << DPAD_DOWN);
        simulatorgamepad2.buttons |= (SDL_JoystickGetButton(joystick, 10) << A);
        simulatorgamepad2.buttons |= (SDL_JoystickGetButton(joystick, 13) << Y);
		joystick_exists = true;
		//printf("Controller Detected\n");
    }
    else
    {
		if(joystick_exists)
        	printf("no controller\n");
		joystick_exists = false;
    }


	//Display and buttons
	//TODO: Make readout a separate window/neater hud
	x_pos = -1;

	char * left_joystick_string = (char *) malloc(100);
	sprintf(left_joystick_string, "left joystick (%f, %f)",
			simulatorgamepad2.joystick1.x, simulatorgamepad2.joystick1.y);
	doHoldButtonNW(left_joystick_string, x_pos, 0.4, 4, 2);

	char * right_joystick_string = (char *) malloc(100);
	sprintf(right_joystick_string, "right joystick (%f, %f)",
			simulatorgamepad2.joystick2.x, simulatorgamepad2.joystick2.y);
	doHoldButtonNW(right_joystick_string, x_pos, 0.3, 4, 2);

	char * time_string = (char *) malloc(100);
	sprintf(time_string, "time %f", timeSim);
	doHoldButtonNW(time_string, x_pos, 0.2, 4, 2);

	float dt = 0.00075;

    timeSim += dt;
	v2f sticks = (v2f) {simulatorgamepad2.joystick1.x, -simulatorgamepad2.joystick1.y};
	v2f modified = smoothJoysticks(sticks, 0, 0.2, 0.8, 1).stick;
	rabbit.left_drive_sim += -modified.y - modified.x;//This is real teleop code TODO: Interface between Mk4Teleop.cpp and this struct
	rabbit.right_drive_sim += -modified.y + modified.x;
	simulateRobot(rabbit);
}

#endif
