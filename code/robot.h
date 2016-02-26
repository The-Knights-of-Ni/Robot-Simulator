/*
Robot.h - Model of the robot, contains information about measurements and what motors do
*/
#include "native_path.h"
#include "gl_renderer.h"

#define turningConstant (628.2*2)
#define drivingConstant (25.0)
struct drivebase
{
	union
	{
		struct
		{
			float width;
			float length;
			float height;
		};
		v3f dimensions;
		float data[3];
	};
	struct //TODO: Add options for other drivebase configurations (6 motor, holonomic, swerve, 4 motor, etc.)
	{
		float left_drive_sim;
		float right_drive_sim;
	};
	float field_orientation; //TODO: Make this a quaternion, and then give it to the imu
	v3f position;
	int model;//TODO: Generate this from given lwh
	float mass;
	v3f center_of_mass;
	float transmission_ratio;//TODO: Allow gearshifts

	drivebase(v3f dimensions_in, float field_orientation_in, v3f position_in, int model_in, float mass_in, v3f center_of_mass_in, float transmission_ratio_in)
	{
		//TODO: 90% of these variables can be loaded from a model
		dimensions = dimensions_in;
		field_orientation = field_orientation_in;
		position = position_in;
		model = model_in;
		mass = mass_in;
		center_of_mass = center_of_mass_in;
		transmission_ratio = transmission_ratio_in;
	}
	void orientationFix()
	{
		if(field_orientation <= -2*pi) field_orientation += 2*pi;
		if(field_orientation > 0) field_orientation-= 2*pi;
	}
	void pwmToMotion()
	{
		orientationFix();
		field_orientation += (left_drive_sim - right_drive_sim)/turningConstant;
		position.x += ((left_drive_sim + right_drive_sim)/drivingConstant)*cos(fabs(field_orientation));
		position.y += ((left_drive_sim + right_drive_sim)/drivingConstant)*sin(fabs(field_orientation));
	}

	inline float &operator[](int a)
	{
		return data[a];
	}
};

void simulateRobot(drivebase bot)
{
	bot.pwmToMotion();
	render_list[n_to_render].model = bot.model;
	render_list[n_to_render].position = (v3f) {bot.position.x, -20.0, bot.position.y};
    render_list[n_to_render].orientation = (v4f) {0.0, sin(bot.field_orientation/2.0), 0.0, cos(bot.field_orientation/2.0)};
	n_to_render++;
}
