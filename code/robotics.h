/*
 * Robotics.h - Functions and constants for all programs.
 *
 *
*/

#ifndef ROBOTICS
#define ROBOTICS

#include "meth.h"
#include "misc.h"
#define bound clamp
//Constants
#define encoder_ticks_per_radian (1440.0f/(2.0f*pi))
#define deadzone_radius 0.1

float deadzoneAdjust(float & a)
{
    if(a > deadzone_radius) return (a-deadzone_radius)/(1-deadzone_radius);
    if(a < -deadzone_radius) return (a+deadzone_radius)/(1-deadzone_radius);
    return 0;
}

//PID Control: UNTESTED, may not be ported correctly (In case the built in doesn't work)
//TODO: de-OOP
struct PID
{
    float k_p;
    float k_i;
    float k_d;
    float k_d2;

    float i;
    float p_old;
    float d;
    float p2_old;
    float d2;

    void PIDController(float K_P, float K_I, float K_D, float K_D2, float initial_val,
                       float initial_val2)
    {
        k_p = K_P;
        k_i = K_I;
        k_d = K_D;
        k_d2 = K_D2;
        p_old = initial_val;
        p2_old = initial_val2;
    }

    float getControl(float p, float p2, float dt)
    {
        d2 = lerp((p2 - p2_old) / dt,
                  d2,
                  exp(-20.0 * dt));
        p2_old = p2;

        if (i != i) i = 0.0f; //this will trigger if i is NaN
        i += p * dt; //might want to try different integrators
        d = lerp((p - p_old) / dt,
                 d,
                 exp(-20.0 * dt));
        p_old = p;
        return k_p * p + k_i * i;//+k_d*d+k_d2*d2;
    }
};

#endif //ROBOTICS
