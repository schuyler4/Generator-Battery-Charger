//
// FILENAME: PID_controller.h
//
// title: PID Controller Header File
//
// description: This is the header file for the PID controller implementation.
// It contains the struct which holds the controller 'object' and the definition
// for the function that is used to update the controller.
// 
// Written by Marek Newton
//

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct pid_controller
{
    // the most recent PID sample
    int32_t sample;
    // the difference between the set point and 
    // the current sample
    int32_t error;
    // the error value calculated on the previous sample
    int32_t last_error;
    // the target value of the PID controller
    int32_t set_point; 
    
    // PID Parameters
    int32_t PROPORTIONAL_COEFFICIENT;
    int32_t INTEGRAL_COEFFICIENT;
    int32_t DERIVATIVE_COEFFICIENT;

    // the sample period
    float dt;

    // used to store the controllers integral value
    int32_t integral;

    // slight adjustments must be made to the computation on the first 
    // sample. 
    int8_t first_sample;
} PID_Controller;

void setup_PID_controller(
    PID_Controller *controller, 
    int32_t P, 
    int32_t I, 
    int32_t D, 
    int32_t set_point, 
    float dt);
int32_t update_PID_controller(PID_Controller *controller, int32_t new_sample);

#endif