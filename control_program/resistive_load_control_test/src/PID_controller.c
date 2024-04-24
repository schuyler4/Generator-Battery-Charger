//
//         FILENAME: PID_controller.c
// 
// description: implements the calculations for a PID controller
// which is represented by the struct defined in the header file.
//
// Written by Marek Newton
//

#include <stdint.h>

#include "PID_controller.h"

// initializes all the properties of the controller
void setup_PID_controller(
    PID_Controller *controller, 
    int32_t P, 
    int32_t I, 
    int32_t D, 
    int32_t set_point, 
    float dt)
{
    controller->set_point = set_point;
    controller->PROPORTIONAL_COEFFICIENT = P;
    controller->INTEGRAL_COEFFICIENT = I;
    controller->DERIVATIVE_COEFFICIENT = D;
    controller->dt = dt;
    controller->integral = 0;
    controller->first_sample = 1;
}

int32_t update_PID_controller(PID_Controller *controller, int32_t new_sample)
{
    // update the sample and the error
    if(!controller->first_sample)
    {
        // the derivative term is not computed on the first sample
        controller->last_error = controller->error;
    }
    controller->sample = new_sample;
    controller->error = controller->set_point - controller->sample;

    // calculate the PID terms
    int32_t P = controller->error;
    int32_t I = controller->integral + controller->dt*controller->error;
    int32_t D;
    if(!controller->first_sample)
    {
        D = (controller->error - controller->last_error)/controller->dt;
    }
    else
    {
        D = 0;
    }
    
    // shorten the coefficient names
    int32_t PC = controller->PROPORTIONAL_COEFFICIENT;
    int32_t IC = controller->INTEGRAL_COEFFICIENT;
    int32_t DC = controller->DERIVATIVE_COEFFICIENT;

    if(controller->first_sample)
    {
        controller->first_sample = 0;
    }

    // return the PID sum
    return P*PC + I*IC + D*DC;
}