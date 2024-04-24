//
// FILENAME: resistive_load_test.c
//
// description: This file contains functions that are purely related
// to the resistive load semi self test functionality. 
//
// Written by Marek Newton
//

#include <avr/io.h>
#include <stdint.h>

#include "logging.h"
#include "UART.h"

#define FIRST_SET_POINT_CHARACTER '1'
#define SECOND_SET_POINT_CHARACTER '2'
#define THIRD_SET_POINT_CHARACTER '3'

#define FIRST_SET_POINT_ADC_CODE 120
#define SECOND_SET_POINT_ADC_CODE 100
#define THIRD_SET_POINT_ADC_CODE 75

// Reads in a selected current set point from the user and returns it. The resistive load test
// evaluates the stability of the system at three different current set points.
uint16_t get_variable_current_set_point(void)
{
    int16_t user_current_set_point;

    while(1)
    {
        log_resistive_load_test_current_input();

        int16_t user_current_set_point = UART_getc();

        echo_selection(user_current_set_point);

        if(user_current_set_point == FIRST_SET_POINT_CHARACTER)
        {
            return FIRST_SET_POINT_ADC_CODE;
        }
        else if(user_current_set_point == SECOND_SET_POINT_CHARACTER)
        {
            return SECOND_SET_POINT_ADC_CODE;
        }
        else if(user_current_set_point == THIRD_SET_POINT_CHARACTER)
        {
            return THIRD_SET_POINT_ADC_CODE;
        }
    }
    
    // The program should never reach this point.
    return 1;
}