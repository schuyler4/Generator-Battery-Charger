//
//      FILENAME: logging.c
//
// description: This file takes care of all the logging for
// the program. Logging entails sending data over UART so
// the user can read it with a serial monitor. All important
// info about the charger should be logged. 
//
// Written by Marek Newton
//

#include "main.h"
#include "UART.h"
#include "RPM.h"

#include "logging.h"

#define SEND_COMMAND "SEND A COMMAND"
#define CHARGE_COMMAND "1 - CHARGE"
#define CONTACTOR_TEST "2 - CONTACTOR TEST"
#define RESISTIVE_LOAD_TEST "3 - RESISTIVE LOAD TEST"
#define FULL_THROTTLE_OPEN_CIRCUIT_TEST "4 - FULL THROTTLE OPEN CIRCUIT TEST"

#define ADC_CODE_SELECTION_MESSAGE "Select a current ADC code set point for the resistive load test: "
#define FIRST_SET_POINT "1 - 120"
#define SECOND_SET_POINT "2 - 100"
#define THIRD_SET_POINT "3 - 75"

#define ECHO_MESSAGE "You entered: "

#define CONTACTOR_TEST_MESSAGE "Verify that the contactors have closed."

#define RESISTIVE_LOAD_TEST_MESSAGE "Enter a current set point: "

#define INVALID_COMMAND_MESSAGE "Invalid command, try again."

#define SYSTEM_DATA_HEADER "== SYSTEM DATA =="

#define STATE_STRING "Charger State: "
#define OFF_STRING "OFF"
#define WARMING_UP_STRING "WARMING UP"
#define THROTTLING_DOWN_STRING "THROTTLING DOWN"
#define CHARGING_STRING "CHARGING"
#define FAULT_STRING "FAULT"
#define CHARGE_COMPLETE_STRING "CHARGE COMPLETE"

#define STARTUP_MESSAGE "GENERATOR CHARGER CONTROLLER RESET"

#define RPM_STRING "RPM: "
#define CURRENT_ADC_CODE_STRING "CURRENT ADC CODE: "
#define VOLTAGE_ADC_CODE_STRING "VOLTAGE ADC CODE: "
#define CONTROLLED_COMMAND_STRING "CONTROLLED_COMMAND: "
#define WARMUP_COUNT "WARMUP COUNT: "
#define CURRENT_SET_POINT_STRING "CURRENT SET POINT: "

#define NEWLINE_AND_CARRIAGE_RETURN "\n\r"
#define LOGGING_COUNT_DELAY 100

// The static functions go first.

// This function logs the current state that 
// the charger is in. 
static void log_state(Charger_State state)
{
    UART_transmit_string(STATE_STRING);

    switch(state)
    {
        case OFF:
            UART_transmit_string(OFF_STRING);
            break;
        case WARMING_UP:
            UART_transmit_string(WARMING_UP_STRING);
            break;
        case THROTTLING_DOWN:
            UART_transmit_string(THROTTLING_DOWN_STRING);
            break;
        case CHARGING:
            UART_transmit_string(CHARGING_STRING);   
            break;
        case FAULT:
            UART_transmit_string(FAULT_STRING);
            break;
        case CHARGE_COMPLETE:
            UART_transmit_string(CHARGE_COMPLETE_STRING);
            break;  
        default:
            // do nothing
            break;
    }

    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
}

// This function logs all the measured values that
// are important to the operation of the charger.
static void log_measurements(Logging_Data *logging_data_pointer)
{
    UART_transmit_string(SYSTEM_DATA_HEADER);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    // Only show the warmup count, if the system is, 
    // in fact, warming up.
    if(logging_data_pointer->state == WARMING_UP)
    {
        UART_transmit_string(WARMUP_COUNT);
        UART_transmit_int(logging_data_pointer->warmup_count);
        UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    }

    log_state(logging_data_pointer->state);

    // Log the RPM.
    UART_transmit_string(RPM_STRING);
    UART_transmit_int(logging_data_pointer->rpm_meter->rpm);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    // Log the measured current ADC code.
    UART_transmit_string(CURRENT_ADC_CODE_STRING);
    UART_transmit_int(logging_data_pointer->averaged_current_adc_code);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    // Log the measured voltage ADC code.
    UART_transmit_string(VOLTAGE_ADC_CODE_STRING);
    UART_transmit_int(logging_data_pointer->voltage_adc_code);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    // Log the control command.
    UART_transmit_string(CONTROLLED_COMMAND_STRING);
    UART_transmit_int((int)logging_data_pointer->controlled_command);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    // Log the current set point. 
    UART_transmit_string(CURRENT_SET_POINT_STRING);
    UART_transmit_int(logging_data_pointer->current_set_point);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    // This adds some white space to the end of the log.
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
}

void update_logging_data(
    Logging_Data* logging_data_pointer,
    Charger_State state,
    RPM_meter *rpm_meter,
    int averaged_current_adc_code,
    uint16_t voltage_adc_code,
    float controlled_command,
    uint16_t warmup_count,
    uint16_t current_set_point
    )
{
    logging_data_pointer->state = state;
    logging_data_pointer->rpm_meter = rpm_meter;
    logging_data_pointer->averaged_current_adc_code = averaged_current_adc_code;
    logging_data_pointer->voltage_adc_code = voltage_adc_code;
    logging_data_pointer->controlled_command = controlled_command;
    logging_data_pointer->warmup_count = warmup_count;
    logging_data_pointer->current_set_point = current_set_point;
}

// This function is called at the beginning of the program.
void log_startup(Logging_Data *logging_data_pointer)
{
    // Initialize the logging count to zero.
    logging_data_pointer->logging_count = 0;

    UART_transmit_string(STARTUP_MESSAGE);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    // Add some whitespace.
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
}

// This function should be called in the control loop 
// to log the data occasionally. 
void periodic_logging(Logging_Data* logging_data_pointer)
{
    if(logging_data_pointer->logging_count >= LOGGING_COUNT_DELAY)
    {
        log_measurements(logging_data_pointer);

        // reset the count
        logging_data_pointer->logging_count = 0;
    }
    else
    {
        logging_data_pointer->logging_count++;
    }
}

// Sends all the messages necessary for the initial start up menu.
void log_start_up_menu(void)
{
    UART_transmit_string(SEND_COMMAND);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    UART_transmit_string(CHARGE_COMMAND);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    UART_transmit_string(CONTACTOR_TEST);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    UART_transmit_string(RESISTIVE_LOAD_TEST);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    UART_transmit_string(FULL_THROTTLE_OPEN_CIRCUIT_TEST);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
}

void log_contactor_close_test(void)
{
    UART_transmit_string(CONTACTOR_TEST_MESSAGE);
}

void log_resistive_load_test_current_input(void)
{
    UART_transmit_string(RESISTIVE_LOAD_TEST_MESSAGE);   
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    UART_transmit_string(FIRST_SET_POINT);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    UART_transmit_string(SECOND_SET_POINT);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    UART_transmit_string(THIRD_SET_POINT);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
}

// Echo the option selected back to the user. 
void echo_selection(char echo_character)
{
    UART_transmit_string(ECHO_MESSAGE);
    UART_transmit_char(echo_character);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
}

void log_invalid_command(void)
{
    UART_transmit_string(INVALID_COMMAND_MESSAGE);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
}