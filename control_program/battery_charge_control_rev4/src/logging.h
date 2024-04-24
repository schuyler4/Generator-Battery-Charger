//
//      FILENAME: logging.h
//
// description: The header file for logging.c.
//
// Written by Marek Newton
//

#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>

#include "main.h"
#include "RPM.h"

// This is a struct to store all the data that will be logged.
typedef struct logging_data
{
    Charger_State state;
    RPM_meter *rpm_meter;
    int averaged_current_adc_code;
    uint16_t voltage_adc_code;
    float controlled_command;
    uint16_t warmup_count;
    uint16_t logging_count;
    uint16_t current_set_point; 
} Logging_Data;

void log_startup(Logging_Data* logging_data_pointer);
void periodic_logging(Logging_Data* logging_data_pointer);
void log_start_up_menu(void);

void log_contactor_close_test(void);
void log_resistive_load_test_current_input(void);

void update_logging_data(
    Logging_Data* logging_data_pointer,
    Charger_State state,
    RPM_meter *rpm_meter,
    int averaged_current_adc_code,
    uint16_t voltage_adc_code,
    float controlled_command,
    uint16_t warmup_count,
    uint16_t current_set_point
);

void echo_selection(char echo_character);
void log_invalid_command(void);

#endif