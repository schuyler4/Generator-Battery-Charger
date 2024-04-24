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

typedef struct logging_data
{
    Charger_State state;
    RPM_meter *rpm_meter;
    int averaged_current_adc_code;
    float controlled_command;
    uint16_t warmup_count;
    uint16_t logging_count;
} Logging_Data;

void log_startup(Logging_Data* logging_data_pointer);
void periodic_logging(Logging_Data* logging_data_pointer);

void update_logging_data(
    Logging_Data* logging_data_pointer,
    Charger_State state,
    RPM_meter *rpm_meter,
    int averaged_current_adc_code,
    float controlled_command,
    uint16_t warmup_count
);

#endif