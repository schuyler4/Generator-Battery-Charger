//
//      FILENAME: RPM.h
// 
// description: The header file for RPM.c.
//

#ifndef RPM_H
#define RPM_H

#include <stdint.h>

// This is the struct used to store the variety of
// parameters needed for RPM measurement.
typedef struct rpm_meter
{
    uint32_t rpm;
    uint32_t encoder_counts;
    uint32_t flag;
} RPM_meter;

// This sets up the registers for the RPM timer.
// It should be called along with the other peripheral setup.
void setup_rpm_timer(void);
void setup_pin_change_interupt(void);
void setup_rpm_meter(RPM_meter *rpm_meter);
void check_rpm_timer_complete(RPM_meter *rpm_meter);
void increment_encoder_counts(RPM_meter *rpm_meter);
void set_rpm_timer_flag(RPM_meter *rpm_meter);

#endif