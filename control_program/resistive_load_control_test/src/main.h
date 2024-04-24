//
//              FILENAME: main.h
//
// description: Header file for main.c.
//
// Written by Marek Newton
// 

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

typedef enum charger_state
{
    OFF,
    WARMING_UP,
    CHARGING
} Charger_State;

void setup_IO(void);
void setup_pin_change_interupt(void);
void setup_rpm_timer(void);
void setup_servo_timer(void);
void setup_control_timer(void);
void setup_ADC(void);
void adc_complete(unsigned int adc_value);
void servo_PWM_signal(uint16_t pulse_length);
void rpm_timer_complete(uint32_t *encoder_counts);
void warmup_cycle(void);
void control_timer_complete(void);
void update_control_system(void);

#endif