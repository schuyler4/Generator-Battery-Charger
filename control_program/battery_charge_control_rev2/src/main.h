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
    THROTTLING_DOWN,
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
void warmup_cycle(void);
void check_control_timer_complete(void);

void off(void);
void warming_up(void);
void throttling_down(void);
void charging(void);

void update_state_machine(void);
void control_charger(void);
void isolation_contactor_control(void);

#endif