//
//              FILENAME: main.h
//
// description: Header file for main.c. Contains enumerations that keep track of
// state. Also contains function prototypes for main.c.
//
// Written by Marek Newton
// 

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// enumeration for the charger state machine
typedef enum charger_state
{
    OFF,
    WARMING_UP,
    THROTTLING_DOWN,
    CHARGING,
    FAULT,
    CHARGE_COMPLETE
} Charger_State;

// enumeration for system state machine
typedef enum system_state
{
    START_UP,
    CHARGE_CYCLE,
    CONTACTOR_TEST,
    RESISTIVE_LOAD_TEST,
    FULL_THROTTLE_TEST
} System_State;

typedef enum measurement_state
{
    VOLTAGE_MEASUREMENT,
    CURRENT_MEASUREMENT
} Measurement_State;

void start_up_sequence(void);
void set_discharge_circuit(void);
void contactor_close_test(void);
void peripheral_setup(void);
void setup_IO(void);
void check_fault_input(void);
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
void check_charge_complete(void);
void set_current_set_point(void);

void update_state_machine(void);
void control_charger(void);
void isolation_contactor_control(void);

#endif
