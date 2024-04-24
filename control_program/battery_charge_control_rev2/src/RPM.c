//
// FILENAME: RPM.c
// 
// description: This file has all the functionality to read RPM from the encoder.
// This includes counting the number of rising edges coming off the encoder, and
// implementing a timer for the per minute part of RPM. 
//
// Written by Marek Newton
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "RPM.h"
#include "UART.h"

// general byte manipulation
#define BYTE_SIZE 8
#define BYTE_MASK 0xFF

// encoder parameters
#define ENCODER_GEAR_RATIO 2.95
#define ENCODER_PULSES_PER_REV 600
#define SEC_PER_MIN 60
// the timer comparison for 1 second

#define TIMER_COUNT 255
#define MAX_RPM_TIMER_COUNTS 123
#define TIMER_PERIOD 0.0161

void setup_rpm_meter(RPM_meter *rpm_meter)
{
    // Initialize the set of counters to zero.
    rpm_meter->rpm = 0;
    rpm_meter->encoder_counts = 0;
    rpm_meter->flag = 0;
}

// Configure timer 1 for RPM measurement.
void setup_rpm_timer(void)
{
    // set the timer to fcpu divide by 1024
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    // enable compare match interupt
    TIMSK2 |= (1 << OCIE2A);
    // set the timer count
    OCR2A = TIMER_COUNT;
}

// setup the pin change interupt to measure RPM with the encoder
void setup_pin_change_interupt(void)
{
    // turn on the pin change interupt on port B
    PCICR |= (1 << PCIE0);
    // set pin change interupt on PORTB0
    PCMSK0 |= (1 << PCINT0);
}

void increment_encoder_counts(RPM_meter *rpm_meter)
{
    rpm_meter->encoder_counts++;
}

void set_rpm_timer_flag(RPM_meter *rpm_meter)
{
    rpm_meter->flag = 1;
}

// This is called when the rpm timer has elapsed.
void check_rpm_timer_complete(RPM_meter *rpm_meter)
{
    // executes when the rpm timer has completed
    if(rpm_meter->flag)
    {
        // calculate the RPM
        float rev_per_sec = (float)rpm_meter->encoder_counts/(ENCODER_PULSES_PER_REV*TIMER_PERIOD);
        rpm_meter->rpm = (rev_per_sec*SEC_PER_MIN)/ENCODER_GEAR_RATIO;
        // reset the encoder counts
        rpm_meter->encoder_counts = 0;
        // reset the flag
        rpm_meter->flag = 0;
    }
}