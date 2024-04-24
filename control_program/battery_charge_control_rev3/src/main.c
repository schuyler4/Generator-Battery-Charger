// 
// FILENAME: main.c
//
// Gasoline Generator Battery Charger Control Program 
//
// This program controls the generator battery charger prototype. 
// This involves controlling the throttle of a gasoline engine with a servo. 
// The gasoline engine is coupled to an induction generator which is connected 
// to an output rectifier. This whole system allows for voltage control simply
// by changing the throttle of the engine. A current measuring device is connected
// in series with the rectified output of the generator. This output is then connected 
// in series with the battery to charge. 
//
// This program must read in the value from the current measuring device and adjust the
// throttle to maintain a constant current. In addition, it measures the RPM of the 
// generator using an encoder. 
//
// Written by Marek Newton
//

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

#include "UART.h"
#include "running_average.h"
#include "RPM.h"
#include "logging.h"

#include "main.h"
  
// Constants for general byte manipulation.
#define BYTE_SIZE 8
#define BYTE_MASK 0xFF

// battery current reading parameters
#define CURRENT_FILTER_LENGTH 10

// controller parameters
#define BATTERY_CURRENT_ADC_CODE_SET_POINT 110

// system state parameters
#define MINIMUM_ON_RPM 10
#define TOTAL_WARMUP_COUNT 2000
#define MAXIMUM_LOW_THROTTLE_RPM 200
#define MINIMUM_CHARGING_RPM 600

// full throttle time: 1.36ms
#define FULL_THROTTLE_COMPARE_MATCH 2775
// low throttle pulse time: 1.51ms
#define LOW_THROTTLE_COMPARE_MATCH 3020

// this flag gets flipped when the ADC conversion is complete
static volatile unsigned int adc_complete_flag = 0;
// this flag gets flipped when the timer finishes
static volatile unsigned int control_timer_complete_flag = 0;
static volatile uint8_t rpm_timer_complete_flag = 0;

// this stores the ADC value
static volatile unsigned int adc_value;

// instantiate filter for adc current sampling
Running_Average_Filter current_adc_filter;

Charger_State state;

float compare_match_command = 0;
uint16_t warmup_count = 0;
int average_current_adc_value = 0;

RPM_meter my_rpm_meter;

// set up the logging data structure
Logging_Data my_logging_data;

int main(void)
{
    meta_setup();
    // start the first ADC conversion
    ADCSRA |= (1 << ADSC); 

    // set the length of the running average filter
    current_adc_filter.length = CURRENT_FILTER_LENGTH;

    // Set the initial state to off.
    state = OFF;

    log_startup(&my_logging_data);

    update_logging_data(
        &my_logging_data,
        state,
        &my_rpm_meter,
        average_current_adc_value,
        compare_match_command,
        warmup_count
    );

    while(1) // the program should never return
    {
        // TODO: refactor this all into the ADC function
        if(adc_complete_flag)
        {
            // executes after an ADC conversion has completed and the
            // flag has been set by the interupt
            adc_complete(adc_value);
            adc_complete_flag = 0;
        }

        // interupt flag checks
        check_control_timer_complete();
        check_rpm_timer_complete(&my_rpm_meter);

        isolation_contactor_control();
    }

    return 1;
}

// This calls all the setup functions.
void meta_setup(void)
{
    // disable interupts for setup
    cli(); 
    // call setup functions
    setup_IO();
    setup_servo_timer();
    setup_control_timer();
    setup_ADC();    
    setup_UART();

    // rpm setup
    setup_rpm_timer();
    setup_pin_change_interupt();

    // reenable interupts
    sei();

    // initialize the rpm meter
    setup_rpm_meter(&my_rpm_meter);
}

// set up the I/O pins as inputs or outputs
void setup_IO(void)
{
    // set the servo control pin to an output
    DDRB |= (1 << DDB4);
    // set D11 for debug
    DDRB |= (1 << DDB3);
    // Set pin D2 to an output to control the contactor. 
    DDRD |= (1 << DDD2);
}

// This closes the isolation contractors during charging
// and keeps them open the rest of the time.
void isolation_contactor_control(void)
{
    if(state == CHARGING)
    {
        PORTD |= (1 << PORTD2);
    }
    else
    {
        PORTD &= ~(1 << PORTD2);
    }
}

// Configure the ADC registers to setup an interupt ADC sample.
void setup_ADC(void)
{
    // set the ADC reference to the supply voltage (5V)
    ADMUX &= ~(1 << REFS0);
    ADMUX &= ~(1 << REFS1);

    // The 0th ADC channel is used, so the channel does not need 
    // to be configured.

    // enable the ADC
    // enable the ADC interupt
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 < ADPS1) | (1 << ADPS0);
    // disable digital logic on ADC pins
    DIDR0 |= (1 << ADC1D) | (1 << ADC0D); 
}

// Setup the timer that is used to time
// the sampling for the PID loop.
void setup_control_timer(void)
{
    // Set the timer to fcpu divide by 1024.
    TCCR0B |= (1 << CS02) | (1 << CS00);
    // Enable the compare match interupt.
    TIMSK0 |= (1 << OCIE0A);
    // Set the timer count for 10ms
    OCR0A = 156;
}

// Configure timer 1 for servo control.
void setup_servo_timer(void)
{
    // Set GPIO for timer1 output for OC1A and OC1B
    DDRB |= (1 << DDB1) | (1 << DDB2);
    // set entire TCCR1A register to 0
    TCCR1A = 0;
    // same for TCCR1B
    TCCR1B = 0;
    //initialize counter value to 0
    TCNT1  = 0;
    // top value
    ICR1 = 0xFFFF;
    // compare match value
    OCR1A = FULL_THROTTLE_COMPARE_MATCH;
    // set none-inverting mode
    TCCR1A |= (1 << COM1A1);
    // Fast PWM
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    // Use eight as the prescaler
    TCCR1B |= (1 << CS11);
}

// This function executes when the ADC has finished its conversion.
void adc_complete(unsigned int adc_value)
{
    current_adc_filter.sample = adc_value;
    average_current_adc_value = running_average_filter_sample(&current_adc_filter);
}

// This function checks if the control timer is complete, and 
// executes the control algorithm if the timer is complete.
void check_control_timer_complete(void)
{
    if(control_timer_complete_flag)
    {
        update_state_machine();
        // TODO: implement control timer update algorithm
        control_timer_complete_flag = 0;

        update_logging_data(
            &my_logging_data,
            state,
            &my_rpm_meter,
            average_current_adc_value,
            compare_match_command,
            warmup_count
        );

        periodic_logging(&my_logging_data);
    }
}

// The implementation of the actual control algorithm that sets the charger
// to the correct output current. 
void control_charger(void)
{
    uint8_t range_min = LOW_THROTTLE_COMPARE_MATCH - compare_match_command >= FULL_THROTTLE_COMPARE_MATCH;
    uint8_t range_max = LOW_THROTTLE_COMPARE_MATCH - compare_match_command <= LOW_THROTTLE_COMPARE_MATCH;
    uint8_t in_rpm_range = my_rpm_meter.rpm > MINIMUM_CHARGING_RPM;

    if(range_min && average_current_adc_value < BATTERY_CURRENT_ADC_CODE_SET_POINT)
    {
        compare_match_command += (BATTERY_CURRENT_ADC_CODE_SET_POINT - average_current_adc_value)*0.001;
    }
    else if(range_max && average_current_adc_value > BATTERY_CURRENT_ADC_CODE_SET_POINT && in_rpm_range)
    {
        compare_match_command -= (average_current_adc_value - BATTERY_CURRENT_ADC_CODE_SET_POINT)*0.001;
    }
}

void off(void)
{
    // Set the engine to maximum throttle so it can be turned on.
    OCR1A = FULL_THROTTLE_COMPARE_MATCH;
    warmup_count = 0;
    compare_match_command = 0;

    if(my_rpm_meter.rpm >= MINIMUM_ON_RPM)
    {
        state = WARMING_UP;
    }
}

void warming_up(void)
{
    // Check if the state needs to be sent back to off.
    if(my_rpm_meter.rpm < MINIMUM_ON_RPM)
    {
        state = OFF;
    }

        // The engine should be at full throttle to warm up.
    OCR1A = FULL_THROTTLE_COMPARE_MATCH;
    warmup_count++;

    // This checks for a state change to charging.
    if(warmup_count >= TOTAL_WARMUP_COUNT)
    {
        state = THROTTLING_DOWN;
    }
}

void throttling_down(void)
{
    // Check if the state needs to be sent back to off.
    if(my_rpm_meter.rpm < MINIMUM_ON_RPM)
    {
        state = OFF;
    }

    if(my_rpm_meter.rpm < MAXIMUM_LOW_THROTTLE_RPM)
    {
        state = CHARGING;
    }

    OCR1A = LOW_THROTTLE_COMPARE_MATCH - 100;
}

void charging(void)
{
    // Check if the state needs to be sent back to off.
    if(my_rpm_meter.rpm < MINIMUM_ON_RPM)
    {
        state = OFF;
    }

    // Set the timer compare match to the controlled variable.
    OCR1A = LOW_THROTTLE_COMPARE_MATCH - ((int)compare_match_command);
    control_charger(); 
}

void update_state_machine(void)
{
    switch(state)
    {
        case OFF:
            off();
            break;
        case WARMING_UP:
            warming_up();
            break;
        case THROTTLING_DOWN:
            throttling_down();
            break;
        case CHARGING: 
            charging();
            break;
        default:
            // The system can't not have a state.
            break; // do nothing
    }   
}

// This ISR is called when the ADC completes a conversion.
ISR(ADC_vect)
{
    // read the integer result from the ADC result register
    adc_value = ADC;
    // flag that executes the adc finished function in main
    adc_complete_flag = 1;
    // start another ADC conversion
    ADCSRA |= (1 << ADSC);
}

// This ISR is called when the control timer is done.
ISR(TIMER0_COMPA_vect)
{
    // Reset the control timer.
    TCNT0 = 0;
    // set the timer complete flag
    control_timer_complete_flag = 1;
}

// This ISR is called when the RPM timer is done.
ISR(TIMER2_COMPA_vect)
{
    // Reset the RPM timer.
    TCNT2 = 0;
    // Set the timer complete flag.
    set_rpm_timer_flag(&my_rpm_meter);
}

// Increment the number of counts received from the encoder.
ISR(PCINT0_vect)
{
    // Check to make sure the pin change is a rising edge.
    if(PINB & (1 << PINB0))
    {
        increment_encoder_counts(&my_rpm_meter);
    }
}
