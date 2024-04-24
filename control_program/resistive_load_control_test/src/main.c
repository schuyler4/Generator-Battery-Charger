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
#include "PID_controller.h"

#include "main.h"
  
// general byte manipulation
#define BYTE_SIZE 8
#define BYTE_MASK 0xFF

// logging
#define NEWLINE_AND_CARRIAGE_RETURN "\n\r"

// battery current reading parameters
#define CURRENT_FILTER_LENGTH 5

// controller parameters
#define PROPORTIONAL 1
#define INTEGRAL 0
#define DERIVATIVE 0
#define BATTERY_CURRENT_ADC_CODE_SET_POINT 120
#define SAMPLE_PERIOD 0.01

#define CHARGER_ON_ADC_THRESHOLD 20

// full throttle time: 1.36ms
#define FULL_THROTTLE_COMPARE_MATCH 2775
// low throttle pulse time: 1.51ms
#define LOW_THROTTLE_COMPARE_MATCH 3020

// this flag gets flipped when the ADC conversion is complete
static volatile unsigned int adc_complete_flag = 0;
// this flag gets flipped when the timer finishes
static volatile unsigned int control_timer_complete_flag = 0;

// this stores the ADC value
static volatile unsigned int adc_value;

// instantiate filter for adc current sampling
Running_Average_Filter current_adc_filter;

// instantiate the PID Controller
PID_Controller controller;

Charger_State state;

float compare_match_command = 200;
uint16_t warmup_count = 0;
int average_current_adc_value = 0;

int main(void)
{
    // disable interupts for setup
    cli(); 
    // call setup functions
    setup_IO();
    setup_servo_timer();
    setup_control_timer();
    setup_ADC();
    setup_UART();
    // reenable interupts
    sei();

    UART_transmit_string("setup complete\n\r");

    // start the first ADC conversion
    ADCSRA |= (1 << ADSC);

    DDRB |= (1 << DDB0);
    PORTB |= (1 << PORTB0);

    // set the length of the running average filter
    current_adc_filter.length = CURRENT_FILTER_LENGTH;

    setup_PID_controller(
        &controller, 
        PROPORTIONAL, 
        INTEGRAL, 
        DERIVATIVE, 
        BATTERY_CURRENT_ADC_CODE_SET_POINT, 
        SAMPLE_PERIOD);

    state = WARMING_UP;

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

        control_timer_complete();
    }

    return 1;
}

// set up the I/O pins as inputs or outputs
void setup_IO(void)
{
    // set the servo control pin to an output
    DDRB |= (1 << DDB4);
    // set D11 for debug
    DDRB |= (1 << DDB3);
    DDRD |= (1 << DDD3);
}

// configure the ADC registers to setup an interupt ADC sample
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

// setup the timer that is used to time
// the sampling for the PID loop
void setup_control_timer(void)
{
    // set the timer to fcpu divide by 1024
    TCCR0B |= (1 << CS02) | (1 << CS00);
    // enable compare match interupt
    TIMSK0 |= (1 << OCIE0A);
    // set the timer count for 10ms // OLD VALUE 156 
    OCR0A = 156;
}

// configure timer 1 for servo control
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

// this function executes when the ADC has finished its conversion
void adc_complete(unsigned int adc_value)
{
    current_adc_filter.sample = adc_value;
    average_current_adc_value = running_average_filter_sample(&current_adc_filter);

    UART_transmit_string("ADC Value: ");
    UART_transmit_int(average_current_adc_value);
    UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);

    // update the state to check if the generator is on
    if(average_current_adc_value >= CHARGER_ON_ADC_THRESHOLD && state == OFF)
    {
        state = WARMING_UP;
    }
    else if(average_current_adc_value <= CHARGER_ON_ADC_THRESHOLD && state != OFF)
    {
        state = OFF;
    }
}

void log_state(void)
{
    if(state == OFF)
    {
        UART_transmit_string("OFF");
        UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    }
    if(state == WARMING_UP)
    {
        UART_transmit_string("WARMING UP");
        UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    }
    if(state == CHARGING)
    {
        UART_transmit_string("CHARGING");
        UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    }
}

// checks if the control timer is complete and 
// executes the control algorithm if it is
void control_timer_complete(void)
{
    if(control_timer_complete_flag)
    {
        log_state();
        update_control_system();
        // TODO: implement control timer update algorithm
        control_timer_complete_flag = 0;
    }
}

void update_control_system(void)
{
    if(state == OFF)
    {
        // set to maximum throttle so the engine can be turned on
        OCR1A = FULL_THROTTLE_COMPARE_MATCH;
        warmup_count = 0;
    }
    if(state == WARMING_UP)
    {
        // the engine should be at full throttle to warm up
        OCR1A = FULL_THROTTLE_COMPARE_MATCH;
        warmup_count++;
        UART_transmit_int(warmup_count);
        UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    }
    // check for a state change to charging
    if(warmup_count >= 1000 && state == WARMING_UP)
    {
        state = CHARGING;
    }
    if(state == CHARGING)
    {
        OCR1A = LOW_THROTTLE_COMPARE_MATCH - ((int)compare_match_command);
    }

    uint8_t range_min = LOW_THROTTLE_COMPARE_MATCH - compare_match_command >= FULL_THROTTLE_COMPARE_MATCH;
    uint8_t range_max = LOW_THROTTLE_COMPARE_MATCH - compare_match_command <= LOW_THROTTLE_COMPARE_MATCH;

    if(state == CHARGING)
    {
        if(range_min && average_current_adc_value < BATTERY_CURRENT_ADC_CODE_SET_POINT)
        {
            //UART_transmit_string("Updating Set Point Plus");
            //UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
            compare_match_command += 0.1;
        }
        else if(range_max && average_current_adc_value > BATTERY_CURRENT_ADC_CODE_SET_POINT)
        {
            //UART_transmit_string("Updating Set Point Minus");
            //UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
            compare_match_command -= 0.1;
        }
        UART_transmit_string("Compare Match Command: ");
        UART_transmit_int(compare_match_command);
        UART_transmit_string(NEWLINE_AND_CARRIAGE_RETURN);
    }
}

// called when the ADC completes a conversion
ISR(ADC_vect)
{
    // read the integer result from the ADC result register
    adc_value = ADC;
    // flag that executes the adc finished function in main
    adc_complete_flag = 1;
    // start another ADC conversion
    ADCSRA |= (1 << ADSC);
}

ISR(TIMER0_COMPA_vect)
{
    // reset the timer
    TCNT0 = 0;
    // set the timer complete flag
    control_timer_complete_flag = 1;
}