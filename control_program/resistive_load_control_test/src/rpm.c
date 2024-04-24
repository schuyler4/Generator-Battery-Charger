// encoder parameters
#define ENCODER_PULSES_PER_REV 600
#define SEC_PER_MIN 60
// the timer comparison for 1 second
#define TIMER_COUNT 15624

// the number of pulses counted coming from the encoder
// the encoder is 600 pulses per revolution
static volatile uint32_t encoder_counts = 0;

// configure timer 1 for RPM measurement
void setup_rpm_timer(void)
{
    // set the timer to fcpu divide by 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);

    // enable compare match interupt
    TIMSK1 |= (1 << OCIE1A);

    // set the timer count
    OCR1AH = (TIMER_COUNT >> BYTE_SIZE) & BYTE_MASK;
    OCR1AL = TIMER_COUNT & BYTE_MASK;
}

// setup the pin change interupt to measure RPM with encoder
void setup_pin_change_interupt(void)
{
    // turn on the pin change interupt on port B
    PCICR |= (1 << PCIE0);
    // set pin change interupt on PORTB0
    PCMSK0 |= (1 << PCINT0);
}

// called when the rpm timer has elapsed
void rpm_timer_complete(uint32_t *encoder_counts)
{
    // executes when the rpm timer has completed
    // COMMENTED CODE IN MAIN
    //if(timer_complete_flag)
    //{
    //    rpm_timer_complete(&encoder_counts);
    //    timer_complete_flag = 0;
    //}
    float rev_per_sec = (float)(*encoder_counts)/ENCODER_PULSES_PER_REV;
    *encoder_counts = 0;
    UART_transmit_int((int)(rev_per_sec*SEC_PER_MIN));
    UART_transmit_string("\n\r");
}

// increment the number of counts received from the encoder
ISR(PCINT0_vect)
{
    // check to make sure the pin change is a rising edge
    if(PINB & (1 << PINB0))
    {
        encoder_counts++;
    }
}
