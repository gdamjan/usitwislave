#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "usitwislave.h"
#include "usitwislave_devices.h"

/*
 * PB2 & PB0 I2C
 * PB1 LED PWM controlled
 *
 */

#define USE_SLEEP 0
#define SLAVE_ADDRESS 0x10

static void twi_callback (uint8_t input_buffer_length, const uint8_t *input_buffer,
                          uint8_t *output_buffer_length, uint8_t *output_buffer)
{
    OCR0B = input_buffer[0];
    *output_buffer_length = 0;
}

static void blink_once(void)
{
    PORTB &= ~(1<<PB1);   /* LED on */
    _delay_ms(1000);
    PORTB |= 1<<PB1;      /* LED off */
    _delay_ms(1000);
    PORTB &= ~(1<<PB1);   /* LED on */
    _delay_ms(1000);
}

int main(void)
{
    // init
    DDRB |= 1<<PB1; /* set PB1 to output */
    blink_once();

    // set PWM to half by default
    // Timer0, Set OC0A on comp. match (inv). Mode 3: Fast PWM
    TCCR0A = (1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1<<WGM00);
    // 1:8 presc.
    TCCR0B = (1<<CS01);

    // set to half PWM on start
    OCR0B = 0x7f;

    // run usi_twi_slave loop
    usi_twi_slave(SLAVE_ADDRESS, USE_SLEEP, &twi_callback, NULL);
    return 0;
}
