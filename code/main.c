/* 
 * File:   main.c
 * Author: Simon de Bakker
 *
 * Created on July 4, 2019, 3:53 PM
 */

#include "hardware.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi_ard.h"


//#define SILENCE     1

/*
 * 
 */

int uart_putchar(char c, FILE *stream)
{
  if (c == '\n')
    uart_putchar('\r', stream);
  
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  
  return 0;
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_RW);

#if defined(SILENCE)
#define rs485_printf(fmt, ...)
#else
void rs485_printf(char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    
    PORTD |= _BV(RS485_DE);
    vfprintf(&uart_str, fmt, va);
    va_end(va);
            
    _delay_ms(2);
    PORTD &= ~_BV(RS485_DE);
}
#endif

void init_uart(void)  
{
    UBRR0L = (F_CPU / (16UL * RS485_BAUDRATE)) - 1;
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    DDRD |= (1 << RS485_DE);
}

void init_leds(void)
{
    DDRB = (_BV(LED_GREEN1) | _BV(LED_GREEN2) | _BV(LED_RED));
    PORTB &= ~(_BV(LED_GREEN1) | _BV(LED_GREEN2) | _BV(LED_RED));
}

void init_signals(void)
{
    /* input signal, pull-up enabled */
    DDRD &= ~(_BV(SIG_IN));
    PORTD |= (_BV(SIG_IN));
    
    /* output signal, tri-state */
    DDRD &= ~(_BV(SIG_OUT));
    PORTD &= ~(_BV(SIG_OUT));
}

int main(int argc, char** argv) 
{
    DDRB = (1 << LED_GREEN1) | (1 << LED_GREEN2) | (1 << LED_RED);
    
    init_leds();
    init_signals();
    init_uart();
    twi_init();
    
    sei();
    
    stdout = &uart_str;
    
    for (;;) {
        uint8_t retval = 0;
        uint8_t temp_register[1] = { 0 };
        uint8_t temp_data[2];
        
        retval = twi_writeTo(MAX30205_ADDR, temp_register, 1, 1, 0);
        if (retval != 0) {
            rs485_printf("TWI write error: %d\n", retval);
        } else {
            retval = twi_readFrom(MAX30205_ADDR, temp_data, 2, 1);
            if (retval != 2) {
                rs485_printf("TWI read error %d\n", retval);
            } else { 
                uint16_t raw = (temp_data[0] << 8) | temp_data[1];
                float temp = raw * 0.00390625;
                rs485_printf("Temperature: %f\r", temp);
            }
        }

        PORTB = _BV(LED_GREEN1);
        _delay_ms(50);
        PORTB = _BV(LED_GREEN2);
        _delay_ms(50);
        PORTB = _BV(LED_RED);
        _delay_ms(50);
    }

    return (0);
}

