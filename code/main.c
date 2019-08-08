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
#include "LM6DS3.h"

volatile uint32_t _ticks = 0;
volatile uint8_t _sig_in_recv = 0;
volatile uint8_t _sig_cntr = 0;

//#define SILENCE     1

/*
 * 
 */

ISR(TIMER0_COMPA_vect)
{  
    _ticks++;
}

ISR(INT0_vect)
{
    _sig_in_recv = 1;
    _sig_cntr++;
}

int uart_putchar(char c, FILE *stream)
{
  if (c == '\n')
    uart_putchar('\r', stream);
  
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  
  return 0;
}

int uart_getchar(FILE *stream) 
{
    return 0;
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

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
            
    _delay_ms(1);
    PORTD &= ~_BV(RS485_DE);
}
#endif

void init_sys_timer(void)
{
    TCCR0A = 0;
    TCCR0B = 0;
    
    /* CTC */
    TCCR0A &= ~(_BV(WGM00));
    TCCR0A |= _BV(WGM01);
    TCCR0B &= ~(_BV(WGM02));
    
    /* Prescaler 1024 */
    TCCR0B |= (_BV(CS00) | _BV(CS02));
    
    /* 0.001024 ms */
    OCR0A = 0x06;
    
    /* Toggle OC0A (SIG_OUT) */
    //DDRD |= _BV(SIG_OUT);
    //TCCR0A |= _BV(COM0A0);
    
    /* enable interrupt */
    TIMSK0 |= _BV(OCIE0A);
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
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
    
    /* setup interrupt INT0 */
    EICRA |= (_BV(ISC00) | _BV(ISC01));
    EIMSK |= _BV(INT0);
    
    /* output signal, tri-state */
    //DDRD &= ~(_BV(SIG_OUT));
    //PORTD &= ~(_BV(SIG_OUT));
    DDRD |= _BV(SIG_OUT);
    PORTD &= ~(_BV(SIG_OUT));
}

int discover_id(void)
{
    uint8_t id = 0;
    uint8_t i;
    uint32_t start_time = 0;
    uint16_t wait_time = 1000;

    if (PIND & _BV(SIG_IN)) {
        PORTD |= _BV(SIG_OUT);
        _delay_ms(1);
        PORTD &= ~(_BV(SIG_OUT));
    } else {
    
        start_time = _ticks;
        do {
            if (_sig_in_recv)  {
                cli();
                _sig_in_recv = 0;
                sei();
                
                id++;
                wait_time = 10;
                start_time = _ticks;
            }
        } while(_ticks < start_time + wait_time);
        
        for (i=0; i<=id; i++) {
            PORTD |= _BV(SIG_OUT);
            _delay_ms(1);
            PORTD &= ~(_BV(SIG_OUT));
            _delay_ms(1);
        }
    }
    return id;
}

float get_temperature(void)
{
    uint8_t retval = 0;
    uint8_t temp_register[1] = { 0 };
    uint8_t temp_data[2];
    float temp = -1.0;
    
    retval = twi_writeTo(MAX30205_ADDR, temp_register, 1, 1, 0);
    if (retval != 0) {
        return -1;
    }
    
    retval = twi_readFrom(MAX30205_ADDR, temp_data, 2, 1);
    if (retval != 2) {
        return -2;
    } else { 
        uint16_t raw = (temp_data[0] << 8) | temp_data[1];
        temp = raw * 0.00390625;
    }
    
    return temp;
}

int8_t IMU_read_register(uint8_t offset, uint8_t *data, uint8_t length) {
    uint8_t retval;
    
    retval = twi_writeTo(LSM6DS3_ADDR, &offset, 1, 1, 0);
    if (retval != 0) {
        return -1;
    }
    
    retval = twi_readFrom(LSM6DS3_ADDR, data, length, 1);
    if (retval != length) {
        return -2;
    }
    
    return 0;
}

int8_t IMU_read_register_int16(uint8_t offset, int16_t *data)
{
    uint8_t retval;
    uint8_t bfr[2];
    
    retval = IMU_read_register(offset, bfr, 2);
    if (retval < 0) {
        return retval;
    }
    
    *data = bfr[0] | (bfr[1] << 8);
    
    return 0;
}

int8_t IMU_write_register(uint8_t *cmd, uint8_t length) 
{
    uint8_t retval;
    
    retval = twi_writeTo(LSM6DS3_ADDR, cmd, length, 1, 1);
    if (retval != 0) {
        return -1;
    }
    
    return 0;
}

uint8_t IMU_available(void)
{
    uint8_t retval;
    uint8_t result;
    
    retval = IMU_read_register(LSM6DS3_ACC_GYRO_WHO_AM_I_REG, &result, 1);
    if (retval < 0) {
        return 0;
    }
    
    if (result != 0x69) {
        return 0;
    }
    
    return 1;
}

/*
 	settings.gyroEnabled = 1;  //Can be 0 or 1
	settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
	settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
	settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
	settings.gyroFifoDecimation = 1;  //set 1 for on /1

	settings.accelEnabled = 1;
	settings.accelODROff = 1;
	settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
	settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
	settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
	settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
	settings.accelFifoDecimation = 1;  //set 1 for on /1

	settings.tempEnabled = 1;

	//Select interface mode
	settings.commMode = 1;  //Can be modes 1, 2 or 3

	//FIFO control data
	settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
	settings.fifoSampleRate = 10;  //default 10Hz
	settings.fifoModeWord = 0;  //Default off
 */

int8_t init_IMU(void)
{
    int8_t retval;
    uint8_t data = 0;
    uint8_t cmd[2];
    
    /* accelerometer init */
    data |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
    data |= LSM6DS3_ACC_GYRO_FS_XL_8g;
    data |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
    
    cmd[0] = LSM6DS3_ACC_GYRO_CTRL1_XL;
    cmd[1] = data;
    retval = IMU_write_register(cmd, 2);
    if (retval < 0) {
        rs485_printf("e: Failed writing register %d\n", retval);
        return retval;
    }

    IMU_read_register(LSM6DS3_ACC_GYRO_CTRL4_C, &data, 1);
    data &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
    cmd[0] = LSM6DS3_ACC_GYRO_CTRL4_C;
    cmd[1] = data;
    IMU_write_register(cmd, 2);
    
    /* gyro init */
    data = 0;
    data |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
    data |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
    
    cmd[0] = LSM6DS3_ACC_GYRO_CTRL2_G;
    cmd[1] = data;
    IMU_write_register(cmd, 2);
    
    return 0;
}

float IMU_acc_raw_to_float(int16_t in)
{
    /* 8 == accel range */
    float out = (float)in * 0.061 * (8 >> 1) / 1000.0;
    return out;
}

float IMU_gyro_raw_to_float(int16_t in)
{
    /* 2000 == gyro range */
    float out = (float)in * 4.375 * (2000 / 125) / 1000.0;
    return out;
}

int8_t IMU_read_acc_XYZ(int16_t xyz[])
{
    int8_t retval;
    int16_t x, y, z;
    
    retval = IMU_read_register_int16(LSM6DS3_ACC_GYRO_OUTX_L_XL, &x);
    if (retval < 0) return -1;
    retval = IMU_read_register_int16(LSM6DS3_ACC_GYRO_OUTY_L_XL, &y);
    if (retval < 0) return -2;
    retval = IMU_read_register_int16(LSM6DS3_ACC_GYRO_OUTZ_L_XL, &z);
    if (retval < 0) return -3;
    
    /*
    xyz[0] = IMU_acc_raw_to_float(x);
    xyz[1] = IMU_acc_raw_to_float(y);
    xyz[2] = IMU_acc_raw_to_float(z);
    */
    
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = z;
    
    return 0;
}

int8_t IMU_read_gyro_XYZ(int16_t xyz[])
{
    int8_t retval;
    int16_t x, y, z;
    
    retval = IMU_read_register_int16(LSM6DS3_ACC_GYRO_OUTX_L_G, &x);
    if (retval < 0) return -1;
    retval = IMU_read_register_int16(LSM6DS3_ACC_GYRO_OUTY_L_G, &y);
    if (retval < 0) return -2;
    retval = IMU_read_register_int16(LSM6DS3_ACC_GYRO_OUTZ_L_G, &z);
    if (retval < 0) return -3;
    
    /*
    xyz[0] = IMU_gyro_raw_to_float(x);
    xyz[1] = IMU_gyro_raw_to_float(y);
    xyz[2] = IMU_gyro_raw_to_float(z);
    */
    
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = z;
    
    return 0;  
}

int main(int argc, char** argv) 
{
    uint8_t i;
    uint8_t id = 0;
    uint32_t old_tick_val = 0;
    
    DDRB = (1 << LED_GREEN1) | (1 << LED_GREEN2) | (1 << LED_RED);
    
    init_leds();
    init_signals();
    init_uart();
    twi_init();
    init_sys_timer();
    
    sei();
    
    stdout = &uart_str;
 
    _delay_ms(200);
    id = discover_id();
    
    switch (id) {
        case 3:
            PORTB |= (_BV(LED_GREEN2) | _BV(LED_RED));
            break;
        case 2: 
            PORTB |= (_BV(LED_GREEN1) | _BV(LED_GREEN2) | _BV(LED_RED));
            break;
        case 1:
            PORTB |= (_BV(LED_GREEN1) | _BV(LED_GREEN2));
            break;
        case 0: 
            PORTB |= _BV(LED_GREEN1); 
            break;
    }
    
    _delay_ms(id * 100);
    rs485_printf("ID:%d,%d\n", id, _sig_cntr);;
    
    for (i=0; i<50; i++) {
        _delay_ms(100);
    }
    
    PORTB &= ~((1 << LED_GREEN1) | (1 << LED_GREEN2) | (1 << LED_RED));
    
    if (IMU_available()) {
        rs485_printf("Found IMU\n");
        init_IMU();
    }
    
    for (;;) {
#if 1
        if (id == 0) {
            if (_ticks > old_tick_val + 50) { 
                int8_t retval;
                int16_t acc[3], gyro[3];
                int8_t have_acc = 0, have_gyro = 0, have_temp = 0;
                float temp;
                
                PORTB |= _BV(LED_RED);
                
                retval = IMU_read_acc_XYZ(acc);
                if (retval < 0) {
                    rs485_printf("%d,e:Error retrieving acc\n");
                } else {
                    have_acc = 1;
                }

                retval = IMU_read_gyro_XYZ(gyro);
                if (retval < 0) {
                    rs485_printf("%d,e:Error retrieving gryo\n");
                } else {
                    have_gyro = 1;
                }
                
                temp = get_temperature();
                if (temp < 0) {
                    rs485_printf("%d,e:Error retrieving temperature - %d\n", id, temp);
                } else {
                    have_temp = 1;
                }
                
                rs485_printf("%%%d;t:%f;a:%d,%d,%d;g:%d,%d,%d\n", id, temp, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
                                     
                old_tick_val = _ticks;
                
                _delay_ms(2);
                PORTD |= _BV(SIG_OUT);
                _delay_ms(1);
                PORTD &= ~(_BV(SIG_OUT));
                
                PORTB &= ~(_BV(LED_RED));
            } 
        } else { 
            if (_sig_in_recv) {
                int8_t retval;
                int16_t acc[3], gyro[3];
                int8_t have_acc = 0, have_gyro = 0, have_temp = 0;
                float temp;
                
                cli();
                _sig_in_recv = 0;
                sei();
                
                PORTB |= _BV(LED_GREEN2);
                
                retval = IMU_read_acc_XYZ(acc);
                if (retval < 0) {
                    rs485_printf("%d,e:Error retrieving acc\n");
                } else {
                    have_acc = 1;
                }

                retval = IMU_read_gyro_XYZ(gyro);
                if (retval < 0) {
                    rs485_printf("%d,e:Error retrieving gryo\n");
                } else {
                    have_gyro = 1;
                }
                
                temp = get_temperature();
                if (temp < 0) {
                    rs485_printf("%d,e:Error retrieving temperature - %d\n", id, temp);
                } else {
                    have_temp = 1;
                }
                
                rs485_printf("%%%d;t:%f;a:%d,%d,%d;g:%d,%d,%d\n", id, temp, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
                
                _delay_ms(2); 
                PORTD |= _BV(SIG_OUT);
                _delay_ms(1);
                PORTD &= ~(_BV(SIG_OUT));
                
                PORTB &= ~(_BV(LED_GREEN2));
            }
        }
#endif
    }

    return (0);
}

