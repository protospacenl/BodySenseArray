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
#include <util/crc16.h>
#include "twi_ard.h"
#include "LM6DS3.h"

volatile uint32_t _ticks = 0;
volatile uint8_t _sig_in_recv = 0;
volatile uint8_t _sig_cntr = 0;

//#define SILENCE     1
#define PACKAGE_START_TOKEN '%'
/*
 * 
 */

struct _sensor_data_stc {
    uint8_t id;
    uint16_t t;
    int16_t acc[3];
    int16_t gyro[3];
}  __attribute__((packed));
typedef struct _sensor_data_stc sensor_data_stc;

#define SENSOR_DATA_SIZE    sizeof(sensor_data_stc)

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

int8_t get_temperature_raw(uint16_t *t)
{
    uint8_t retval = 0;
    uint8_t temp_register[1] = { 0 };
    uint8_t temp_data[2];
    
    retval = twi_writeTo(MAX30205_ADDR, temp_register, 1, 1, 0);
    if (retval != 0) {
        return -1;
    }
    
    retval = twi_readFrom(MAX30205_ADDR, temp_data, 2, 1);
    if (retval != 2) {
        return -2;
    }
    
    *t = (temp_data[0] << 8) | temp_data[1];
    
    return 0;  
}

int8_t get_temperature(float *t)
{
    int8_t retval;
    uint16_t raw;
    
    retval = get_temperature_raw(&raw);
    if (retval < 0) {
        return retval;
    }
       
    *t = raw * 0.00390625;
    
    return 0;
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

int8_t init_IMU(void)
{
    int8_t retval;
    uint8_t data = 0;
    uint8_t cmd[2];
    
    /* accelerometer init */
    data |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
    data |= LSM6DS3_ACC_GYRO_FS_XL_2g;
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
    /* 2 == accel range */
    float out = (float)in * 0.061 * (2 >> 1) / 1000.0;
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

static inline void get_sensor_data(sensor_data_stc *data)
{
    IMU_read_acc_XYZ(data->acc);
    IMU_read_gyro_XYZ(data->gyro);
    get_temperature_raw(&data->t);
}

static inline void toggle_sig_out(void)
{
    _delay_ms(2);
    PORTD |= _BV(SIG_OUT);
    _delay_ms(1);
    PORTD &= ~(_BV(SIG_OUT));  
}

static inline uint8_t compute_crc(sensor_data_stc *data)
{
    uint8_t *d = (uint8_t*)data;
    uint8_t crc = 0x00;
    uint8_t i = 0;
    
    crc = _crc_ibutton_update(crc, PACKAGE_START_TOKEN);
    for (i=0; i<SENSOR_DATA_SIZE; i++) {
        crc = _crc_ibutton_update(crc, d[i]);
    } 
    return crc;
}

static inline uint8_t send_data(sensor_data_stc *data)
{
    uint8_t *d = (uint8_t*)data;
    uint8_t crc = 0x00;
    uint8_t i = 0;
    
    PORTD |= _BV(RS485_DE);
    
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = PACKAGE_START_TOKEN;
        
    for (i=0; i<SENSOR_DATA_SIZE; i++) {
        uint8_t c = d[i];
        
        crc = _crc_ibutton_update(crc, c);
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = c;
    }
    
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = crc;
    
    _delay_ms(1);
    PORTD &= ~_BV(RS485_DE);
    
    return crc;
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
    
#if defined(DEBUG)
    rs485_printf("ID:%d,%d\n", id, _sig_cntr);;
#endif
    
    for (i=0; i<50; i++) {
        _delay_ms(100);
    }
    
    PORTB &= ~((1 << LED_GREEN1) | (1 << LED_GREEN2) | (1 << LED_RED));
    
    if (IMU_available()) {        
        init_IMU();
#if defined(DEBUG)
        rs485_printf("Found IMU\n");
#endif
    }
    
    for (;;) {
#if 1
        if (id == 0) {
            if (_ticks > old_tick_val + 50) { 
                sensor_data_stc data;
                uint8_t crc;
                
                PORTB |= _BV(LED_RED);
                
                data.id = id;
                get_sensor_data(&data);
                crc = send_data(&data);
                
#if defined(DEBUG)
                rs485_printf(" -- 0x%.2x\n", crc);
                rs485_printf("%c%d;t:%d;a:%d,%d,%d;g:%d,%d,%d - 0x%.2x\n", '#', data.id, data.t, data.acc[0], data.acc[1], data.acc[2], data.gyro[0], data.gyro[1], data.gyro[2], crc);
#endif
                
                old_tick_val = _ticks;
                
                toggle_sig_out();
                
                PORTB &= ~(_BV(LED_RED));
            } 
        } else { 
            if (_sig_in_recv) {
                sensor_data_stc data;
                uint8_t crc;
                
                cli();
                _sig_in_recv = 0;
                sei();
                
                PORTB |= _BV(LED_GREEN2);
                
                data.id = id;
                get_sensor_data(&data);
                crc = send_data(&data);
                
#if defined(DEBUG)
                rs485_printf(" -- 0x%.2x\n", crc);
                rs485_printf("%c%d;t:%d;a:%d,%d,%d;g:%d,%d,%d - 0x%.2x\n", '#', data.id, data.t, data.acc[0], data.acc[1], data.acc[2], data.gyro[0], data.gyro[1], data.gyro[2], crc);
#endif
                
                toggle_sig_out();
                
                PORTB &= ~(_BV(LED_GREEN2));
            }
        }
#endif
    }

    return (0);
}