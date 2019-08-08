/* 
 * File:   hardware.h
 * Author: simon
 *
 * Created on July 4, 2019, 11:52 PM
 */

#ifndef HARDWARE_H
#define	HARDWARE_H

#ifdef	__cplusplus
extern "C" {
#endif

#define F_CPU           8000000UL
#define RS485_BAUDRATE  250000
    
#define LED_RED         PB1
#define LED_GREEN2      PB0
#define LED_GREEN1      PB2
#define SIG_IN          PD2
#define SIG_OUT         PD6
#define RS485_DE        PD5
    
#define MAX30205_ADDR   0x4F    /* temperature sensor */
#define LSM6DS3_ADDR    0x6A    /* IMU accel / gyro */

#ifdef	__cplusplus
}
#endif

#endif	/* HARDWARE_H */

