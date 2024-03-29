import serial
from struct import *

import math
import sys
import time
from datetime import datetime


PAYLOAD_STC_FORMAT = '<BHhhhhhhB'
PAYLOAD_SIZE = 16

FORMAT_STRING = "Sensor {}: t={:2.3f},\t g=[x:{:9.6f}, y:{:9.6f}, z:{:9.6f}],\t  dps=[x:{:8.2f}, y:{:8.2f}, z:{:8.2f}]"

s = serial.Serial('/dev/ttyUSB0', 250000)

class SensorData():
    def __init__(self, id, t, accX, accY, accZ, gyroX, gyroY, gyroZ):
        self.__id  = id
        self.__temperature = t
        self.__acc = [accX, accY, accZ]
        self.__gyro = [gyroX, gyroY, gyroZ]
        
    def temperature_from_raw(self, t):
        return t * 0.00390625

    def acc_from_raw(self, d, g=2):
        """
          8g: (float)in * 0.061 * (8 >> 1) / 1000.0;  min: -7.995392g max: 7.995148g
          2g: (float)in * 0.061 * (2 >> 1) / 1000.0;  min: -1.99848g max: 1.99787g
        """
        return d * 0.061 * (g >> 1) / 1000.0

    def dps_from_raw(self, d):
        """
          (float)in * 4.375 * (2000 / 125) / 1000.0;
          min: -2293.76 dps
          max: 2293.69 dps
        """
        return d * 4.375 * 16 / 1000.0

        
    @property
    def id(self):
        return self.__id
    
    @property
    def temperature(self):
        return self.temperature_from_raw(self.__temperature)
    
    @property
    def acc(self):
        return [self.acc_from_raw(x, g=2) for x in self.__acc]
    
    @property
    def gyro(self):
        return [self.dps_from_raw(x) for x in self.__gyro]


def compute_crc(data):
    crc = 0x00
    for c in data[:-1]:
        crc = crc ^ c
        for i in range(0, 8):
            if crc & 0x01:
                crc  = (crc >> 1) ^ 0x8c
            else:
                crc = crc >> 1
    return crc

if __name__ == '__main__':
    while True:
        c = s.read(1)
        if c == b'%':
            data = s.read(PAYLOAD_SIZE)
            crc = compute_crc(data)
            unpacked = unpack(PAYLOAD_STC_FORMAT, data)
            
            if unpacked[-1] == crc:
                sensorData = SensorData(*unpacked[:-1])

                temp = sensorData.temperature
                acc = sensorData.acc
                gyro = sensorData.gyro

                print(FORMAT_STRING.format(sensorData.id, temp, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]))

            else:
                print('Bad checksum')
    s.close()
