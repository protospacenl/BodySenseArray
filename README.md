# BodySenseArray
Body Sensor Array

## Data transfer
Data transfer between sensor units uses RS485. Use a RS485 to usb converter to send data over to the PC.

Serial settings:
 * Baudrate: 250000
 * 8n1
 * no flow control

## Sensor data format
Each packet starts with a `%` start token and ends with a CRC. Check the crc to verify data integrity. If crc is not valid the received packet should be discarded.

 * byte order: little endian
 * format: `%<id><temperature><acc_x><acc_y><acc_z><gyro_x><gyro_y><gyro_z><crc>`

field | type
--- | ---
% | uint8_t
id | uint8_t
temperature | uint16_t
acc_[xyz] | int16_t
gyro_[xyz] | int16_t
crc | uint8_t

## CRC
CRC algorithm used is the [Dalles iButton 8-bit CRC](https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1ga37b2f691ebbd917e36e40b096f78d996.html).

Example C implementation  (taken from above link):
```C
uint8_t
_crc_ibutton_update(uint8_t crc, uint8_t data)
{
    uint8_t i;

    crc = crc ^ data;
    for (i = 0; i < 8; i++)
    {
        if (crc & 0x01)
            crc = (crc >> 1) ^ 0x8C;
        else
            crc >>= 1;
    }
    
    return crc;
}
```
or in python (see [examples](https://github.com/protospacenl/BodySenseArray/blob/90c5e3de4db39f0ca5cd2b3785d3c42a4cba6765/examples/serial_read/serial_read.py#L60) for usage)
```python
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
```

## Conversions
For efficiency the sensor data is in its integer form and needs to be translated to real world values. 
See the [examples](https://github.com/protospacenl/BodySenseArray/tree/master/examples) for implementation details.

### Temperature
```python
def temperature_from_raw(t):
    return t * 0.00390625
```

### acceleration
Translation from the raw acceleration values to *g*:
`(float)in * 0.061 * (g >> 1) / 1000.0`  
where *in* is the raw 16-bit two's complement value and *g* is the sensitivity (currently 2).

Range for 2g:
 - min: -1.99848g
 - max: 1.99787g

Example:
```python
def acc_from_raw(in, g=2):
    return in * 0.061 * (g >> 1) / 1000.
 ```

### angular rate
Translation from the raw angular rate to *dps*:
`(float)in * 4.375 * (dps / 125) / 1000.0`
where *in* is the raw 16-bit two's complement value and *dps* is the sensitivity (currently 2000).

Range for 2000dps:
 - min: -2293.76 dps
 - max: 2293.69 dps

Example:
```python
def dps_from_raw(in, dps=2000):
  return in * 4.375 * (dps / 125) / 1000.
```

