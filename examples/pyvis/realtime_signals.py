#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vispy: gallery 2
# Copyright (c) Vispy Development Team. All Rights Reserved.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.

"""
Multiple real-time digital signals with GLSL-based clipping.
"""

import serial
from struct import *

from vispy import gloo
from vispy import app
import numpy as np
import math
import sys
import time
from datetime import datetime

PAYLOAD_STC_FORMAT = '<BHhhhhhhB'
PAYLOAD_SIZE = 16

s = serial.Serial('/dev/ttyUSB0', 250000)

# Number of cols and rows in the table.
nrows = 21
ncols = 1

# Number of signals.
m = nrows*ncols

# Number of samples per signal.
n = 1000

# Various signal amplitudes.
amplitudes = .1 + .2 * np.random.rand(m, 1).astype(np.float32)

# Generate the signals as a (m, n) array.
y = amplitudes * np.random.randn(m, n).astype(np.float32)
y = np.zeros((m,n), np.float32)

# Color of each vertex (TODO: make it more efficient by using a GLSL-based
# color map and the index).
color = np.repeat(np.random.uniform(size=(m, 3), low=.5, high=.9),
                  n, axis=0).astype(np.float32)

# Signal 2D index of each vertex (row and col) and x-index (sample index
# within each signal).
index = np.c_[np.repeat(np.repeat(np.arange(ncols), nrows), n),
              np.repeat(np.tile(np.arange(nrows), ncols), n),
              np.tile(np.arange(n), m)].astype(np.float32)

VERT_SHADER = """
#version 120

// y coordinate of the position.
attribute float a_position;

// row, col, and time index.
attribute vec3 a_index;
varying vec3 v_index;

// 2D scaling factor (zooming).
uniform vec2 u_scale;

// Size of the table.
uniform vec2 u_size;

// Number of samples per signal.
uniform float u_n;

// Color.
attribute vec3 a_color;
varying vec4 v_color;

// Varying variables used for clipping in the fragment shader.
varying vec2 v_position;
varying vec4 v_ab;

void main() {
    float nrows = u_size.x;
    float ncols = u_size.y;

    // Compute the x coordinate from the time index.
    float x = -1 + 2*a_index.z / (u_n-1);
    vec2 position = vec2(x - (1 - 1 / u_scale.x), a_position);

    // Find the affine transformation for the subplots.
    vec2 a = vec2(1./ncols, 1./nrows)*.9;
    vec2 b = vec2(-1 + 2*(a_index.x+.5) / ncols,
                  -1 + 2*(a_index.y+.5) / nrows);
    // Apply the static subplot transformation + scaling.
    gl_Position = vec4(a*u_scale*position+b, 0.0, 1.0);

    v_color = vec4(a_color, 1.);
    v_index = a_index;

    // For clipping test in the fragment shader.
    v_position = gl_Position.xy;
    v_ab = vec4(a, b);
}
"""

FRAG_SHADER = """
#version 120

varying vec4 v_color;
varying vec3 v_index;

varying vec2 v_position;
varying vec4 v_ab;

void main() {
    gl_FragColor = v_color;

    // Discard the fragments between the signals (emulate glMultiDrawArrays).
    if ((fract(v_index.x) > 0.) || (fract(v_index.y) > 0.))
        discard;

    // Clipping test.
    vec2 test = abs((v_position.xy-v_ab.zw)/v_ab.xy);
    if ((test.x > 1) || (test.y > 1))
        discard;
}
"""

class SensorData():
    def __init__(self, id, t, accX, accY, accZ, gyroX, gyroY, gyroZ):
        self.__id  = id
        self.__temperature = t
        self.__acc = [accX, accY, accZ]
        self.__gyro = [gyroX, gyroY, gyroZ]
        
    def temperature_from_raw(self, t):
        return t * 0.00390625

    def acc_from_raw(self, d):
        #(float)in * 0.061 * (8 >> 1) / 1000.0;
        return d * 0.061 * 4.0 / 1000.0

    def dps_from_raw(self, d):
        #(float)in * 4.375 * (2000 / 125) / 1000.0;
        return d * 4.375 * 16 / 1000.0
        
    @property
    def id(self):
        return self.__id
    
    @property
    def temperature(self):
        return self.temperature_from_raw(self.__temperature)
    
    @property
    def acc(self):
        return [self.acc_from_raw(x) for x in self.__acc]
    
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


class Canvas(app.Canvas):
    def __init__(self):
        app.Canvas.__init__(self, title='Use your wheel to zoom!',
                            keys='interactive')
        self.program = gloo.Program(VERT_SHADER, FRAG_SHADER)
        self.program['a_position'] = y.reshape(-1, 1)
        self.program['a_color'] = color
        self.program['a_index'] = index
        self.program['u_scale'] = (1., 25.)
        self.program['u_size'] = (nrows, ncols)
        self.program['u_n'] = n

        gloo.set_viewport(0, 0, *self.physical_size)

        self._timer = app.Timer('auto', connect=self.on_timer, start=True)

        gloo.set_state(clear_color='black', blend=True,
                       blend_func=('src_alpha', 'one_minus_src_alpha'))

        self.show()

    def on_resize(self, event):
        gloo.set_viewport(0, 0, *event.physical_size)

    def on_mouse_wheel(self, event):
        dx = np.sign(event.delta[1]) * .05
        scale_x, scale_y = self.program['u_scale']
        scale_x_new, scale_y_new = (scale_x * math.exp(2.5*dx),
                                    scale_y * math.exp(0.0*dx))
        self.program['u_scale'] = (max(1, scale_x_new), max(1, scale_y_new))
        self.update()

    def on_timer(self, event):
        """Add some data at the end of each signal (real-time signals)."""
        #k = 1
        #y[:, :-k] = y[:, k:]
        #y[:, -k:] = amplitudes * np.random.randn(m, k)
        print(y)

        self.program['a_position'].set_data(y.ravel().astype(np.float32))
        self.update()

    def on_draw(self, event):
        gloo.clear()
        self.program.draw('line_strip')

if __name__ == '__main__':
    canv = Canvas()
    while True:
        c = s.read(1)
        if c == b'%':
            data = s.read(PAYLOAD_SIZE)
            crc = compute_crc(data)
            unpacked = unpack(PAYLOAD_STC_FORMAT, data)
            
            if unpacked[-1] == crc:
                sensorData = SensorData(*unpacked[:-1])
                y[sensorData.id, :-1] = y[sensorData.id, 1:]
                y[sensorData.id, -1:] = (1 / 40.0) * sensorData.temperature

                #ax_temp_window[sensorData.id]['data'].pop(0)
                #ax_temp_window[sensorData.id]['data'].append(sensorData.temperature)
                
                #acc = sensorData.acc
                #ax_acc_window[sensorData.id]['axis'][0]['data'].pop(0)
                #ax_acc_window[sensorData.id]['axis'][0]['data'].append(acc[0])
                #ax_acc_window[sensorData.id]['axis'][1]['data'].pop(0)
                #ax_acc_window[sensorData.id]['axis'][1]['data'].append(acc[1])
                #ax_acc_window[sensorData.id]['axis'][2]['data'].pop(0)
                #ax_acc_window[sensorData.id]['axis'][2]['data'].append(acc[2])
            else:
                print('Bad checksum')
        app.process_events()
    s.close()
