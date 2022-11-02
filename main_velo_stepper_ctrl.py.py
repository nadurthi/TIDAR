# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 18:16:10 2022

@author: nadur
"""
import serial
import time

arduino = serial.Serial(port='COM3', baudrate=9600, timeout=.1)
def write_read(x,retflag=None):
    arduino.write(bytes(x, 'utf-8'))
    data=None
    while(1):
        time.sleep(0.01)
        data = str(arduino.readline())
        if retflag is None:
            if data is not None:
                break
        else:
            if data is not None:
                if retflag in data:
                    break
    return data
# value = write_read('s\n',retflag='ok')
# print(value) # printing the value


value = write_read('h\n',retflag='ok')
print(value) # printing the value

value = write_read('g\n',retflag=None)
print(value) # printing the value
 

