# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 18:16:10 2022

@author: nadur
"""
import serial
import time

arduino = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=.1)
time.sleep(1)


arduino.reset_input_buffer()
arduino.reset_output_buffer()

def write_read(x,retflag=None):

    arduino.write(bytes(x, 'utf-8'))

    # arduino.flush()


    data=None
    while(data is None):
        data = arduino.readline()
        if len(data)==0:
            data=None

    data=data.decode()

    if retflag is not None:
        if retflag in data:
            return True,data
        else:
            return False, data
    else:
        return True,data
# value = write_read('s\n',retflag='ok')
# print(value) # printing the value


value = write_read('-1000\n',retflag='ok')
print(value) # printing the value

# value = write_read('h\n',retflag='ok')
# print(value) # printing the value

value = write_read('g\n',retflag=None)
print(value) # printing the value
 

arduino.close()