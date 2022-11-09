# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 18:16:10 2022

@author: nadur
"""
import serial
import time





class ArduinoCtrl:
    def __init__(self,port='/dev/ttyACM0'):
        self.serialcom = serial.Serial(port=port, baudrate=9600, timeout=None)
        time.sleep(5)
        self.serialcom.reset_input_buffer()
        self.serialcom.reset_output_buffer()
        
    def write_read(self,x):
    
        self.serialcom.write(bytes(x, 'utf-8'))
    
        data=None
        while(data is None):
            data = self.serialcom.readline()
            if len(data)==0:
                data=None
    
        data=data.decode()
    
        return data
    def close(self):
        self.serialcom.close()
        
# value = write_read('s\n',retflag='ok')
# print(value) # printing the value

ard=ArduinoCtrl()

value = ard.write_read('<s>')
print(value)  # printing the value


value = ard.write_read('<2050>')
print(value) # printing the value

ard.close()