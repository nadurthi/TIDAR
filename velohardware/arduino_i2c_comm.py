from smbus import SMBus
import time

i2cbus = SMBus(7)
i2caddress = 0x33  # Address of MCP23017 device





class ArduinoI2cStepper:
    def __init__(self,bus=7,i2caddress = 0x33 ):
        self.i2cbus = SMBus(bus)
        self.i2caddress = i2caddress

    def StringToBytes(self,val):
        retVal = []
        for c in val:
            retVal.append(ord(c))
        return retVal
    def BytesToString(selfs,rec):
        ss=[chr(rec[i]) for i in range(len(rec)) if rec[i]<150]
        return "".join(ss)

    def ping(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<p>'))
        time.sleep(0.01)
        while(1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss=self.BytesToString(rec)
            if '<p>' in ss:
                break
            time.sleep(0.05)
        return ss

    def flip(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<f>'))
        time.sleep(0.01)
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if '<cw>' in ss or '<ccw>' in ss:
                break
            time.sleep(0.05)
        return ss


    def set(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<s>'))
        time.sleep(0.01)
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if '<s>' in ss:
                break
            time.sleep(0.05)
        return ss

    def getcurrSteploc(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<g>'))
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if 'None' not in ss:
                break
            time.sleep(0.05)
        return ss

    def encoderval(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<e>'))
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if 'None' not in ss:
                break
            time.sleep(0.05)
        return ss

    def step(self,stp):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<%d>'%stp))
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if '<ok>' in ss:
                break
            time.sleep(0.1)
        return ss

    def close(self):
        self.i2cbus.close()

ard = ArduinoI2cStepper()
# self=ard
ard.encoderval()

ard.ping()

ard.getcurrSteploc()

ard.step(0)
