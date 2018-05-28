'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''

import smbus	#import SMself.bus module of I2C
from time import sleep          #import
import threading

class gyro():

    def __init__(self):
        #some MPU6050 Registers and their Address
        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47
        self.Ax = 0
        self.Ay = 0
        self.Az = 0
        self.Gx = 0
        self.Gy = 0
        self.Gz = 0
        self.bus = smbus.SMBus(1)

        self.Device_Address = 0x68   # MPU6050 device address
        
        #write to sample rate register
	self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
	
	#Write to power management register
	self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)
	
	#Write to Configuration register
	self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)
	
	#Write to Gyro configuration register
	self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self,addr):
	#Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value

    def get_value(self):
        return self.Ax,self.Ay,self.Az,self.Gx,self.Gy,self.Gz
    
    def get_str_value(self):   
        #Read Accelerometer raw value
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)
    
        #Read Gyroscope raw value
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        self.Ax = float(("%.2f" % (acc_x/16384.0)))
        self.Ay = float(("%.2f" % (acc_y/16384.0)))
        self.Az = float(("%.2f" % (acc_z/16384.0)))
    
        self.Gx = float(("%.2f" % (gyro_x/131.0)))
        self.Gy = float(("%.2f" % (gyro_y/131.0)))
        self.Gz = float(("%.2f" % (gyro_z/131.0)))
        
        return str(self.Ax),str(self.Ay),str(self.Az),str(self.Gx),str(self.Gy),str(self.Gz)
