'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
##from smbus import SMbus			#import SMself.bus module of I2C
from time import sleep          #import
import threading

class Gyro(threading.Thread):
    

    def __init__(self):
        threading.Thread.__init__(self)
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
        self.bus = SMbus(1) 	# or self.bus = smself.bus.SMself.bus(0) for older version boards
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

    def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
            return value

    def run(self):


        print " Reading Data of Gyroscope and Accelerometer"    
        while True:
	
            #Read Accelerometer raw value
            acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
            acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
            acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)
	
            #Read Gyroscope raw value
            gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
            gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
            gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
            
            #Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = acc_x/16384.0
            Ay = acc_y/16384.0
            Az = acc_z/16384.0
	
            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0
	

            print Gx, Gy, Gz, Ax, Ay, Az 	
            sleep(1)
