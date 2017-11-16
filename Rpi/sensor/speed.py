import RPi.GPIO as GPIO
import time
import threading
import serial

class SpeedSensor:
        '''
        Funtion in this Class
        SpeedSensor()   << Constructor
        setSpeed()    << Set Default
        a_counter(c)
        b_counter(c)
        feedback_control()
        calculate_rps()
        calculate_pwn()
        '''
        def __init__(self,ir_left,ir_right):
                self.ir_left = ir_left
                self.ir_right = ir_right
                self.a_count = 0
                self.b_count = 0
                self.pwm_a = 0
                self.pwm_b = 0
                self.max_pwm = 255
                self.min_pwm = 0
                self.speed_ref = 0
                self.max_speed = 20
                self.min_speed = 1
                self.rps_a = 0
                self.rps_b = 0
                
        def getIr_left(self):
                return int(self.ir_left)
        def getIr_right(self):
                return int(self.ir_right)

        def setSpeed(self):
                GPIO.setwarnings(False)
                # GPIO Setup
                GPIO.setup(self.getIr_left(),GPIO.IN)
                GPIO.setup(self.getIr_right(),GPIO.IN)
                
                #ser = serial.Serial('/dev/ttyACM0',115200)
        
        def a_counter(self,c): 
                if self.a_count == 0:
                        self.a_count += 1
                else:
                        self.a_count += 1

        def b_counter(self,c):
                if self.b_count == 0:
                        self.b_count += 1
                else:
                        self.b_count += 1

        
       
        def get_pwn(self):
                return int(self.pwn)
##        def calculate_rps(self):
##                self.rps_a = self.a_count / 20
##                self.rps_b = self.b_count / 20
##                self.a_count = 0
##                self.b_count = 0
##                return self.rps_a,self.rps_b
##        
 
        def calculate_pwm(self,distance,set_distance):
            
            self.rps_a = self.a_count / 20
            self.rps_b = self.b_count / 20
            self.a_count = 0
            self.b_count = 0


            self.speed_ref += (distance - set_distance) / 100
            print self.speed_ref, distance, set_distance
            
            if (self.speed_ref > self.max_speed):
                    self.speed_ref = self.max_speed
            elif (self.speed_ref < self.min_speed):
                    self.speed_ref = self.min_speed


            self.pwm_a += (((self.speed_ref + self.rps_a) / 2) - self.rps_a) * 5;
            self.pwm_b += (((self.speed_ref + self.rps_b) / 2) - self.rps_b) * 5;

            if (self.pwm_a > self.max_pwm):
                self.pwm_a = self.max_pwm
            if (self.pwm_b > self.max_pwm):
                self.pwm_b = self.max_pwm
            if (self.pwm_a < self.min_pwm):
                self.pwm_a = self.min_pwm;
            if (self.pwm_b < self.min_pwm):
                self.pwm_b = self.min_pwm;

            
         
                    
                  

        def get_pwm(self):
                pwm = str(self.pwm_a) + " " + str(self.pwm_b)
                return str(pwm)

                
###Raspberry Pi Pin
##
##
##speed_ref = 10
##max_speed = 20
##min_speed = 1
##
##a_count = 0
##b_count = 0
##
##max_pwm = 255
##min_pwm = 0
##pwm_a = 0
##pwm_b = 0




        


        
##GPIO.add_event_detect(ir_right, GPIO.RISING, callback = b_counter)
##GPIO.add_event_detect(ir_left, GPIO.RISING, callback = a_counter)
##
##
##
##while True: 
##        t = threading.Timer(1, feedback_control)
##        t.start()
##        t.join()
