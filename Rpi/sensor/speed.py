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
                self.pwm_a = 0.0
                self.pwm_b = 0.0
                self.max_pwm = 255
                self.min_pwm = 50
                self.speed_ref = 0
                self.max_speed = 20
                self.min_speed = 1
                self.rps_a = 0.0
                self.rps_b = 0.0
                self.hole_a = 0
                self.hole_b = 0
                self.avg_hole = 0
                self.expect_hole = 0
                self.a_total = 0
                self.b_total = 0
                
        def getIr_left(self):
                return int(self.ir_left)
        
        def getIr_right(self):
                return int(self.ir_right)

        def get_total_avg_hole(self):
                return (self.a_total + self.b_total) / 2
        
        def get_avg_hole(self):

                return (self.hole_a + self.hole_b ) / 2
        
        def setupGPIO(self):
                GPIO.setwarnings(False)
                # GPIO Setup
                GPIO.setup(self.getIr_left(),GPIO.IN)
                GPIO.setup(self.getIr_right(),GPIO.IN)
        
        def a_counter(self,c):
                self.a_total += 1
                self.a_count += 1
                self.hole_a += 1

        def b_counter(self,c):
                self.b_total += 1
                self.b_count += 1
                self.hole_b += 1

        def get_speed_ref(self):
                return self.speed_ref

        def is_goal(self,goal_distance,avg_hole):
                #20 hole = 22 cm        
                #1 hole = 1.1 cm
                expect_hole =  goal_distance

##                print "expect_hole",expect_hole
                if (avg_hole < expect_hole):
                        return False
                else:
                        return True

        def calculate_pwm(self,goal_distance,ultra_distance):

##            print "goal_distance", goal_distance
            distance = min(int(goal_distance),ultra_distance)
##            distance = int(goal_distance)

            if not (self.is_goal(distance,self.get_avg_hole())):
                    self.rps_a = self.a_count / 20.0
                    self.rps_b = self.b_count / 20.0
                    self.a_count = 0
                    self.b_count = 0
        

                    self.speed_ref += (ultra_distance - distance) / 100 ## why
##                    print "speed referene: ",self.speed_ref
                    
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

            else:
                        pwm_a = 0
                        pwm_b = 0

        def get_pwm(self):
                if(self.pwm_a == 0 and self.pwm_b == 0):
                        pwm = str(self.pwm_a) + " " + str(self.pwm_b)
                        return pwm
                else:
                        pwm = str(self.pwm_a) + " " + str(self.pwm_b)
                        return pwm
