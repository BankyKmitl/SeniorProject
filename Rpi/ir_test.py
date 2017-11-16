import RPi.GPIO as GPIO
import time
import threading
import serial
from sensor import distance 

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Raspberry Pi Pin
ir_left = 20
ir_right = 21

speed_ref = 10
max_speed = 20
min_speed = 1

a_count = 0
b_count = 0

max_pwm = 255
min_pwm = 0
pwm_a = 0
pwm_b = 0

# GPIO Setup
GPIO.setup(ir_left,GPIO.IN)
GPIO.setup(ir_right,GPIO.IN)

d = distance.DistanceSensor(16,19)
d.setDistance
#Serial Setup
ser = serial.Serial('/dev/ttyACM0',115200)
def a_counter(c):
        global a_count 
        if a_count == 0:
                a_count += 1
        else:
                a_count += 1

def b_counter(c):
        global b_count
        if b_count == 0:
                b_count += 1
        else:
                b_count += 1

def feedback_control():
        calculate_pwm(calculate_rps())

        
def calculate_rps():
        global a_count
        global b_count
        rps_a = a_count / 20
        rps_b = b_count / 20
        a_count = 0
        b_count = 0
        return rps_a,rps_b

def calculate_pwm(rps):
        rps_a, rps_b = rps
        global speed_ref,max_speed,min_speed,pwm_a,pwm_b
        if (speed_ref > max_speed):
                speed_ref = max_speed
        elif (speed_ref < min_speed):
                speed_ref = min_speed
        
        pwm_a += (((speed_ref + rps_a) / 2) - rps_a) * 5;
        pwm_b += (((speed_ref + rps_b) / 2) - rps_b) * 5;

        if (pwm_a > max_pwm):
            pwm_a = max_pwm
        if (pwm_b > max_pwm):
            pwm_b = max_pwm
        if (pwm_a < min_pwm):
            pwm_a = min_pwm;
        if (pwm_b < min_pwm):
            pwm_b = min_pwm;
            
        pwm = str(pwm_a) + " " + str(pwm_b)
        print pwm
        ser.write(pwm)

        
GPIO.add_event_detect(ir_right, GPIO.RISING, callback = b_counter)
GPIO.add_event_detect(ir_left, GPIO.RISING, callback = a_counter)



while True: 
        t = threading.Timer(1, feedback_control)
        t.start()
        t.join()
