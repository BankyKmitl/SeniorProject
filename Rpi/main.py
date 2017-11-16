import RPi.GPIO as GPIO
import time
import threading
import serial
from sensor import speed
from sensor import distance
from communication import iot

###Raspberry Pi Pin


speed_ref = 10
max_speed = 20
min_speed = 1

a_count = 0
b_count = 0

max_pwm = 255
min_pwm = 0
pwm_a = 0
pwm_b = 0

set_distance = 20

direction_flag = 1; #0 = stop, 1 = forward , 2 = backward, 3 =turnleft, 4 = turnright
#start Serial
ser = serial.Serial('/dev/ttyACM3',115200)

#sensor setting
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Object Declare
d = distance.DistanceSensor(16,19)
s = speed.SpeedSensor(20,21)
d.setDistance()
s.setSpeed()
com_iot = iot.Iot('1eXOnAYRB9OIRX3','oF3mfRcijvfaSfBPAjAfOqTS8','RobotCarPlatoon','carA')
com_iot.setDevice()
com_iot.connect(False)

##Start Counting
GPIO.add_event_detect(s.getIr_right(), GPIO.RISING, callback = s.b_counter)
GPIO.add_event_detect(s.getIr_left(), GPIO.RISING, callback = s.a_counter)


    ##Run Sensor
while True:
    tSpeed = threading.Timer(1,s.calculate_pwm,[d.distance,set_distance])
    tDis   = threading.Timer(1,d.runDistance)
    
    direction_flag = com_iot.direction_flag
    ser_message = str(direction_flag) + " " + s.get_pwm()
    com_iot.restart_message()
    ser.write(ser_message)
    
    tDis.start()
    tSpeed.start()
    tDis.join()
    tSpeed.join()
    
    #tCon.start()
    #tCon.join()




