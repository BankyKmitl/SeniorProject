import RPi.GPIO as GPIO
import time
import threading
import serial
import math
from sensor import speed
from sensor import distance
from communication import iot
from path import pathmap
import turtle
import Tkinter
import datetime

def check_direction(xy):
        if (xy[0] == '0'):
            if (xy[1] == '1'):
                return "right"
            elif (xy[1] == '-1'):
                return "left"
            else:
                return "stop"
        elif (xy[0] == '1'):
            if (xy[1] == '0'):
                return "fwd"
        elif (xy[0] == '-1'):
            if (xy[1] == '0'):
                return "bwd"
        
def parse_direction(str_direction):
        if (str_direction == 'fwd'):
                return 1
        elif (str_direction == 'bwd'):
                return 2
        elif (str_direction == 'left'):
                return 3
        elif (str_direction == 'right'):
                return 4
        else:
                return 0

def degreeToCm(degree):
    return (math.radians(int(degree)) * 3.5)

def cmToDegree(cm):
    return cm * 16.370222718

def stopCar(s):
        global current_direction
        current_direction = (0,0)
        s.pwm_a = 0
        s.pwm_b = 0
        s.hole_a = 0
        s.hole_b = 0
        s.speed_ref = 0
    
def send_serial(direction,pwm):
##    print direction,pwm
    ser_message = str(direction) + " " + pwm
    ser.write(ser_message)
    previous_direction = direction

def plan_draw(message):
        if len(message) != 0:
            bob.pencolor("blue")
            xy = (message[0],message[1])
            draw_direction = check_direction(xy)
            draw_distance = int(message[2])
            if draw_direction == "fwd":
                bob.forward(draw_distance)
                bob.dot("blue")
            if draw_direction == "bwd":
                bob.backward(draw_distance)
            if draw_direction == "left":
                bob.left(draw_distance)
            if draw_direction == "right":
                bob.right(draw_distance)

def self_draw(message):
        if len(message) != 0:
            alice.pencolor("red")
            xy = (message[0][0],message[0][1])
            draw_direction = check_direction(xy)
            draw_distance = int(message[1])
            
            if draw_direction == "fwd":
                alice.forward(draw_distance)
            if draw_direction == "bwd":
                alice.backward(draw_distance)
            if draw_direction == "left":
                alice.left(cmToDegree(draw_distance))
            if draw_direction == "right":
                alice.right(cmToDegree(draw_distance))
                
def init_self_draw(message):
        if len(message) != 0:
            alice.pencolor("red")
            xy = (message[0][0],message[0][1])
            draw_direction = check_direction(xy)
            draw_distance = int(message[1])

            alice.penup()
            alice.backward(draw_distance)
            alice.pendown()



            
d = distance.DistanceSensor(16,19)
s = speed.SpeedSensor(13,21)

root = Tkinter.Tk()

frame = Tkinter.Frame(bg='black')
gui_ultra_distance = Tkinter.Label(frame, text="",fg='white',bg="black")
gui_total_distance = Tkinter.Label(frame, text="",fg='white',bg="black")
gui_timestamp = Tkinter.Label(frame, text="",fg='white',bg="black")
gui_direction = Tkinter.Label(frame, text="",fg='white',bg="black")

gui_timestamp.pack()
gui_direction.pack()
gui_total_distance.pack()
gui_ultra_distance.pack()

canvas = Tkinter.Canvas(frame, width=750, height=750)
canvas.pack()
frame.pack(fill='both', expand=True)

bob = turtle.RawPen(canvas)
alice = turtle.RawPen(canvas)

root.deiconify()
    
speed_ref = 10
max_speed = 20
min_speed = 1

a_count = 0
b_count = 0

max_pwm = 255
min_pwm = 30
pwm_a = 0
pwm_b = 0

set_distance = 20

previous_direction = (0,0)
current_direction = (0,0)
#start Serial
ser = serial.Serial('/dev/ttyACM0',2000000)

#sensor setting
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Object Declare

d.setupGPIO() 
s.setupGPIO()
GPIO.add_event_detect(s.getIr_left(), GPIO.RISING, callback=s.a_counter)
GPIO.add_event_detect(s.getIr_right(), GPIO.RISING, callback=s.b_counter)

plan_map = pathmap.Map()
self_map = pathmap.Map()

com_iot = iot.Iot('1eXOnAYRB9OIRX3','oF3mfRcijvfaSfBPAjAfOqTS8','RobotCarPlatoon','carB')
com_iot.setDevice()
com_iot.connect(False)
com_iot.subscribe("com1")
com_iot.subscribe("carA")

while True:
    tDis = threading.Timer(0.1,d.runDistance)
    tSpeed = threading.Timer(0.05,s.calculate_pwm,[plan_map.getFirst()[2],d.distance])
    tDis.start()
    tDis.join()
    tSpeed.start()
    tSpeed.join()
##
##    print "Left Motor: ",s.rps_a
##    print "Right Motor: ",s.rps_b
    plan_draw(com_iot.message)
    plan_map.update(com_iot.message) 

##    print "plan : ",plan_map.getMap()
##    print "avg_hole :", s.get_avg_hole()
##    print "total_hole :",s.get_total_avg_hole()
##    print "pwm :", s.get_pwm()
##    print "total_distance : ",s.get_total_avg_hole()
    if (plan_map.get_len() != 0):
        if(d.check_distance(d.get_distance(),set_distance) and current_direction == ('1','0')):
            stopCar(s)
        elif (d.distance < set_distance):
            current_direction = ('-1','0')
        else:
            current_direction = (plan_map.getFirst()[0],plan_map.getFirst()[1])

            if (check_direction(current_direction) == "left"):
                if (s.is_goal(degreeToCm(plan_map.getFirst()[2]),s.hole_b)):
                    plan_map.pop_left()
            elif (check_direction(current_direction) == "right"):
                if (s.is_goal(degreeToCm(plan_map.getFirst()[2]),s.hole_a)):
                    plan_map.pop_left()
            else:
                if (s.is_goal((float(plan_map.getFirst()[2])),s.get_avg_hole())):            
                    plan_map.pop_left()
    else:
        stopCar(s)

        
    ts = time.time()
    dt = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    
    self_map.update([str(ts),str(current_direction),str(s.get_total_avg_hole()),str(d.get_distance())]);

    if (self_map.get_len() > 1):
        self_draw([current_direction,
            (float(self_map.getMap()[-1][2]) - float(self_map.getMap()[-2][2]))])
    else:
            print "Start"
            init_self_draw([current_direction,
                       d.distance])
            
    print self_map.getMap()
    gui_timestamp.config(text = "Current Time : " + dt)
    gui_ultra_distance.config(text = "Ultrasonic Sensor : " + str(d.get_distance()) +" cm")
    gui_total_distance.config(text = "Total Distance : "+str( s.get_total_avg_hole())+" cm")
    gui_direction.config(text = "Direction : " + str(current_direction))
    root.update()

  
    if (current_direction != previous_direction):
          com_iot.publish(com_iot.alias,self_map.getLast())
          previous_direction = current_direction

    send_serial(parse_direction(check_direction(current_direction)),s.get_pwm())
    com_iot.restart_message()

##    print "Finish loop"
    

