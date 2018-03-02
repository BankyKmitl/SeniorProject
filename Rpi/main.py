import RPi.GPIO as GPIO
import microgear.client as client
import time
import threading
import serial
import math
from sensor import speed
from sensor import distance
from path import pathmap
import turtle
from mttkinter import mtTkinter as tk
import datetime
import Queue

##############################################################################

def check_direction(xy):
        if (xy[0] == 0):
            if (xy[1] == 1):
                return "right"
            elif (xy[1] == -1):
                return "left"
            else:
                return "stop"
        elif (xy[0] == 1):
            if (xy[1] == 0):
                return "fwd"
        elif (xy[0] == -1):
            if (xy[1] == 0):
                return "bwd"


def parse_direction(str_direction):
        if (str_direction == "fwd"):
                return 1
        elif (str_direction == "bwd"):
                return 2
        elif (str_direction == "left"):
                return 3
        elif (str_direction == "right"):
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
    ser_message = str(direction) + " " + pwm + ';'
    ser.write(ser_message)
    previous_direction = direction

def master_draw(message):
        bob.color("blue")
        xy = message[0]
        draw_direction = check_direction(xy)
        draw_distance = int(message[1])

        if  draw_direction == "fwd":
                bob.forward(draw_distance)
        elif draw_direction == "bwd":
                bob.backward(draw_distance)
        elif draw_direction == "left":
                bob.left(cmToDegree(draw_distance))
        elif draw_direction == "right":
                bob.right(cmToDegree(draw_distance))

def plan_draw(message):
        if len(message) != 0:
            bob.pencolor("blue")
            xy = message[0]
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
            xy = message[0]
            draw_direction = check_direction(xy)
            draw_distance = int(message[1])
            
            print xy,check_direction(xy), draw_direction , draw_distance
            
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
            xy = message[0]
            draw_direction = check_direction(xy)
            draw_distance = int(message[1])

            alice.penup()
            alice.backward(draw_distance)
            alice.pendown()


class DistanceSensor(threading.Thread):
        def __init__(self,TRIG,ECHO):
            threading.Thread.__init__(self)
            self.TRIG = TRIG
            self.ECHO = ECHO
            self.distance = 0
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.TRIG,GPIO.OUT)
            GPIO.setup(self.ECHO,GPIO.IN)
            GPIO.setwarnings(False)

        def run(self):
            while (True):
              GPIO.output(self.TRIG, True)
              time.sleep(0.00001)
              GPIO.output(self.TRIG, False)
              while GPIO.input(self.ECHO)==0:
                pulse_start = time.time()

              while GPIO.input(self.ECHO)==1:
                pulse_end = time.time()

              pulse_duration = pulse_end - pulse_start
              self.distance = int(pulse_duration * 17150)
              if (self.distance > 400):
                      self.distance = 400
              time.sleep(1)

        def check_distance(self,ultra_distance,set_distance):
              if (int(ultra_distance) in [set_distance-1,set_distance,set_distance+1]):
                return True
              else:
                return False

##############################################################################

class SpeedSensor:
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
                GPIO.setup(self.ir_left,GPIO.IN)
                GPIO.setup(self.ir_right,GPIO.IN)

        def get_total_avg_hole(self):
                return (self.a_total + self.b_total) / 2

        def get_avg_hole(self):
                return (self.hole_a + self.hole_b ) / 2

        def a_counter(self,c):
                self.a_total += 1
                self.a_count += 1
                self.hole_a += 1

        def b_counter(self,c):
                self.b_total += 1
                self.b_count += 1
                self.hole_b += 1

        def is_goal(self,goal_distance,avg_hole):
                #20 hole = 22 cm
                #1 hole = 1.1 cm
                expect_hole =  goal_distance
                if (avg_hole < expect_hole):
                        return False
                else:
                        return True

        def run(self):
                while (True):
                        self.calculate_pwm(int(plan_map.getFirst()[2])-s.get_avg_hole(),d.distance)
                        time.sleep(0.5)

        def calculate_pwm(self,ultra_distance,goal_distance = 10000):
            global plan_map
            ##print "goal_distance", goal_distance
            distance = min(int(goal_distance),ultra_distance)
            ##distance = int(goal_distance)

            if not (self.is_goal(distance,self.get_avg_hole())):
                    self.rps_a = self.a_count / 20.0
                    self.rps_b = self.b_count / 20.0
                    self.a_count = 0
                    self.b_count = 0

                    self.speed_ref += (ultra_distance - distance) / 100 ## why

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
                        self.pwm_a = self.min_pwm
                    if (self.pwm_b < self.min_pwm):
                        self.pwm_b = self.min_pwm
                    
            else:
                        pwm_a = 0
                        pwm_b = 0
                        try:
                                plan_map.popLeft()
                        except IndexError:
                                pass

            return str(int(self.pwm_a)) + " " + str(int(self.pwm_b)) 

        def get_pwm(self):
                pwm = str(self.pwm_a) + " " + str(self.pwm_b)
                return pwm

##################################################################################

################################# IOT Class ###################################

class Iot:
    def __init__(self, gearkey, gearsecret, appid, aliasName):
        client.gearkey = gearkey
        client.gearsecret = gearsecret
        client.appid = appid
        self.gearkey = gearkey
        self.gearsecret = gearsecret
        self.appid = appid
        self.alias = aliasName
        self.direction = ()
        self.goal_distance = 0
        self.message = []
        self.mas_total_dis = 0
        self.mas_temp_dis = 0
        client.create(gearkey,gearsecret,appid,{'debugmode':True})
        client.setalias(self.alias)
        client.on_message = self.callback_message
        client.on_error = self.callback_error
        client.on_connect = self.callback_connect

    def callback_connect(self):
        print ("Now I am connected with netpie")

    def restart_message(self):
        self.message = []
        self.direction = (0,0)
        self.goal_distance = 0

    def callback_message(self,topic,message):
        try:
                master_map.update(message)
                global temp_xy,previous_message
                xy = (int(message.split(";")[1][1]),int(message.split(";")[1][-2]))  
                self.mas_total_dis = int(message.split(";")[2])

                
                if (xy != temp_xy):
                
                        bob.dot("blue")
                        plan_map.update(previous_message)
                        plan_map.update(message+";"+str(d.distance))
                        master_draw([xy,self.mas_total_dis])
                        self.mas_temp_dis = self.mas_total_dis
                else:
                        master_draw([xy,self.mas_total_dis - self.mas_temp_dis])
                        self.mas_temp_dis = self.mas_total_dis

                

                temp_xy = xy
##                if (check_direction(xy) == "right" or check_direction(xy) == "left"):
##                    self.message = message.split(",")
##                    self.direction = xy
##                    self.goal_distance = degreeToCm(message.split(",")[3][2:-1])
##                else:
##                    self.message = message.split(",")
##                    self.direction = xy
##                    self.goal_distance = message.split(",")[3][2:-1]
                previous_message = message
        except Queue.Empty:
                pass

    def callback_error(msg):
        print "Error"

    def subscribe(self,topic):
        client.subscribe("/" + topic)

    def connect(self,boo):
        client.connect(boo)

    def unsubsrcibe(self,topic):
        client.unsubscribe("/"+topic)

    def publish(self,topic, payload):
        client.publish("/"+topic, payload)

class Actual_Map(threading.Thread):
        def __init__(self):
                threading.Thread.__init__(self)
                plan_map.update(str(time.time())+";"+"(1,0);0;0;"+str(d.distance))

        def run(self):
                while True:
                    ts = time.time()
                    dt = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')

                    self_map.update([str(dt),str(current_direction),str(s.get_total_avg_hole()),str(d.distance)]);
                    
                    if (self_map.getLen() > 1):
                        self_draw([current_direction,
                            (float(self_map.getMap()[-1][2]) - float(self_map.getMap()[-2][2]))])
                    else:
                            init_self_draw([current_direction,
                                       d.distance])
                    
                    gui_timestamp.config(text = "Current Time : " + dt)
                    gui_ultra_distance.config(text = "Ultrasonic Sensor : " + str(d.distance) +" cm")
                    gui_total_distance.config(text = "Total Distance : "+str(s.get_total_avg_hole())+" cm")
                    gui_direction.config(text = "Direction : " + str(current_direction))

                    com_iot.publish(com_iot.alias,self_map.getLast())
                    time.sleep(1)
                    
class Drive(threading.Thread):
        def __init__(self):
                threading.Thread.__init__(self)
                
        def run(self):
                global current_direction
                while True:
                        if not d.check_distance(d.distance,20):
                                if (plan_map.getLen() != 0):
                                        xy = (int(plan_map.getFirst().split(";")[1][1:2]),int(plan_map.getFirst().split(";")[1][-2:-1]))
                                        goal_distance = int(plan_map.getFirst().split(";")[2])
        ##                                print xy, check_direction(xy), parse_direction(check_direction(xy))
                                        print parse_direction(check_direction(xy)),s.calculate_pwm(d.distance,goal_distance)
                                        send_serial(parse_direction(check_direction(xy)),s.calculate_pwm(d.distance,goal_distance)) 
                                        current_direction = xy
                                else:
                                        print parse_direction(check_direction(xy)),s.calculate_pwm(d.distance,goal_distance)
                                        send_serial(parse_direction(check_direction(current_direction)),s.calculate_pwm(d.distance))
                        else:
                                send_serial(0,"0 0")
                                current_direction = (0,0)  
                        time.sleep(1)
                                
######################################################################



###########################Turtle GUI ############################

root = tk.Tk()

frame = tk.Frame(bg='black')
gui_ultra_distance = tk.Label(frame, text="",fg='white',bg="black")
gui_total_distance = tk.Label(frame, text="",fg='white',bg="black")
gui_timestamp = tk.Label(frame, text="",fg='white',bg="black")
gui_direction = tk.Label(frame, text="",fg='white',bg="black")

gui_timestamp.pack()
gui_direction.pack()
gui_total_distance.pack()
gui_ultra_distance.pack()

canvas = tk.Canvas(frame, width=750, height=750)
canvas.pack()
frame.pack(fill='both', expand=True)

bob = turtle.RawPen(canvas)
alice = turtle.RawPen(canvas)

root.deiconify()

####################################################################

##################### Setup Control Value #########################

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

######## start Serial
ser = serial.Serial('/dev/ttyACM0',115200)

######## sensor setting
d = DistanceSensor(16,19)
s = SpeedSensor(13,21)
GPIO.setmode(GPIO.BCM)
GPIO.add_event_detect(s.ir_left, GPIO.RISING, callback=s.a_counter)
GPIO.add_event_detect(s.ir_right, GPIO.RISING, callback=s.b_counter)

plan_map = pathmap.Map()
master_map = pathmap.Map()
self_map = pathmap.Map()

###############################################################

d.start()
d.join
temp_xy = (0,0)
previous_message = ""
#################  IOT setting ################################

com_iot = Iot("O81ngNPLQoe9ppO","BQUAqXZ6wM8eiqQPYLWDnP2G9",'RobotCarPlatoon','car01')
com_iot.subscribe("car00")
com_iot.connect(False)

ac_map_thread = Actual_Map()
ac_map_thread.start()

drive_thread = Drive()
drive_thread.start()

##    if (plan_map.getLen() != 0):
##        if(d.check_distance(d.distance,set_distance) and current_direction == ('1','0')):
##            stopCar(s)
##        elif (d.distance < set_distance):
##            current_direction = (-1,0)
##        else:
##            current_direction = (plan_map.getFirst()[0],plan_map.getFirst()[1])
##
##            if (check_direction(current_direction) == "left"):
##                if (s.is_goal(degreeToCm(plan_map.getFirst()[2]),s.hole_b)):
##                    plan_map.pop_left()
##            elif (check_direction(current_direction) == "right"):
##                if (s.is_goal(degreeToCm(plan_map.getFirst()[2]),s.hole_a)):
##                    plan_map.pop_left()
##            else:
##                if (s.is_goal((float(plan_map.getFirst()[2])),s.get_avg_hole())):
##                    plan_map.pop_left()
##    else:
##        stopCar(s)
root.mainloop()

