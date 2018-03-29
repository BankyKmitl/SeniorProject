import RPi.GPIO as GPIO
import microgear.client as client
import time
import threading
import serial
import math
import subprocess
from bluetooth.ble import BeaconService
import os
from sensor import speed
from sensor import distance
from path import pathmap
import turtle
from mttkinter import mtTkinter as tk
import datetime
import Queue
from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
import command
from collections import deque
##from sensor import Gyro

##############################################################################
def drive_motor(direction,pwm):
        pwm_a = int(pwm.split(" ")[0])/255.0
        pwm_b = int(pwm.split(" ")[1])/255.0
        if direction == 0:
                stop(pwm_a,pwm_b)
        elif direction == 1:
                fwd(pwm_a,pwm_b)
        elif direction == 2:
                bwd(pwm_a,pwm_b)
        elif direction == 3:
                left(pwm_a,pwm_b)
        elif direction == 4:
                right(pwm_a,pwm_b)                

def stop(a,b):
	forwardLeft.value = False
	reverseLeft.value = False
	forwardRight.value = False
	reverseRight.value = False
	driveLeft.value = 0
	driveRight.value = 0
	
def fwd(a,b):
	forwardLeft.value = True
	reverseLeft.value = False
	forwardRight.value = True
	reverseRight.value = False
	driveLeft.value = a
	driveRight.value = b
 
def bwd(a,b):
	forwardLeft.value = False
	reverseLeft.value = True
	forwardRight.value = False
	reverseRight.value = True
	driveLeft.value = a
	driveRight.value = b

def left(a,b):
	forwardLeft.value = False
	reverseLeft.value = False
	forwardRight.value = True
	reverseRight.value = False
	driveLeft.value = a
	driveRight.value = b
 
def right(a,b):
	forwardLeft.value = True
	reverseLeft.value = False
	forwardRight.value = False
	reverseRight.value = False
	driveLeft.value = a
	driveRight.value = b

def check_direction(xy):
        if (xy[0] == 0):
            if (xy[1] == 2):
                return "right"
            elif (xy[1] == 1):
                return "left"
            else:
                return "stop"
        elif (xy[0] == 1):
            if (xy[1] == 0):
                return "fwd"
        elif (xy[0] == 2):
            if (xy[1] == 0):
                return "bwd"
        
def check_direction_tuple(xy):
        if (xy == (0,0)):
                return "stop"
        elif (xy == (1,0)):
                return "fwd"
        elif (xy == (2,0)):
                return "bwd"
        elif (xy == (0,1)):
                return "left"
        elif (xy == (0,2)):
                return "right"

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

def draw_plan_map(data):
        split_data = data.split(";")
        xy = (int(data.split(";")[1][1:2]),int(data.split(";")[1][-2:-1]))
        draw_direction = check_direction(xy)
        draw_distance = int(split_data[2])

        if draw_direction == "fwd":
                leader_turtle.forward(draw_distance)
        elif draw_direction == "bwd":
                leader_turtle.backward(draw_distance)
        elif draw_direction == "left":
                leader_turtle.left(draw_distance)
        elif draw_direction == "right":
                leader_turtle.right(draw_distance)
        
def master_draw(message):
        leader_turtle.color("blue")
        xy = message[0]
        draw_direction = check_direction(xy)
        draw_distance = int(message[1])

        if  draw_direction == "fwd":
                leader_turtle.forward(draw_distance)
        elif draw_direction == "bwd":
                leader_turtle.backward(draw_distance)
        elif draw_direction == "left":
                leader_turtle.left(draw_distance)
        elif draw_direction == "right":
                leader_turtle.right(draw_distance)

def plan_draw(message):
        if len(message) != 0:
            leader_turtle.pencolor("blue")
            xy = message[0]
            draw_direction = check_direction(xy)
            draw_distance = int(message[2])
            if draw_direction == "fwd":
                leader_turtle.forward(draw_distance)
                leader_turtle.dot("blue")
            if draw_direction == "bwd":
                leader_turtle.backward(draw_distance)
            if draw_direction == "left":
                leader_turtle.left(draw_distance)
            if draw_direction == "right":
                leader_turtle.right(draw_distance)

def self_draw(direction,distance):
            actual_turtle.pencolor("red")
            draw_direction = check_direction_tuple(direction)
            draw_distance = distance
            
            if draw_direction == "fwd":
                actual_turtle.forward(draw_distance)
            elif draw_direction == "bwd":
                actual_turtle.backward(draw_distance)
            elif draw_direction == "left":
                actual_turtle.left(cmToDegree(draw_distance))
            elif draw_direction == "right":
                actual_turtle.right(cmToDegree(draw_distance))

def init_self_draw(message):
        if len(message) != 0:
            actual_turtle.pencolor("red")
            xy = message[0]
            draw_direction = check_direction(xy)
            draw_distance = int(message[1])

            actual_turtle.penup()
            actual_turtle.backward(draw_distance)
            actual_turtle.pendown()

def map_to_str(plan_map):
        return "*".join(plan_map)

def str_to_map(message):
        return message.split("*")

##### Leader Mode robot will advertise untill get join from follower
##### then close beacon and publish control data to mac topic
        
def leader(data,des):
    global plan_map,start_drive
    com_iot.subscribe(my_mac)
    beacon_advertise("12345678-1234-1234-1234-"+my_mac,1,1,1,1)
    start_drive = True 
    
#### Follower Mode tobot will scan for Beacon data then join to topic leader
#### and send join to topic and get data from leader
    
def follower(data,des):
    beacon_scan()
    master_topic = deviceData[0][24:]
    com_iot.publish(master_topic,"join") #### Publish join state  to leader topic
    com_iot.subscribe(master_topic)   ####Subscribe Leader
    beacon_advertise("12345678-1234-1234-1234-"+my_mac,1,1,1,1)
    com_iot.subscribe(my_mac) #### Subscribe to own topic
    
############################# Class #####################################
            
class Beacon(object):
    
    def __init__(self, data, address):
        self._uuid = data[0]
        self._major = data[1]
        self._minor = data[2]
        self._power = data[3]
        self._rssi = data[4]
        self._address = address
        
    def __str__(self):
        ret = "Beacon: address:{ADDR} uuid:{UUID} major:{MAJOR}"\
                " minor:{MINOR} txpower:{POWER} rssi:{RSSI}"\
                .format(ADDR=self._address, UUID=self._uuid, MAJOR=self._major,
                        MINOR=self._minor, POWER=self._power, RSSI=self._rssi)
        return ret

def beacon_advertise(uuid, major, minor, tx, interval):
    service.start_advertising(uuid,major, minor, tx, interval)

def beacon_scan():
    print "scan start ..."
    devices = service.scan(5)
    t1 = time.time()
    i = 1
    obj = 0
    while True:
        for address, data in list(devices.items()):
            b = Beacon(data, address)
            print b._uuid
            print deviceData
            if i == 1:
               obj = b
               deviceData.append(b._uuid)
            if obj._rssi > b._rssi:
               deviceData[0] = b._uuid
        if len(deviceData) != 0:
            break;
        
class DistanceSensor(threading.Thread):
        def __init__(self,TRIG,ECHO):
            threading.Thread.__init__(self)
            self.TRIG = TRIG
            self.ECHO = ECHO
            self.distance = 100
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
                self.pwm_a = 0
                self.pwm_b = 0
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
##                print "Motor A : ", hole_a, a_count, a_total

        def b_counter(self,c):
                self.b_total += 1
                self.b_count += 1
                self.hole_b += 1
##                print "Motor B : ", hole_b, b_count, b_total

        def calculate_speed_ref(self,distance):
                    self.rps_a = self.a_count / 20.0
                    self.rps_b = self.b_count / 20.0
                    self.a_count = 0
                    self.b_count = 0

                    self.speed_ref += distance / 100 ## why

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

        def is_goal(self,goal_distance,avg_hole):
                #20 hole = 22 cm
                #1 hole = 1.1 cm
                expect_hole =  goal_distance
                if (avg_hole < expect_hole):
                        return False
                else:
                        return True

        def calculate_pwm(self,ultra_distance,goal_distance = 10000):
            global plan_map, current_direction,previous_direction
            ##print "goal_distance", goal_distance
            ##distance = int(goal_distance)

            if (current_direction == (0,1) or current_direction == (0,2)):
                    turn_distance = degreeToCm(goal_distance)
                    if (not (self.is_goal(turn_distance,self.get_avg_hole()*2))):
                            self.calculate_speed_ref(goal_distance)
                    else:
                            self.hole_a = 0
                            self.hole_b = 0
                            self.pwm_a = 0
                            self.pwm_b = 0                
                            try:
                                plan_map.data.popleft()
                            except IndexError:
                                pass

            elif (current_direction == (1,0) or current_direction == (2,0)):
                    distance = min(int(goal_distance),ultra_distance)
                    if (not (self.is_goal(goal_distance,self.get_avg_hole()))):
                            self.calculate_speed_ref(distance)
                    else:
                            self.hole_a = 0
                            self.hole_b = 0
                            self.pwm_a = 0
                            self.pwm_b = 0                
                            try:
                                plan_map.data.popleft()
                            except IndexError:
                                pass


            return str(int(self.pwm_a)) + " " + str(int(self.pwm_b)) 

        def get_pwm(self):
                pwm = str(self.pwm_a) + " " + str(self.pwm_b)
                return pwm

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

    def callback_message(self,topic,message):
        global data,plan_map,my_seq,platoon_topic
        des = 0
        
        if message[:4] == "plan":
                message = message[4:]
                plan_message = message.split("*")
                plan_map.update(plan_message)
                for item in plan_message:
                        leader_turtle.color("blue")
                        leader_turtle.dot("blue")
                        draw_plan_map(item)
                        
        if message == "join":   ### Wait Message Join for approve when member join topic 
                print "Someone join Topic: ", topic
                ####### to stop Beacon advertise must restart bluetooth adapter
                os.system("sudo service bluetooth stop" )   
                os.system("sudo service bluetooth start" )
                com_iot.publish(my_mac,"platoon"+my_mac)
                com_iot.publish(my_mac,"seq"+my_seq)

                com_iot.unsubscribe(my_mac) ####Unsubscribe own topic to avoid complex data from more topic
                com_iot.publish(my_mac,"plan"+map_to_str(plan_map.getMap()))

        if message[:7] == "platoon":
                platoon_topic = message
                com_iot.subscribe(platoon_topic)

        if message[:3] == "seq":
                my_seq = int(message[3:])+1
                com_iot.publish(platoon_topic,"last"+my_seq)

        if message[:4] == "last":
                last_seq = int(message[4:])
         
        if message[:5] == "Topic":    ######### Swap topic unsub leader topic and sub new leader
                com_iot.unsubscribe(master_topic) 
                com_iot.subscribe(message[5:])

        if message == "Leader":    ######### User Choose Mode Leader 
                data[0] = 1
                my_seq = 1
                
                        
        if message == "Follower":   ######### User Choose Mode Follower
                data[0] = 0
                my_seq = 0
                
        
        if message == "start":    ######### User Start Platooning
                print "go go!"
                if data[0] == 1:
                        leader(data,des)
                        com_iot.subscribe("platoon"+my_mac)
                if data[0] == 0:
                        follower(data,des)
##                        com_iot.unsubscribe("platoon"+my_mac)
        
        if message == "exit":      ######### Leader exit platoons
                com_iot.publish(my_mac,"Topic:"+master_topic)
                com_iot.unsubscribe(master_topic)
                
        else:     ######### Other input from free board will be destination code
                if message in data[1]:
                        des = data[1].index(str(message))
##
##        print "Now Data is ",data
##        print "Destination",des

        #########################################################
##        master_map.update(message)
##        global temp_xy,previous_message
##        xy = (int(message.split(";")[1][1:2]),int(message.split(";")[1][-2:-1]))  
##        self.mas_total_dis = int(message.split(";")[2])
##
##        if (xy != temp_xy):
##                leader_turtle.dot("blue")
##                plan_map.update(message)
##                master_draw([xy,self.mas_total_dis])
##                self.mas_temp_dis = self.mas_total_dis
##        else:
##                master_draw([xy,self.mas_total_dis - self.mas_temp_dis])
##                self.mas_temp_dis = self.mas_total_dis
##
##        temp_xy = current_direction
        #####################################################

    def callback_error(msg):
        print "Error"

    def subscribe(self,topic):
        client.subscribe("/" + topic)

    def connect(self,boo):
        client.connect(boo)

    def unsubscribe(self,topic):
        client.unsubscribe("/"+topic)

    def publish(self,topic, payload):
        client.publish("/"+topic, payload)
        
########################################################################################

class Actual_Map(threading.Thread):
        global leader_state
        def __init__(self):
                threading.Thread.__init__(self)
##                plan_map.update(str(time.time())+";"+"(1,0);0;0;"+str(d.distance))

        def run(self):
                global current_direction,previous_direction,self_map
                previous_distance = 0
                try:
                        while True:

                            ts = time.time()
                            dt = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                            self_map.update([str(dt),str(current_direction),str(s.get_avg_hole()),str(d.distance)])
                            
                            if (self_map.getLen() == 1):
                                previous_distance = 0
                                
                            if (len(self_map.getMap()) > 1) and (current_direction == previous_direction):
                                distance = int(self_map.getMap()[-1][2])
                                self_draw(current_direction,distance - previous_distance)
                                previous_distance = distance

                            elif (len(self_map.getMap()) > 1) and (current_direction != previous_direction):
                                previous_direction = current_direction
                                previous_distance = 0
                                distance = int(self_map.getMap()[-1][2])
                                self_draw(current_direction,distance - previous_distance)
                                previous_distance = distance
                                
                            gui_timestamp.config(text = "Current Time : " + dt)
                            gui_ultra_distance.config(text = "Ultrasonic Sensor : " + str(d.distance) +" cm")
                            gui_total_distance.config(text = "Total Distance : "+str(s.get_total_avg_hole())+" cm")
                            gui_direction.config(text = "Direction : " + str(current_direction))
                                
##                            print self_map.getLast()
                            com_iot.publish(com_iot.alias,self_map.getLast())
                            time.sleep(0.1)
                except Queue.Empty:
                        pass

#########################################################################################
                    
class Drive(threading.Thread):
        def __init__(self):
                threading.Thread.__init__(self)
                
        def run(self):
                global current_direction,previous_direction,plan_map
                while True:
                        if start_drive:
                                if not d.check_distance(d.distance,20):
                                        xy = (0,0)
                                        if (plan_map.getLen() != 0):
                                                xy = (int(plan_map.getFirst().split(";")[1][1:2]),int(plan_map.getFirst().split(";")[1][-2:-1]))
                                                goal_distance = int(plan_map.getFirst().split(";")[2])
                                                current_direction = xy
                                                print current_direction,s.get_avg_hole()
                ##                                print xy, check_direction(xy), parse_direction(check_direction(xy))
        ##                                        print parse_direction(check_direction(xy)),s.calculate_pwm(d.distance,goal_distance)
                                                drive_motor(parse_direction(check_direction(current_direction)),s.calculate_pwm(d.distance,goal_distance)) 
                                        else:
                                                drive_motor(0,"0 0")
                                                current_direction = (0,0)
        ##                                        print parse_direction(check_direction(xy)),s.calculate_pwm(d.distance,goal_distance)
        ##                                        drive_motor(parse_direction(check_direction(current_direction)),s.calculate_pwm(d.distance))
                                else:
                                        drive_motor(0,"0 0")
                                        current_direction = (0,0)
                        else:
                                pass
                        time.sleep(0.01)



                                
######################################################################
                        
leader_state = 1

########################### Bluetooth Service########################
service = BeaconService("hci0")
p = subprocess.Popen(["cat","/sys/class/net/eth0/address"], stdout=subprocess.PIPE)
output, err = p.communicate()  
my_mac = "".join(output.split(":")  )[:12]
deviceData = []
data = [0,["Meanburi","Bangkapi","Ladkrabang"]]

platoon_topic = ""
my_seq = 0
last_seq = 0
########################### Turtle GUI ############################

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

actual_turtle = turtle.RawPen(canvas)
actual_turtle.speed('fastest')
leader_turtle = turtle.RawPen(canvas)
leader_turtle.speed('fastest')



##################### Setup Control Value #########################

# Motor A, Left Side GPIO CONSTANTS
PWM_DRIVE_LEFT = 18		# ENA - H-Bridge enable pin
FORWARD_LEFT_PIN = 24	# IN1 - Forward Drive
REVERSE_LEFT_PIN = 23	# IN2 - Reverse Drive
# Motor B, Right Side GPIO CONSTANTS
PWM_DRIVE_RIGHT = 13		# ENB - H-Bridge enable pin
FORWARD_RIGHT_PIN = 6	# IN1 - Forward Drive
REVERSE_RIGHT_PIN = 5	# IN2 - Reverse Drive

driveLeft = PWMOutputDevice(PWM_DRIVE_LEFT, True, 0, 30.5)
driveRight = PWMOutputDevice(PWM_DRIVE_RIGHT, True, 0, 30.5)

forwardLeft = PWMOutputDevice(FORWARD_LEFT_PIN)
reverseLeft = PWMOutputDevice(REVERSE_LEFT_PIN)
forwardRight = PWMOutputDevice(FORWARD_RIGHT_PIN)
reverseRight = PWMOutputDevice(REVERSE_RIGHT_PIN)

set_distance = 20

previous_direction = (0,0)
current_direction = (0,0)
temp_xy = (0,0)

start_drive = False
######## start Serial
##ser = serial.Serial('/dev/ttyACM0',115200)

######## sensor setting
##g = Gyro.Gyro()
d = DistanceSensor(16,19)
s = SpeedSensor(20,21)
GPIO.setmode(GPIO.BCM)
GPIO.add_event_detect(s.ir_left, GPIO.RISING, callback=s.a_counter)
GPIO.add_event_detect(s.ir_right, GPIO.RISING, callback=s.b_counter)

###############################################################

##d.start()
##d.join
##g.start()
previous_message = ""
#################  IOT setting ################################
com_iot = Iot("O81ngNPLQoe9ppO","BQUAqXZ6wM8eiqQPYLWDnP2G9",'RobotCarPlatoon',my_mac)
com_iot.subscribe("freeboard"+my_mac)
com_iot.connect(False)

plan_map = pathmap.Map()

        
master_map = pathmap.Map()
self_map = pathmap.Map()
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

