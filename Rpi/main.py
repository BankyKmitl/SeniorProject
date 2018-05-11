import RPi.GPIO as GPIO
import microgear.client as client
import time
import threading
import math
import subprocess
from bluetooth.ble import BeaconService
import os
from sensor import speed
from sensor import distance
from path import pathmap
import datetime
import Queue
from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from collections import deque
from sensor import Gyro
import smbus

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
	reverseLeft.value = True
	forwardRight.value = True
	reverseRight.value = False
	driveLeft.value = a
	driveRight.value = b
 
def right(a,b):
	forwardLeft.value = True
	reverseLeft.value = False
	forwardRight.value = False
	reverseRight.value = True
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
    return (math.radians(int(degree)) * 5)

def cmToDegree(cm):
    return cm * 16.370222718

def map_to_str(plan_map):
        return "*".join(plan_map)

def str_to_map(message):
        return message.split("*")

##### Leader Mode robot will advertise untill get join from follower
##### then close beacon and publish control data to mac topic
        
def leader(data,des):
    global plan_map
    com_iot.subscribe(my_mac)
    beacon_advertise("12345678-1234-1234-1234-"+my_mac,1,1,1,1)
    
#### Follower Mode tobot will scan for Beacon data then join to topic leader
#### and send join to topic and get data from leader
    
def follower(data,des):
    global master_topic
    beacon_scan()
    master_topic = deviceData[0][24:]
    com_iot.publish(master_topic,"join"+my_mac) #### Publish join state  to leader topic
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
        
def check_distance(ultra_distance,set_distance):
        
        if (ultra_distance > set_distance+2):
                return "More"
        elif (ultra_distance < set_distance-2):
                return "Less"
        else:
                return "Equal"  

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
                self.min_pwm = 255
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

##        def get_total_avg_hole(self):
##                return (self.a_total + self.b_total) / 2

        def get_avg_hole(self):
                return (self.hole_a + self.hole_b ) / 2

        def a_counter(self,c):
                self.a_total += 0.5
                self.a_count += 0.5
                self.hole_a += 0.5
##                print "Motor A : ", hole_a, a_count, a_total

        def b_counter(self,c):
                self.b_total += 0.5
                self.b_count += 0.5
                self.hole_b += 0.5
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



##                    self.pwm_a += (((self.speed_ref + self.rps_a) / 2) - self.rps_a) * 5;
                    self.pwm_b += (((self.speed_ref + self.rps_b) / 2) - self.rps_b) * 5
                    self.pwm_a = self.pwm_b - 5
                    print str(int(self.pwm_a)) + " " + str(int(self.pwm_b)) 
                    
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
                if (avg_hole < goal_distance):
                        return False
                else:
                        return True

        def calculate_pwm(self,ultra_distance,goal_distance = 10000):
            global plan_map, current_direction,previous_direction,self_map
            ##print "goal_distance", goal_distance
            ##distance = int(goal_distance)

            if (current_direction == (0,1) or current_direction == (0,2)):
                    turn_distance = degreeToCm(goal_distance)
                    if (not (self.is_goal(turn_distance,self.get_avg_hole()))):
                            self.calculate_speed_ref(goal_distance)
                    else:
                            com_iot.publish(com_iot.alias,"0;"+str(current_direction)+";"+
                                            str(turn_distance)+";"+str(d.get_value())+";")
                            self.hole_a = 0
                            self.hole_b = 0
                            self.pwm_a = 0
                            self.pwm_b = 0
                            current_direction = (0,0)
                            try:
                                plan_map.data.popleft()
                            except IndexError:
                                pass

            elif (current_direction == (1,0) or current_direction == (2,0)):
                    distance = min(int(goal_distance),ultra_distance)
                    if (not (self.is_goal(goal_distance,self.get_avg_hole()))):
                            self.calculate_speed_ref(distance)
                    else:
                            com_iot.publish(com_iot.alias,"0;"+str(current_direction)+";"+
                                            str(goal_distance)+";"+str(d.get_value())+";")
                            self.hole_a = 0
                            self.hole_b = 0
                            self.pwm_a = 0
                            self.pwm_b = 0
                            current_direction = (0,0)
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
        global data,plan_map,my_seq,platoon_topic,start_drive,master_topic,com_iot
        des = 0

        if message[:3] == "com":
                slice_message = message[3:]
                plan_message = slice_message.split("*")
                plan_map.extend(plan_message)
                com_iot.publish("turtle"+my_mac,"plan"+slice_message)
                
        if message[:4] == "plan" and len(message) > 4:
                message = message[4:]
                plan_message = message.split("*")
                plan_map.extend(plan_message)
                plan_map.extendleft(["0;(1,0);"+str(d.get_value()+24)+";0"])
                com_iot.publish("turtle"+my_mac,"plan"+map_to_str(plan_map.getMap()))
                        
        elif message[:4] == "join":   ### Wait Message Join for approve when member join topic 
                print "Someone join Topic: ", topic
                ####### to stop Beacon advertise must restart bluetooth adapter
                os.system("sudo service bluetooth stop" )   
                os.system("sudo service bluetooth start" )
                com_iot.publish(my_mac,"platoon"+my_mac)
##                com_iot.publish(my_mac,"seq"+str(my_seq))
                com_iot.unsubscribe("RobotCarPlatoon/"+my_mac) ####Unsubscribe own topic to avoid complex data from more topic
                com_iot.publish(message[4:],"plan"+map_to_str(plan_map.getMap()))

        elif message[:7] == "platoon":
                platoon_topic = message
                com_iot.subscribe(platoon_topic)
##
##        elif message[:3] == "seq":
##                my_seq = int(message[3:])+1
##                com_iot.publish(platoon_topic,"last"+str(my_seq))
##

##        elif message[:4] == "last":
##                last_seq = int(message[4:])

        elif message == "launch":
                start_drive = True
                
        elif message[:5] == "Topic":    ######### Swap topic unsub leader topic and sub new leader
                com_iot.unsubscribe(master_topic) 
                com_iot.subscribe(message[5:])

        elif message == "Leader":    ######### User Choose Mode Leader 
                data[0] = 1
                my_seq = 1
                
        elif message == "Follower":   ######### User Choose Mode Follower
                data[0] = 0
                my_seq = 0

        elif message == "start":    ######### User Start Platooning
                print "Set Mode"
                com_iot.publish("turtle"+my_mac,data[0])
                if data[0] == 1:
                        leader(data,des)
                        com_iot.subscribe("platoon"+my_mac)
                if data[0] == 0:
                        follower(data,des)
##                        com_iot.unsubscribe("platoon"+my_mac)
        
        elif message == "exit":      ######### Leader exit platoons
                com_iot.publish(my_mac,"Topic:"+master_topic)
                com_iot.unsubscribe(master_topic)
                
        else:
                ######### Other input from free board will be destination code
                if (topic == ("/RobotCarPlatoon/"+master_topic)):
                        com_iot.publish("robotcarplatoons1",message)
                
    def callback_error(msg):
        print "Error"

    def chat(self,topic,message):
        client.chat("/"+topic,message)

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
        def __init__(self):
                threading.Thread.__init__(self)

        def run(self):
                global current_direction,previous_direction,self_map
                try:
                        while True:
                            ultra_distance = d.get_value()
                            if start_drive:
                                if (check_distance(int(ultra_distance),20) == "More"):
                                        xy = (0,0)
                                        if (plan_map.getLen() != 0):
                                                xy = (int(plan_map.getFirst().split(";")[1][1:2]),int(plan_map.getFirst().split(";")[1][-2:-1]))
                                                goal_distance = int(plan_map.getFirst().split(";")[2])
                                                current_direction = xy
                                                print s.rps_a, s.rps_b,s.get_avg_hole()
                                                drive_motor(parse_direction(check_direction(current_direction)),s.calculate_pwm(ultra_distance,goal_distance)) 
                                        else:
                                                drive_motor(0,"0 0")
                                                current_direction = (0,0)
                                                
                                elif (check_distance(int(ultra_distance),20) == "Less"):
                                        drive_motor(2,"100 100")
                                        current_direction = (2,0)
                                else:
                                        drive_motor(0,"0 0")
                                        current_direction = (0,0)
                            else:
                                pass
                            if (current_direction != previous_direction):
                                        s.hole_a = 0
                                        s.hole_b = 0
                                        s.pwm_a = 0
                                        s.pwm_b = 0
                            previous_direction = current_direction
##                            print current_direction,s.get_avg_hole()
                            ax,ay,az,gx,gy,gz = g.get_str_value()
                            ts = time.time()
                            dt = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                            self_map.append([str(dt),str(current_direction),str(s.get_avg_hole()),str(ultra_distance)
                                             ,str(ax),str(ay),str(az),str(gx),str(gy),str(gz)])
                            com_iot.publish(com_iot.alias,self_map.getLast())
                            time.sleep(0.5)
                except Queue.Empty:
                        pass

########################### Bluetooth Service########################
service = BeaconService("hci0")
p = subprocess.Popen(["cat","/sys/class/net/eth0/address"], stdout=subprocess.PIPE)
output, err = p.communicate()  
my_mac = "".join(output.split(":")  )[:12]
deviceData = []
data = [0,["Meanburi","Bangkapi","Ladkrabang"]]

master_topic = ""
platoon_topic = ""
my_seq = 0
last_seq = 0

##################### Setup Control Value #########################

# Motor A, Left Side GPIO CONSTANTS
PWM_DRIVE_LEFT = 18		# ENA - H-Bridge enable pin
FORWARD_LEFT_PIN = 24	# IN1 - Forward Drive
REVERSE_LEFT_PIN = 23	# IN2 - Reverse Drive
# Motor B, Right Side GPIO CONSTANTS
PWM_DRIVE_RIGHT = 13		# ENB - H-Bridge enable pin
FORWARD_RIGHT_PIN = 6	# IN1 - Forward Drive
REVERSE_RIGHT_PIN = 5	# IN2 - Reverse Drive

driveLeft = PWMOutputDevice(PWM_DRIVE_LEFT, True, 0, 30)
driveRight = PWMOutputDevice(PWM_DRIVE_RIGHT, True, 0, 30)

forwardLeft = PWMOutputDevice(FORWARD_LEFT_PIN)
reverseLeft = PWMOutputDevice(REVERSE_LEFT_PIN)
forwardRight = PWMOutputDevice(FORWARD_RIGHT_PIN)
reverseRight = PWMOutputDevice(REVERSE_RIGHT_PIN)

set_distance = 20

previous_direction = (0,0)
current_direction = (0,0)
start_drive = False

######## sensor setting
d = distance.DistanceSensor(16,19)
g = Gyro.Gyro()
s = SpeedSensor(20,21)
GPIO.setmode(GPIO.BCM)
GPIO.add_event_detect(s.ir_left, GPIO.RISING, callback=s.a_counter)
GPIO.add_event_detect(s.ir_right, GPIO.RISING, callback=s.b_counter)
#################  IOT setting ################################
com_iot = Iot("GsZwSMTN7bgp7Bv","mnA4mIRADYdHSAeHlxsOMO4Pn","RobotCarPlatoon",my_mac)
com_iot.subscribe("freeboard"+my_mac)
com_iot.connect(False)

plan_map = pathmap.Map()
master_map = pathmap.Map()
self_map = pathmap.Map()

ac_map_thread = Actual_Map()
ac_map_thread.start()



