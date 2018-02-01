import microgear.client as client
import logging
import time
import serial
from path import pathmap
import math

class Iot:

    def __init__(self, gearkey, gearsecret, appid, aliasName):
        self.gearkey = gearkey
        self.gearsecret = gearsecret
        self.appid = appid
        self.alias = aliasName
        self.direction = ()
        self.goal_distance = 0
        self.message = []
        
    def degreeToCm(self,degree):
        return (math.radians(int(degree)) * 3.5)

    def check_direction(self,xy):
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
        
    def callback_connect(self):
        print ("Now I am connected with netpie")

    def restart_message(self):
        self.message = []
        self.direction = (0,0)
        self.goal_distance = 0
        
    def callback_message(self,topic,message):
        xy = (message.split(",")[0],message.split(",")[1])
        
        if (self.check_direction(xy) == "right" or self.check_direction(xy) == "left"):
            self.message = message.split(",")
            self.direction = xy
            self.goal_distance = self.degreeToCm(message.split(",")[2])
        else:
            self.message = message.split(",")
            self.direction = xy
            self.goal_distance = message.split(",")[2] 

    def callback_error(msg):
        print "Error"

    def setDevice(self):
        client.create(self.gearkey,self.gearsecret,self.appid,{'debugmode':True})
        client.setalias(self.alias)
        client.on_message = self.callback_message
        client.on_error = self.callback_error
        client.on_connect = self.callback_connect 

    def connect(self,boo):
        client.connect(boo)

    def subscribe(self,topic):
        client.subscribe("/" + topic)
        
    def unsubsribe(self,topic):
        client.unsubscribe("/"+topic)
        
    def publish(self,topic, payload):
        client.publish("/"+topic, payload)


    
