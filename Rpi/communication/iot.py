import microgear.client as client
import logging
import time
import serial

class Iot:
    
    def __init__(self, gearkey, gearsecret, appid, aliasName):
        self.gearkey = gearkey
        self.gearsecret = gearsecret
        self.appid = appid
        self.alias = aliasName
        self.direction_flag = 0
    
        
    def callback_connect(self):
        print ("Now I am connected with netpie")

    def restart_message(self):
        self.direction_flag = 0
        
    def callback_message(self,topic,message):
        self.direction_flag = message


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

    def subscribe(topic) :
        client.subscribe("/" + topic)

    def publish(topic, payload) :
        client.publish("/"+topic, payload)

    
