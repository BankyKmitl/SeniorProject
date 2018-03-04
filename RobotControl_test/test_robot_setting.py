import microgear.client as microgear
import logging
import time

appid = "RobotCarPlatoon"
gearkey = "cxTnjFAuwED7L7P"
gearsecret =  "wvVgcX12vfZLKqDVvGtkibxWx"
data = [0,["Meanburi","Bangkapi","Ladkrabang"]]
microgear.create(gearkey,gearsecret,appid,{'debugmode': True})

def connection():
    logging.info("Now I am connected with netpie")

def subscription(topic,message):
    des = 0
    print type(message)
    print message
    if str(message) == "Leader":
        data[0] = 1
    if str(message) == "Follower":
        data[0] = 0
    if str(message) == "start":
        print "go go!"
    else:
        if message in data[1]:
            des = data[1].index(str(message))

    print "Now Data is ",data
    print "Destination",des

def disconnect():
    logging.info("disconnected")

microgear.setalias("doraemon")
microgear.on_connect = connection
microgear.on_message = subscription
microgear.on_disconnect = disconnect
microgear.subscribe("/platoons")
microgear.connect(True)
