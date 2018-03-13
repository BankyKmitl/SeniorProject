import microgear.client as microgear
import logging
import time
from bluetooth.ble import BeaconService
import thread
import subprocess
import os


##################### Beacon Setting #############################
service = BeaconService("hci0")
devices = service.scan(5)
p = subprocess.Popen(["cat","/sys/class/net/eth0/address"], stdout=subprocess.PIPE)
output, err = p.communicate()

############################# Variable     ######################
my_mac = output.split(":")      
my_mac = "".join(my_mac)[:12]   #### MAC Address
deviceData = []        ##### list of UUID
leader_topic = ""      ##### Topic of Leader


data = [0,["Meanburi","Bangkapi","Ladkrabang"]]
### Data that get from IoT
## Data = [leader or follower state, [ list of Destination*]]
#*list of Destination Code will reference by index of list

#########################    Beacon OBject ######################
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


########Beacon Advertise
def beacon_advertise(uuid, major, minor, tx, interval):
    service.start_advertising(uuid,major, minor, tx, interval)

########Beacon Scan
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
               DeviceData[0] = b._uuid
        if len(deviceData) != 0:
            break;
            
            
                
        
##### Leader Mode robot will advertise untill get join from follower
##### then close beacon and publish control data to mac topic
        
def leader(data,des):
    microgear.subscribe("/"+my_mac)
    microgear.on_message
    beacon_advertise("12345678-1234-1234-1234-"+my_mac,1,1,1,1)
    
#### Follower Mode tobot will scan for Beacon data then join to topic leader
#### and send join to topic and get data from leader
    
def follower(data,des):
    beacon_scan()
    leader_topic = deviceData[0][24:]
    microgear.publish("/"+leader_topic,"join") #### Publish join state  to leader topic
    microgear.subscribe("/"+leader_topic)   ####Subscribe Leader
    beacon_advertise("12345678-1234-1234-1234-"+my_mac,1,1,1,1)
    microgear.subscribe("/"+my_mac) #### Subscribe to own topic
    
                
        
##################### IoT ###############################
appid = "RobotCarPlatoon"
gearkey = "cxTnjFAuwED7L7P"
gearsecret =  "wvVgcX12vfZLKqDVvGtkibxWx"
microgear.create(gearkey,gearsecret,appid,{'debugmode': True})


def connection():
    logging.info("Now I am connected with netpie")

    

def subscription(topic,message):
    des = 0
    print message
    if str(message) == "join":   ### Wait Message Join for approve when member join topic 
        print "Someone join Topic: ", topic
        ####### to stop Beacon advertise must restart bluetooth adapter
        os.system("sudo service bluetooth stop" )   
        os.system("sudo service bluetooth start" )

        microgear.unsubscribe("/"+my_mac) ####Unsubscribe own topic to avoid complex data from more topic
        ###call funtion communication between leader & follower
        controlCar()
        ############## Wait
     
        


    if str(message[:5]) == "Topic":    ######### Swap topic unsub leader topic and sub new leader
        microgear.unsubscribe("/"+leader_topic) 
        microgear.subscribe("/"+message[5:])


    if str(message) == "Leader":    ######### User Choose Mode Leader 
        data[0] = 1
    if str(message) == "Follower":   ######### User Choose Mode Follower
        data[0] = 0
    
    if str(message) == "start":    ######### User Start Platooning
        print "go go!"
        if data[0] == 1:
            leader(data,des)
        if data[0] == 0:
            follower(data,des)
    
    if str(message) == "exit":      ######### Leader exit platoons
        microgear.publish("/"+my_mac,"Topic:"+leader_topic)
        microgear.unsubscribe("/"+leader_topic)
        
    
    else:     ######### Other input from free board will be destination code
        if message in data[1]:
            des = data[1].index(str(message))

    print "Now Data is ",data
    print "Destination",des

def disconnect():
    logging.info("disconnected")

############################# Control ################################
def controlCar():
    pass

#################### IoT  Comand

microgear.setalias("doraemon")
microgear.on_connect = connection
microgear.on_message = subscription
microgear.on_disconnect = disconnect
microgear.subscribe("/platoons") ###### Subscribe Freeboard
microgear.connect(True)
