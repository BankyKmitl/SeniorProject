from bluetooth.ble import BeaconService
import thread
import time
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
    
service = BeaconService("hci0")
devices = service.scan(5)
time_st = hex(int(time.time()))[2:]
idPlatoon = "00"
orderPlatoon = "00"
distanceLeader = "0000"
direction = "01"
ultraDistance = "1234"
totalDistance ="123456"
leader = ""
uuid = time_st + "-" + idPlatoon + orderPlatoon +"-" +"0000"+"-"+ distanceLeader +"-" + direction +ultraDistance + totalDistance
deviceData = []
rank = []
guider = ""

def calculate_accuracy(txpower, rssi):
    if rssi == 0 or txpower == 0:
        return -1
    else:
        ratio = rssi/txpower
        if ratio < 1:
            return ratio**10
        else:
            return 0.89976 * ratio**7.7095 + 0.111
        
def beacon_advertise(uuid, major, minor, tx, interval):
    print "Advertise ..."
    while (True):
        service.start_advertising(uuid,major, minor, tx, interval)
        

def beacon_scan():
    print "scan start ..."
    while (True):
        devices = service.scan(5)
        for address, data in list(devices.items()):
            b = Beacon(data, address)
            print b._address
            if len(deviceData) == 0 :
                deviceData.append([[b._address],[b._uuid],[b._power],[b._rssi],[b._major],[b._minor]])
            else:        
                for i in range(len(deviceData)):
                    if b._address not in deviceData[i][0] :
                        deviceData.append([[b._address],[b._uuid],[b._power],[b._rssi],[b._major],[b._minor]])
                    else:
                        deviceData[i][1] = b._uuid
                        deviceData[i][2] = b._power
                        deviceData[i][3] = b._rssi
                        deviceData[i][4] = b._major
                        deviceData[i][5] = b._minor
            print(deviceData)
            print calculate_accuracy(b._power,b._rssi)
                
def beacon_shake():
        devices = service.scan(5)
        for address, data in list(devices.items()):
            b = Beacon(data, address)
            if len(deviceData) == 0 :
                deviceData.append([[b._address],[b._uuid],[b._power],[b._rssi],[b._major],[b._minor]])
            else:        
                for i in range(len(deviceData)):
                    print str(i)+"is"
                    if b._address not in deviceData[i][0] :
                        deviceData.append([[b._address],[b._uuid],[b._power],[b._rssi],[b._major],[b._minor]])
                    else:
                        deviceData[i][1] = b._uuid
                        deviceData[i][2] = b._power
                        deviceData[i][3] = b._rssi
                        deviceData[i][4] = b._major
                        deviceData[i][5] = b._minor
            if int((b._uuid[26:30])) == 400:    
                leader = str(b._address)
                rssi_leader = str(b._rssi)
                power_leader = str(b._power)
                distanceLeader = "00"+str(abs(b._rssi))
                uuid = time_st + "-" + idPlatoon + orderPlatoon +"-" +"0000"+"-"+ distanceLeader +"-" + direction +ultraDistance + totalDistance
                service.start_advertising(uuid,1, 1, 4, 1)
                rank.append(abs(b._rssi))
        for i in deviceData:
            rank.append(abs(i[3][0]))
        guider = ""
        rank.sort()
        print deviceData
        print rank
        print rank.index(int(distanceLeader))
        if rank.index(int(distanceLeader)) == 0:
             for i in deviceData:
                 if abs(i[3][0]) ==  rank[0]:
                     print i[0]
                     guider = i[0][0]
        else:
            for i in deviceData:
                if abs(i[3][0]) == rank[rank.index(int(distanceLeader))-1]:
                    print i[0]
                    guider = i[0][0]
        print "this is " + guider
        

    
    
        
#beacon_shake()         

service = BeaconService()
devices = service.scan(2)
thread.start_new_thread(beacon_scan, ())
thread.start_new_thread(beacon_advertise(uuid,1,1,4,1), ())


  

print("Done.")
