import RPi.GPIO as GPIO
import time
from collections import deque

class Map:
    def __init__(self):
        self.data = deque()

    def delData(self,time):
        for i in range(len(self.data)):
            if (self.data[i][2] == time):
                del self.data[i]
                break
            
    def extendleft(self,message):
        if len(message) != 0:
            self.data.extendleft(message)
        
    def extend(self,message):
        if len(message) != 0:
            self.data.extend(message)

    def append(self,message):
        if len(message) != 0:
            self.data.append(message)

    def getData(self,time):
        for i in range(len(self.data)):
            if (self.data[i][2] == time):
                return self.data[i]
            
    def getLen(self):
        return len(self.data)

    def getFirst(self):
        try:
            return self.data[0]
        except IndexError:
            pass

    def getFirst_str(self):
        try:
            first = ";".join(self.data[0])
            return first
        except IndexError:
            pass
        
    def getMap(self):
        return self.data

    def getLast(self):
        last = ";".join(self.data[-1])
        return last
    
            
            
