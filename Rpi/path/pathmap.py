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
        
    def update(self,message):
        if len(message) != 0:
            self.data.append(message)

    def getData(self,time):
        for i in range(len(self.data)):
            if (self.data[i][2] == time):
                return self.data[i]

    def popLeft(self):
            self.data.popleft()
            
    def getLen(self):
        return len(self.data)

    def getFirst(self):
        try:
            return self.data[0]
        except IndexError:
            pass
    
    def getMap(self):
        return self.data

    def getLast(self):
        last = ";".join(self.data[-1])
        return last
    
            
            
