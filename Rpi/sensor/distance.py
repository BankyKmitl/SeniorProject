import RPi.GPIO as GPIO
import time


class DistanceSensor:
  
  def __init__(self,TRIG,ECHO):
    self.TRIG = TRIG
    self.ECHO = ECHO
    self.distance = 0
    self.max_distance = 400
        
 ##setup GPIO   
  def setupGPIO(self):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.TRIG,GPIO.OUT)
    GPIO.setup(self.ECHO,GPIO.IN)

##to RunUltrasonic
##  @classmethod
##  def updateDistance(self, distance):
##    self.distance = distance
##

  def runDistance(self):
      GPIO.output(self.TRIG, True)
      time.sleep(0.00001)
      GPIO.output(self.TRIG, False)

      while GPIO.input(self.ECHO)==0:
        pulse_start = time.time()

      while GPIO.input(self.ECHO)==1:
        pulse_end = time.time()

      pulse_duration = pulse_end - pulse_start
      distance = pulse_duration * 17150
      distance = round(distance, 2)
      
      self.distance = min(distance,self.max_distance)
      
##      print "Distance: ",distance," cm"
      
  def get_distance(self):
      return int(self.distance)

  def check_distance(self,ultra_distance,set_distance):
      if (int(ultra_distance) in [set_distance-1,set_distance,set_distance+1]):
        return True
      else:
        return False
      
 ##CleanUp   
  def cleanup():
    GPIO.cleanup()
