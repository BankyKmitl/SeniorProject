import RPi.GPIO as GPIO
import time

class DistanceSensor:
  def __init__(self,TRIG,ECHO):
    self.TRIG = TRIG
    self.ECHO = ECHO
    
 ##setup GPIO   
  def setDistance(self):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.TRIG,GPIO.OUT)
    GPIO.setup(self.ECHO,GPIO.IN)

##to RunUltrasonic
  def runDistance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
      pulse_start = time.time()

    while GPIO.input(ECHO)==1:
      pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    print "Distance: ",distance," cm"
    return distance
    
 ##CleanUp   
  def clenupDistance():
    GPIO.cleanup()
