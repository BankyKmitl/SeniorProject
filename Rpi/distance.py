import RPi.GPIO as GPIO
import time
from threading import Thread

class DistanceSensor(threading.Thread):
  def __init__(self,TRIG,ECHO):
    threading.Thread.__init__(self)
    self.TRIG = TRIG
    self.ECHO = ECHO
    self.distance = 0
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.TRIG,GPIO.OUT)
    GPIO.setup(self.ECHO,GPIO.IN)

  def run(self):
    print "before while"
    while (True):
      GPIO.output(self.TRIG, True)
      time.sleep(0.00001)
      GPIO.output(self.TRIG, False)
      while GPIO.input(self.ECHO)==0:
        pulse_start = time.time()

      while GPIO.input(self.ECHO)==1:
        pulse_end = time.time()

      pulse_duration = pulse_end - pulse_start
      self.distance = round(pulse_duration * 17150, 2)
      time.sleep(1)
