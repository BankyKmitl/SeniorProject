import time
import RPi.GPIO as GPIO

class DistanceSensor():
        def __init__(self,TRIG,ECHO):
            GPIO.setmode(GPIO.BCM)
            self.TRIG = TRIG
            self.ECHO = ECHO
            self.distance = 0
            GPIO.setup(self.TRIG,GPIO.OUT)
            GPIO.setup(self.ECHO,GPIO.IN)

        def run(self):
            while (True):
              GPIO.output(self.TRIG, True)
              time.sleep(0.00001)
              GPIO.output(self.TRIG, False)
              while GPIO.input(self.ECHO)==0:
                pulse_start = time.time()

              while GPIO.input(self.ECHO)==1:
                pulse_end = time.time()

              pulse_duration = pulse_end - pulse_start
              self.distance = int(pulse_duration * 17150)
              if (self.distance > 400):
                      self.distance = 400
              print self.distance
              time.sleep(1)

        def check_distance(self,ultra_distance,set_distance):
              if (int(ultra_distance) in [set_distance-1,set_distance,set_distance+1]):
                return True
              else:
                return False

d = DistanceSensor(16,19)
d.run()
