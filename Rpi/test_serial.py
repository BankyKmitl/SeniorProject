import serial
import time
ser = serial.Serial('/dev/ttyACM0',115200)
pwm_a = 50
pwm_b = 50
while True:
    ser.write("1 " + str(pwm_a) + " " + str(pwm_b)+";")
