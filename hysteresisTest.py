from adafruit_motorkit import MotorKit
from time import sleep
import board
import csv

c = open('magTestprograms.csv','w')

print('Start')

kit = MotorKit()

Throttle = 1.0
freq = 50   # hertz
period = 1/freq
step = period/4

while True:
    for i in range(10):
        kit.motor1.throttle = (0.1*i*Throttle)
        sleep(step)

    for i in range(10):
        kit.motor1.throttle = Throttle - (0.1*i*Throttle)
        sleep(step)

    for i in range(10):
        kit.motor1.throttle = 0 - (0.1*i*Throttle)
        sleep(step)

    for i in range(10):
        kit.motor1.throttle = -Throttle + (0.1*i*Throttle)
        sleep(step)
        
print('Done')