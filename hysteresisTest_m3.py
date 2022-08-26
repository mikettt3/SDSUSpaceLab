from adafruit_motorkit import MotorKit
from time import sleep
import board
import csv

c = open('magTestprograms.csv','w')

print('Start')

kit = MotorKit()

Throttle = 1.0
freq = 5   # hertz
period = 1/freq
step = period/4

kit.motor3.throttle = 0
print('Throttle = 0')
sleep(1)

# kit.motor3.throttle = Throttle

while True:
    for i in range(10):
        kit.motor3.throttle = (0.1*i*Throttle)
        sleep(step)

    print('Done 1')
    
    for i in range(20):
        kit.motor3.throttle = Throttle - (0.1*i*Throttle)
        sleep(step)

    print('Done 2')
    
    for i in range(10):
        kit.motor3.throttle = 0 - (0.1*i*Throttle)
        sleep(step)

    print('Done 3')
    
    for i in range(10):
        kit.motor3.throttle = -Throttle + (0.1*i*Throttle)
        sleep(step)
        
print('Done all')

