from adafruit_motorkit import MotorKit
from time import sleep
import board
import csv
import math
import numpy as np

c = open('magTestprograms.csv','w')

print('Start')

kit = MotorKit()

Throttle = 1.0
freq = 10   # hertz
period = 1/freq
step = period/2

kit.motor1.throttle = 0
kit.motor2.throttle = 0
kit.motor3.throttle = 0
print('Throttle = 0')
sleep(1)

# kit.motor1.throttle = Throttle
# kit.motor2.throttle = Throttle
# kit.motor3.throttle = Throttle

print('On')

'''Square Wave'''
##while True:
##    kit.motor3.throttle = Throttle
##    sleep(step)
##    kit.motor3.throttle = -Throttle
##    sleep(step)

'''Code for Sine function'''
dsin = 101        # Size of sine array. pi/2, pi must land on integer values
dt = period/dsin  # time step
PI = math.pi      # Pi
Sine = [0 for element in range(dsin)] # Initialize sine array
piArray  = np.linspace(0, 2*PI, dsin)

''' Define sine array'''
for i in range(dsin):
    Sine[i] = math.sin(piArray[i])

while True:
    for j in range(dsin):
        kit.motor3.throttle = Throttle*Sine[j]
        # print(Throttle*Sine[j])
        sleep(dt)


# while True:
#     for i in range(10):
#         kit.motor3.throttle = (0.1*i*Throttle)
#         sleep(step)
# 
#     print('Done 1')
#     
#     for i in range(20):
#         kit.motor3.throttle = Throttle - (0.1*i*Throttle)
#         sleep(step)
# 
#     print('Done 2')
#     
#     for i in range(10):
#         kit.motor3.throttle = 0 - (0.1*i*Throttle)
#         sleep(step)
# 
#     print('Done 3')
#     
#     for i in range(10):
#         kit.motor3.throttle = -Throttle + (0.1*i*Throttle)
#         sleep(step)
        
print('Done all')

