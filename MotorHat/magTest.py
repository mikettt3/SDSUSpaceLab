from adafruit_motorkit import MotorKit
from time import sleep
import board
import csv

c = open('magTestprograms.csv','w')

print('Start')

kit = MotorKit()


step = 1/10
ramp = step/1
xRange = 1/step

# 0V for 10 secs
kit.motor1.throttle = None
sleep(10)

# ramp 5V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = (ramp*i)
    sleep(step)

# 5V for 10 sec
kit.motor1.throttle = 1
sleep(10)

# ramp 0V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = 1 - (ramp*i)
    sleep(step)

# 0V for 10 secs
kit.motor1.throttle = 0
sleep(10)

# ramp to -5V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = 0 - (ramp*i)
    sleep(step)

# -5V for 10 sec
kit.motor1.throttle = -1
sleep(10)

# ramp 0V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = -1 + (ramp*i)
    sleep(step)

# 0V for 10 secs
kit.motor1.throttle = 0
sleep(10)

# End

'''for i in range(50):
    kit.motor1.throttle = 1.0 - 2*(i/50)
    sleep(0.1)
        
kit.motor1.throttle = None
sleep(1)
kit.motor1.throttle = -0.5
sleep(2)
kit.motor1.throttle = None
sleep(1)
kit.motor1.throttle = 0.0

kit.motor2.throttle = 0.5
sleep(1)
kit.motor2.throttle = None
sleep(1)
kit.motor2.throttle = 0.0

kit.motor3.throttle = 0.5
sleep(1)
kit.motor3.throttle = None
sleep(1)
kit.motor3.throttle = 0.0
'''

print('Done')
# commit straight to repo
