from adafruit_motorkit import MotorKit
from time import sleep
import board
import csv

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
while True:
    kit.motor3.throttle = Throttle
    sleep(step)
    kit.motor3.throttle = -Throttle
    sleep(step)

'''Thoughts on code for Sin function
Lookup table for sin values. Tables are probably faster than math.sin() verify?
https://www.grc.nasa.gov/www/k-12/airplane/tablsin.html

Loop for 1 - 90 possibly by 2 or 3. (Math sin(radians))
While true
    Loop +89 - -90
    Loop -89 - +90

#######

ArraySin = [0 for element in range(90)]
for i in range(90)
    ArraySin(i) = math sin(i)


'''


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

