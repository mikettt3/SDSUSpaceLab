from adafruit_motorkit import MotorKit
from time import sleep
import board

print('Start')

kit = MotorKit()

for i in range(50):
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

print('Done')