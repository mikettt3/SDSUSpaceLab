from roboclaw_3 import Roboclaw
from adafruit_motorkit import MotorKit
from time import sleep
import board

# Motion test running MTQs only

# Addresses and open statements
address1 = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
address2 = 0x81
roboclaw = Roboclaw("/dev/ttyS0", 38400) # Create the RoboClaw object, passing the serial port and baudrate
roboclaw.Open() # Call the Open() function on the RoboClaw object and start communication
kit = MotorKit() # Start communication with MotorHat for magnetorquer control.

txl = 5  # extra long pause
tl = 2   # long pause time
ts = 0.5 # short pause time

'''Startup'''

# Set PID values
roboclaw.SetM1VelocityPID(address1,3,1,0,7125)
roboclaw.SetM2VelocityPID(address1,3,1,0,7125)
roboclaw.SetM1VelocityPID(address2,3,1,0,7125)
roboclaw.SetM2VelocityPID(address2,3,1,0,7125)

# Stop motors
roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
roboclaw.ForwardM1(address2,0)
roboclaw.ForwardM2(address2,0)
kit.motor1.throttle = None
kit.motor2.throttle = None
kit.motor3.throttle = None
sleep(tl)

# Read main battery voltage level Command 24 pg68
MBatt = roboclaw.ReadMainBatteryVoltage(address1)
print('Main battery level in 0.1 VDC (10 = 1 VDC)')
print(MBatt, '(address #, Volts DC)')

# Reset encoders. Command 20 Pg 87
roboclaw.ResetEncoders(address1)
roboclaw.ResetEncoders(address2)

print('MTQ Test Start')

# MotorHat ramp settings
step = 1/10
ramp = step/1
xRange = int(1/step)

# ramp 5V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = (ramp*i)
	kit.motor2.throttle = (ramp*i)
    sleep(step)

# 5V for 10 sec
kit.motor1.throttle = 1
kit.motor2.throttle = 1
sleep(10)

# ramp 0V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = 1 - (ramp*i)
	kit.motor2.throttle = 1 - (ramp*i)
    sleep(step)

# 0V for 10 secs
kit.motor1.throttle = 0
kit.motor2.throttle = 0
sleep(10)

# ramp to -5V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = 0 - (ramp*i)
	kit.motor2.throttle = 0 - (ramp*i)
    sleep(step)

# -5V for 10 sec
kit.motor1.throttle = -1
kit.motor2.throttle = -1
sleep(10)

# ramp 0V for 1 sec
for i in range(xRange):
    kit.motor1.throttle = -1 + (ramp*i)
	kit.motor2.throttle = -1 + (ramp*i)
    sleep(step)


'''Kill motors and print done'''
roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
roboclaw.ForwardM1(address2,0)
roboclaw.ForwardM2(address2,0)
kit.motor1.throttle = None
kit.motor2.throttle = None
kit.motor3.throttle = None
print('Done')
sleep(ts)
quit()
