from roboclaw_3 import Roboclaw
from adafruit_motorkit import MotorKit
import time
from time import sleep
import board
import numpy as np
import csv
import pymp #imports the pymp package to utulize OpenMP which allows you to use all cores of pi.
import smbus
from datetime import datetime
from MinIMU_v5_pi import MinIMU_v5_pi

# Add MinIMU_v5_pi to /home/pi/.local/lib/python3.9/site-packages/

# Accelerate all motors to rotate CubeSat hung in the testbed platform
# Make initial state, the zero state. Attempt to revert any deviations 
# back to the initial state

# Addresses and open statements
address1 = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
address2 = 0x81
roboclaw = Roboclaw("/dev/ttyS0", 38400) # Create the RoboClaw object, passing the serial port and baudrate
roboclaw.Open()      # Start communication with Roboclaw for motor full control
kit = MotorKit()     # Start communication with MotorHat for magnetorquer control.
IMU = MinIMU_v5_pi() # Start IMU communication

txl = 5  # extra long pause
tl = 2   # long pause time
ts = 0.5 # short pause time

def imu(timeStamp):
    data = [timeStamp,"%03.4f " % IMU.readMagnetometer()[0],"%03.4f " % IMU.readMagnetometer()[1],"%03.4f " % IMU.readMagnetometer()[2],
        "%03.4f " % IMU.readAccelerometer()[0],"%03.4f " % IMU.readAccelerometer()[1],"%03.4f " % IMU.readAccelerometer()[2],
        "%03.4f " % IMU.readGyro()[0],"%03.4f " % IMU.readGyro()[1],"%03.4f " % IMU.readGyro()[2]]
    return data

'''Startup'''
# Stop motors
print('MTQs and motors off')
roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
roboclaw.ForwardM1(address2,0)
roboclaw.ForwardM2(address2,0)
kit.motor1.throttle = None
kit.motor2.throttle = None
kit.motor3.throttle = None
print('MTQs and motors off')
sleep(tl)

# Set PID values
roboclaw.SetM1VelocityPID(address1,3,1,0,7125)
roboclaw.SetM2VelocityPID(address1,3,1,0,7125)
roboclaw.SetM1VelocityPID(address2,3,1,0,7125)
roboclaw.SetM2VelocityPID(address2,3,1,0,7125)

# Reset encoders. Command 20 Pg 87
roboclaw.ResetEncoders(address1)
roboclaw.ResetEncoders(address2)

# Read main battery voltage level Command 24 pg68
MBatt = roboclaw.ReadMainBatteryVoltage(address1)
print('Main battery level in 0.1 VDC (10 = 1.0 VDC)')
print(MBatt, '(address #, Volts DC)')
if int(MBatt[1]) <= 132:
    print('!!!!!!!!!!!!!!!!!Warning!!!!!!!!!!!!!!!!!')
    print('MAIN BATTERY LOW. Exiting.')
    sleep(txl)
    quit()

# Start IMU functions
IMU.trackAngle()
IMU.readGyro()
IMU.readAccelerometer()
IMU.readMagnetometer()

# Log and filter IMU data and return single initial state array.
dataNow = datetime.now()    # create time stamp for each data point
timeStamp = dataNow.strftime("%H%M%S.%s")
timeStamp = timeStamp[0:10]
# print(timeStamp[0:10])
ZSA = np.empty((1,9))	# ZeroStateArray = [1x9]
ZSAstd = np.empty((1,9))	# Standard deviation of ISA columns
ISA = np.empty((100,10))	# InitialStateArray = [100x9]
for x in range(0, 100):
    ISA[x,:] = imu(timeStamp)
    time.sleep(0.01)

for y in range(len(ZSA.transpose())):
#     average each column and dump into ZSA matrix
    ZSA[0,y] = np.average(ISA[:,y+1])
    ZSAstd[0,y] = np.std(ISA[:,y+1])
    
    
print(ZSA)
print('\n')
print(ZSAstd)

sleep(txl)
'''End startup'''

# if deviation larger than 3*std then rotate back. Just watch yaw for live demo.






print('Test Start')


# Spin all 4 rxn wheels from 0 to full forward
# 0-127; 0-full reverse, 64-stop, 127-full forward
for i in range(64, 127, 1):
    roboclaw.ForwardBackwardM1(address1,i)
    roboclaw.ForwardBackwardM2(address1,i)
    roboclaw.ForwardBackwardM1(address2,i)
    roboclaw.ForwardBackwardM2(address2,i)
    sleep(0.01)

# Spin all 4 rxn wheels from full forward to full reverse
for i in range(127, 0, -1):
    roboclaw.ForwardBackwardM1(address1,i)
    roboclaw.ForwardBackwardM2(address1,i)
    roboclaw.ForwardBackwardM1(address2,i)
    roboclaw.ForwardBackwardM2(address2,i)
    sleep(0.01)
    
# Spin all 4 rxn wheels from full reverse to 0
for i in range(0, 64, 1):
    roboclaw.ForwardBackwardM1(address1,i)
    roboclaw.ForwardBackwardM2(address1,i)
    roboclaw.ForwardBackwardM1(address2,i)
    roboclaw.ForwardBackwardM2(address2,i)
    sleep(0.01)
    
print('Motors done')
sleep(tl)

# MotorHat ramp settings
print('Starting MTQs')
step = 1/10
ramp = step/1
xRange = int(1/step)

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


'''Kill motors and print done'''
roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
roboclaw.ForwardM1(address2,0)
roboclaw.ForwardM2(address2,0)
kit.motor1.throttle = None
kit.motor2.throttle = None
kit.motor3.throttle = None
sleep(ts)
print('Done')
quit()
