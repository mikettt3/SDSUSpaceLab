from roboclaw_3 import Roboclaw
from adafruit_motorkit import MotorKit
import time
from time import sleep
from datetime import datetime
import board
import numpy as np
import csv
import smbus
from MinIMU_v5_pi import MinIMU_v5_pi
from vnpy import *
from vnpy.libvncxx import VnSensor

# Add MinIMU_v5_pi to /home/pi/.local/lib/python3.9/site-packages/

# Accelerate all motors to rotate CubeSat hung in the testbed platform
# Make initial state, the zero state. Attempt to revert any deviations
# back to the initial state

# Update 21 Dec 2022
# Either integrate Gyro readings or use magnetometer readings and figure 
# out how to adjust for motor and MTQ running later.
# Or use VN200 if this IMU is being a PITA

# 20 Jan 2023
# Consider adjusting DriveWheels rates. Make it react faster to inputs.
# Next task. Design file so that it captures 0 point/initial state, then 
#  rotates some mount using the reaction wheels and then attempts to return 
#  to the initial state.
# After this, obtain angle values either from fixed MinIMU9 or from VN200, 
#  then do same idea of rotating and rotating back.
# Alternative: Take initial state array as 0 and figure out what 90 degress 
#  from that is according to the IMU and rotate to that point.


# Addresses and open statements and initialize
address1 = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
address2 = 0x81
roboclaw = Roboclaw("/dev/ttyS0", 38400) # Create the RoboClaw object, passing the serial port and baudrate
roboclaw.Open()      # Start communication with Roboclaw for motor full control
kit = MotorKit()     # Start communication with MotorHat for magnetorquer control.
IMU = MinIMU_v5_pi() # Start IMU communication
s = VnSensor()
s.connect('/dev/ttyUSB0', 115200)

# Make file for MotionTest.csv to save IMU data
fileDateTime = datetime.now().strftime("%y%b%d%H%M.%s")
fileName = "/home/pi/Desktop/Mike/SDSUSpaceLabADCS_Testbed/MotionTestCSV/MotionTest"
fileName = fileName+fileDateTime[0:7]+"_"+fileDateTime[7:11]+".csv"
f = open(fileName, 'w', newline='')

# # # # Make file for YawGyro0. Or don't. w/e IDC
# # # YG0_FN = "/home/pi/Desktop/Mike/SDSUSpaceLabADCS_Testbed/MotionTestCSV/YawGyro0"
# # # YG0_FN = YG0_FN+fileDateTime[0:7]+"_"+fileDateTime[7:11]+".csv"
# # # g = open(YG0_FN, 'w', newline='')

txl = 5  # extra long pause
tl = 2   # long pause time
ts = 0.5 # short pause time
DriveDir = int(0)
K_rxn    = int(1)
mDrive   = int(64)
# yawGyro0 = int(0)
# yawMag0  = int(0)

''' ------------------------------------------------'''
# Definitions

def imu(timeStamp):
    data = [timeStamp,"%03.4f " % IMU.readMagnetometer()[0],
        "%03.4f " % IMU.readMagnetometer()[1],"%03.4f " % IMU.readMagnetometer()[2],
        "%03.4f " % IMU.readAccelerometer()[0],"%03.4f " % IMU.readAccelerometer()[1],
        "%03.4f " % IMU.readAccelerometer()[2],"%03.4f " % IMU.readGyro()[0],
        "%03.4f " % IMU.readGyro()[1],"%03.4f " % IMU.readGyro()[2] ]
        # ,
        # "%03.4f " % IMU.updateAngle()[0],"%03.4f " % IMU.updateAngle()[1],
        # "%03.4f " % IMU.updateAngle()[2]    ]
    return data

def DriveWheels(yawGyro):
    # 0-127; 0-full reverse, 64-stop, 127-full forward
    # Adjustment for reaction rates. Lower value == rxn wheels 
    #   spin up faster. - is min, + is max input to be mapped to 0, 127
    #   respectively. +/- 16 is min motor drive threshold to prevent 
    #   slow starting and ensure smooth wheel behavior. Below about 
    #   6/127 the motors jerk to a start and stop rather than acting 
    #   definitively.
    thresholdHi = 30 # 150
    threshLo    = 16
    if yawGyro <= -thresholdHi:
        mDrive = 127
    elif (yawGyro > -thresholdHi) and (yawGyro <= -threshLo):
        mDrive = 63-(yawGyro*64/thresholdHi)
    elif (yawGyro > -threshLo) and (yawGyro < threshLo):
        mDrive = 64
        # Prefer coast command here.
    elif (yawGyro < thresholdHi) and (yawGyro >= threshLo):
        mDrive = 64-(yawGyro*64/thresholdHi)
    elif yawGyro >= thresholdHi:
        mDrive = 0
    else:
        print('Error in Drive loop');

    mDrive = round(mDrive);

    roboclaw.ForwardBackwardM1(address1,mDrive)
    roboclaw.ForwardBackwardM2(address1,mDrive)
    roboclaw.ForwardBackwardM1(address2,mDrive)
    roboclaw.ForwardBackwardM2(address2,mDrive)

def battery():
    # Read main battery voltage level Command 24 pg68
    if roboclaw.ReadMainBatteryVoltage(address1)[1] <= 136:
        print('!!!!!!!!!!!!!!!!!Warning!!!!!!!!!!!!!!!!!')
        print('MAIN BATTERY LOW')
        print(roboclaw.ReadMainBatteryVoltage(address1)[1])
        # sleep(tl)
        if roboclaw.ReadMainBatteryVoltage(address1)[1] <= 132:
            print('Exiting')
            stopAll()
            sleep(tl)
            quit()
            
def stopAll():
    # Stop motors
    roboclaw.ForwardM1(address1,0)
    roboclaw.ForwardM2(address1,0)
    roboclaw.ForwardM1(address2,0)
    roboclaw.ForwardM2(address2,0)
    kit.motor1.throttle = None
    kit.motor2.throttle = None
    kit.motor3.throttle = None
    print('MTQs and motors off')
    # sleep(tl)

''' ------------------------------------------------'''

'''Startup'''
stopAll()

# Set PID values
roboclaw.SetM1VelocityPID(address1,3,1,0,7125)
roboclaw.SetM2VelocityPID(address1,3,1,0,7125)
roboclaw.SetM1VelocityPID(address2,3,1,0,7125)
roboclaw.SetM2VelocityPID(address2,3,1,0,7125)

# Reset encoders. Command 20 Pg 87
roboclaw.ResetEncoders(address1)
roboclaw.ResetEncoders(address2)

# Check battery voltage
battery()
print(roboclaw.ReadMainBatteryVoltage(address1)[1])

# Start IMU functions
# IMU.trackAngle()	# Line 421 in MinIMU_v5_pi.py
'''Creates another thread which calls updateAngle every 4ms (250Hz)
    to track the current roll, pitch, and yaw. 
    Currently gives no return. Not sure why. -MS'''
IMU.updateAngle()
IMU.readGyro()
IMU.readAccelerometer()
IMU.readMagnetometer()
'''Also in package: trackYaw(), trackRoll(), updateYaw()'''

# Log time
tic = time.time()
timeStamp = datetime.now().strftime("%H%M%S.%f")

# Log and filter IMU data and return single initial state array. Noise floor.
ZSA    = np.empty((1,9))	# ZeroStateArray = [1x9]
ZSAstd = np.empty((1,9))	# Standard deviation of ISA columns
ISA    = np.empty((100,10))	# InitialStateArray = [100x9]
for x in range(0, 100):
    ISA[x,:] = imu(timeStamp)
    time.sleep(0.01)

for y in range(len(ZSA.transpose())):
#     average each column and dump into ZSA matrix
    ZSA[0,y] = np.average(ISA[:,y+1])
    ZSAstd[0,y] = np.std(ISA[:,y+1])

print('Ready')
'''End startup'''

''' ------------------------------------------------'''

# If deviation larger than 3*std then rotate back. Just watch yaw for live demo.
# Do 6 sigma so the system isn't constantly trying to battle transient measurements
# If error is + then rotate motors forward and drive mtq forward
# If error is - then rotate motors backward and drive mtq backward
# Integrate Gyro readings.
	# dt = t2-t1
	# yawk = last yaw reading
	# yawk1 = current yaw reading
	# yawdk = yawk1-yawk
	# yawSum = yawSum + (yawdk*dt)
	# Minimize yawSum
    
sleeptime = 0.01
dy = 5 # yaw threshold for movement
K = 1 # gain
runDur = 10
runtime = runDur/(10*sleeptime) # 15min * 60s = 900s
i = 0
timeStamp2 = np.empty((1,int(runtime+1)))
timeStamp2[i] = datetime.now().strftime("%H%M%S%f")[0:9]
# imuR     = np.empty((int(runtime),10))
imuR     = np.zeros((int(runtime),13))
yprSto   = np.zeros((int(runtime),3))
yawGyro0 = np.empty((int(runtime+1),1))
yawMag0  = np.empty((int(runtime+1),1))
yawSum   = 0
Rotated  = 0    # Is CubeSat rotated by the rxn wheels yet?


while i<=(runtime-1):
    # create time stamp for each data point(HHMMSS(6 fig microseconds))
    # Recommend time.time command instead of datetime.bullshit
    timeStamp2[0,i+1] = datetime.now().strftime("%H%M%S%f")[0:9]
    dt = (timeStamp2[0,i+1] - timeStamp2[0,i])/1000
    imuR[i,0:10] = imu(timeStamp2[0,i+1])   # Read IMU
    ypr = s.read_yaw_pitch_roll() # Read better IMU/INS/AHRS
    imuR[i,10:13] = [np.float64(ypr.x), np.float64(ypr.y), np.float64(ypr.z)]
    print(ypr.x, ypr.y, ypr.z)
    # print(i)
#     print(imuR[i])

    # Zero yaw data
    yawGyro0[i+1,0] = float(imuR[i,9]) - ZSA[0,8]
    yawMag0[i+1,0]  = float(imuR[i,3]) - ZSA[0,2]
    # Z mag does not change when rotating about Z. X and Y do.
    
    # yawdk = yawGyro0[i+1,0] - yawGyro0[i,0]
    # yawSum = yawSum + (yawdk*dt)
    yawSum = yawSum + (yawGyro0[i+1,0]*dt)
    print(yawSum, dt)
    
    imuR[i,10] = yawSum
    
    if Rotated == 0:    # Initial rotation of CubeSat
        DriveWheels(150)
        sleep(0.25)
        Rotated = 1
        stopAll()
	# DriveWheels(0)
        # sleep(0.5)
    
	# Minimize yawSum
    if (abs(yawSum*K))>=dy: # (abs(3*ZSAstd[0,8]))
        # print('yawGyro0')
        print(yawSum)
        # Then correct attitude
        # Send error amount and direction to drive rxn wheels or mtqs
        DriveWheels(yawSum*K)
        # DriveMTQ1(yawMag0)
    elif (abs(yawSum*K))<dy: # (abs(3*ZSAstd[0,8]))
        # Stop motors from turning when not needed.
        DriveWheels(0)
    else:
        print('Error in while loop')
  
    # Battery voltage check
    battery()
    # Time code run time. Quit if >15 min to prevent excessive runtime
    # toc = time.time()
    # if (toc-tic)> runDur:
        # print('Check runtime. Exiting soon')
    sleep(sleeptime) 
    # This allows the IMU and Roboclaws to catch up. Reduces apparent 
    # hitching in read+output errors which cause the Roboclaws to skip a 
    # command.
    i += 1  # Iterate timestep


''' ------------------------------------------------'''
print('End loop')

# Output imu data into a csv file
writer = csv.writer(f)
writer.writerows(imuR)
# writer.writerows([imuR, yprSto])
# writer.writerows([imuR,ypr.x, ypr.y, ypr.z])

# # # # Output yawGyro0 data into a csv file
# # # writer = csv.writer(g)
# # # writer.writerows(yawGyro0)
# Thought I was being cute. IDK. Might delete later.

'''Kill motors and print done'''
s.disconnect()
f.close()
stopAll()
print('Done')
# quit()
