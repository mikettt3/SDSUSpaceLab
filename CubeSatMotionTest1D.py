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

# Add MinIMU_v5_pi to /home/pi/.local/lib/python3.9/site-packages/

# Accelerate all motors to rotate CubeSat hung in the testbed platform
# Make initial state, the zero state. Attempt to revert any deviations
# back to the initial state

# Update 21Dec22
# Either integrate Gyro readings or use magnetometer readings and figure 
# out how to adjust for motor and MTQ running later.
# Or use VN200 if this IMU is being a PITA


# Addresses and open statements and initialize
address1 = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
address2 = 0x81
roboclaw = Roboclaw("/dev/ttyS0", 38400) # Create the RoboClaw object, passing the serial port and baudrate
roboclaw.Open()      # Start communication with Roboclaw for motor full control
kit = MotorKit()     # Start communication with MotorHat for magnetorquer control.
IMU = MinIMU_v5_pi() # Start IMU communication

fileDateTime = datetime.now().strftime("%y%b%d%H%M.%s")
fileName = "/home/pi/Desktop/Mike/SDSUSpaceLabADCS_Testbed/MotionTestCSV/MotionTest"
fileName = fileName+fileDateTime[0:7]+"_"+fileDateTime[7:11]+".csv"
f = open(fileName, 'w', newline='')

txl = 5  # extra long pause
tl = 2   # long pause time
ts = 0.5 # short pause time
DriveDir = int(0)
K_rxn    = int(1)
mDrive   = int(64)
yawGyroErr = int(0)
yawMagErr  = int(0)

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

def DriveWheels(yawGyroErr):
    # 0-127; 0-full reverse, 64-stop, 127-full forward

    # # Save as B
    if yawGyroErr <= -150:
        mDrive = 127
    elif (yawGyroErr > -150) and (yawGyroErr <= -16):
        mDrive = 63-(yawGyroErr*64/150)
    elif (yawGyroErr > -16) and (yawGyroErr < 16):
        mDrive = 64
    elif (yawGyroErr < 150) and (yawGyroErr >= 16):
        mDrive = 64-(yawGyroErr*64/150)
    elif yawGyroErr >= 150:
        mDrive = 0
    else:
        print('Error in YGE loop');
    
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
        sleep(txl)
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
    sleep(tl)

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

# Start IMU functions
IMU.trackAngle()	# Line 421 in MinIMU_v5_pi.py
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
timeStamp = datetime.now().strftime("%H%M%S.%s")[0:6]

# Log and filter IMU data and return single initial state array.
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
runDur = 5
runtime = runDur/(10*sleeptime) # 15min * 60s = 900s
i = 0
timeStamp2 = np.empty((1,int(runtime+1)))
imuR           = np.empty((int(runtime),10))
timeStamp2[i] = datetime.now().strftime("%H%M%S.%s")[0:6]

while i<=(runtime-1):
    # create time stamp for each data point
    timeStamp2[0,i+1] = datetime.now().strftime("%H%M%S.%s")[0:6]
    dt = timeStamp2[0,i+1] - timeStamp2[0,i]
    imuR[i,:] = imu(timeStamp2[0,i+1])   # Read IMU

    # Estimate yaw error from gyro and mag.
    yawGyroErr = float(imuR[i,9]) - ZSA[0,8]
    yawMagErr  = float(imuR[i,3]) - ZSA[0,2]

    if (abs(yawGyroErr))>=(abs(6*ZSAstd[0,8])):
        # print('YawGyroErr')
        print(yawGyroErr)
        # Then correct attitude
        # Send error amount and direction to drive rxn wheels or mtqs
        DriveWheels(yawGyroErr)
        # DriveMTQ1(yawMagErr)
    elif (abs(yawGyroErr))<(abs(6*ZSAstd[0,8])):
        # print('Less than yaw error')
        yawGyroErr = 0
        yawMagErr  = 0
        DriveWheels(yawGyroErr)
    else:
        print('Error in while loop')

    # Output data into a csv file
    writer = csv.writer(f)
    writer.writerow(imuR)

    # Time code run time. Quit if >15 min to prevent excessive runtime
    # Battery voltage check
    battery()
    # toc = time.time()
    # if (toc-tic)> runDur:
        # print('Check runtime. Exiting soon')
    sleep(sleeptime)
    i += 1  # Iterate timestep


''' ------------------------------------------------'''

'''Kill motors and print done'''
stopAll()
f.close()
print('Done')
# quit()
