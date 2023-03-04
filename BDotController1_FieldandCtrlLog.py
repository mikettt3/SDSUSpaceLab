from roboclaw_3 import Roboclaw
from adafruit_motorkit import MotorKit
import time
from time import sleep
from datetime import datetime
import board
import numpy as np
import csv
import smbus
from vnpy import *
from vnpy.libvncxx import VnSensor

# ##
# Make Bdot controller. Using magnetometer readings
#  Proportional controller:
# 	mu_ctrl = -K*B_dot
# 	T_ctrl = mu_ctrl X B
# 
# 	T_Bx   [0   Bz -By][mu_x]
# 	T_By = [-Bz  0  Bx][mu_y]
# 	T_Bz   [By -Bx   0][mu_z]
# 	
# 	Advantages are simple and easy to implement.
# 	Disadvantage that it sends power to MTQ when they are off axis and 
# 	cannot offer much control actuation torque. Wastes energy and time.
# 
#  Bdot bang bang controller:
# 	mu_i,ctrl = -mu_max*sgn(Bdot_i) = { mu_max for Bdot_i <0
# 									  {-mu_max for Bdot_i >0
# 		mu_max is max MTQ torque in Am^2. 
# 		
# 	Faster detumbling and more energy efficient than proportional. 
# 	Slightly more complicated. Can produce electrical noise and voltage 
# 	spikes as the controller will command an immediate reversal of current 
#  	flow in an inductor. This can harm equipment. Measures have been taken 
# 	in the hardware on this project to reduce and eliminate these voltage 
# 	spikes, but this transitional period should still be avoided. This 
# 	method also wastes energy when the magnetorquer axis and the magnetic 
# 	field axis are nearly aligned. This gives some slight detumbling time 
# 	advantage but not much.
# 
#  Step bang-bang controller:
# 	Modified bang-bang using the proportional control values.
# 	
# 	mu_i,ctrl = { 0 Am^2	 	if mu_i,prop/mu_prop < 0.33
# 				{ 0.5*mu_max 	if 0.33 <= mu_i,prop/mu_prop <= 0.66 
# 								(or just use "else" here as it is simpler 
# 								and faster logic)
# 				{ mu_max		if mu_i,prop/mu_prop > 0.66
# 		mu_prop is absolute max value of controlling mag. moment from 
# 			proportional controller.
# 	Offers a medium between the slower and more wasteful proportional 
# 	controller and the faster but potentially electrically damaging 
# 	bang-bang controller. This gives the system time to adjust electrically 
# 	and wastes less energy.
# 	The optimal system here would be to have a smoother cross between the 
# 	step bang-bang controller and the proportional controller. The proportional 
# 	controller offers a smooth voltage ramp but wastes time and energy. The step 
# 	bang-bang attempts to ramp quickly to an interstitial state but is much 
# 	better than the pure bang-bang controller in safety and energy use. A system 
# 	here could be the step-prop-bang-bang controller. Or ramp-bang-bang.
# 	
#  Step-prop-bang-bang
# 	mu_i,ctrl = { 0 Am^2	 	if mu_i,prop/mu_prop < 0.25
# 				{ mu_i,prop 	if 0.25 <= mu_i,prop/mu_prop <= 0.5 
# 				{ mu_max		if mu_i,prop/mu_prop > 0.5
# 	mu_i,prop can also be mapped from 25-50% to 0-50%. This further improves 
# 	ramping functionality.
# 	This allows a more efficient use of the MTQs when they are called and 
# 	minimizes electrical noise and spikes while also not wasting energy at all 
# 	during periods where the magnetic moment is nearly aligned with the magnetic 
# 	field and thus provides minimal or no actuation torque. Another advantage of 
# 	this style is that the voltage applied to the magnetorquers can be safely 
# 	increased with minimal risk of high voltage spikes or electrical heating 
# 	since the duty cycleof the magnetorquers is reduced.

# Addresses and open statements and initialize
address1 = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
address2 = 0x81
roboclaw = Roboclaw("/dev/ttyS0", 38400) # Create the RoboClaw object, passing the serial port and baudrate
roboclaw.Open()      # Start communication with Roboclaw for motor full control
kit = MotorKit()     # Start communication with MotorHat for magnetorquer control.
# IMU = MinIMU_v5_pi() # Start IMU communication
s = VnSensor()
s.connect('/dev/ttyUSB0', 115200)

# Make file for BDotTest.csv to save IMU data
fileDateTime = datetime.now().strftime("%y%b%d%H%M.%s")
fileName = "/home/pi/Desktop/Mike/SDSUSpaceLabADCS_Testbed/BDotTestCSV/BDotTest"
fileName = fileName+fileDateTime[0:7]+"_"+fileDateTime[7:11]+".csv"
f = open(fileName, 'w', newline='')


''' ------------------------------------------------'''
# Definitions of functions
DriveDir = int(0)
K_rxn    = int(1)
mDrive   = int(64)
def DriveWheels(yawGyro):
    # 0-127; 0-full reverse, 64-stop, 127-full forward
    # Adjustment for reaction rates. Lower value == rxn wheels 
    #   spin up faster. - is min, + is max input to be mapped to 0, 127
    #   respectively. +/- 16 is min motor drive threshold to prevent 
    #   slow starting and ensure smooth wheel behavior. Below about 
    #   6/127 the motors jerk to a start and stop rather than acting 
    #   definitively.
    thresholdHi = 150 # 150
    threshLo    = 20
            
    if yawGyro <= -thresholdHi:
        mDrive = 0
    elif (yawGyro > -thresholdHi) and (yawGyro <= -threshLo):
        mDrive = 64+(yawGyro*64/thresholdHi)
    elif (yawGyro > -threshLo) and (yawGyro < threshLo):
        mDrive = 64
        # Prefer coast command here.
    elif (yawGyro < thresholdHi) and (yawGyro >= threshLo):
        mDrive = 63+(yawGyro*64/thresholdHi)
    elif yawGyro >= thresholdHi:
        mDrive = 127
    else:
        print('Error in Drive loop');

    mDrive = round(mDrive);

    roboclaw.ForwardBackwardM1(address1,mDrive)
    roboclaw.ForwardBackwardM2(address1,mDrive)
    roboclaw.ForwardBackwardM1(address2,mDrive)
    roboclaw.ForwardBackwardM2(address2,mDrive)

def battery():
    # Read main battery voltage level Command 24 pg68
    MainBattV = roboclaw.ReadMainBatteryVoltage(address1)[1] 
    if MainBattV <= 130:
        print('!!!!!!!!!!!!!!!!!Warning!!!!!!!!!!!!!!!!!')
        print('MAIN BATTERY LOW')
        print(MainBattV)
        if MainBattV <= 120:
            print('Exiting')
            stopAll()
            # Output imu data into a csv file
            writer = csv.writer(f)
            writer.writerows(imuR)
            s.disconnect()
            f.close()
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
    
def r2d(rad):
    deg = rad*180/3.1415926535
    return deg

''' ------------------------------------------------'''

'''Startup'''
stopAll()
sleep(2)

# Reset encoders. Command 20 Pg 87
roboclaw.ResetEncoders(address1)
roboclaw.ResetEncoders(address2)

# Check battery voltage
battery()
print(roboclaw.ReadMainBatteryVoltage(address1)[1])

# Log time
# Clean up datetime.mess fucntions and replace with time.time where applicable and functional
tic = time.time()
timeStamp = datetime.now().strftime("%H%M%S.%f")

K_B = 30

runDur = 30
runtime = runDur*10 # 15min * 60s = 900s
i = 0
timeStamp2 = np.empty((1,int(runtime+1)))
timeStamp2[i] = datetime.now().strftime("%H%M%S%f")[0:9]
imuR      = np.zeros((int(runtime), 22))
mu_ctrl   = np.zeros((3,1))
turnClock = 0

try:
    while i<=(runtime-1):
        # Create time stamp for each data point(HHMMSS(6 fig microseconds))
        # Recommend time.time command instead of datetime.xxxxx
        timeStamp2[0,i+1] = datetime.now().strftime("%H%M%S%f")[0:9]
        dt = (timeStamp2[0,i+1] - timeStamp2[0,i])/1000
    
        # Read IMU
        reg = s.read_yaw_pitch_roll_magnetic_acceleration_and_angular_rates()
        imuR[i,0:13] = [timeStamp2[0,i+1], 
            reg.mag.x, reg.mag.y, reg.mag.z,
            reg.accel.x, reg.accel.y, reg.accel.z,
            r2d(reg.gyro.x), r2d(reg.gyro.y), r2d(reg.gyro.z),
            reg.yaw_pitch_roll.x, reg.yaw_pitch_roll.y, reg.yaw_pitch_roll.z]

        # If i>=1 and imuR[i,10] crosses 180 deg. It will only do this at 180. 
        # Check if going from pos to neg or neg to pos.
        if (i>=1): 
            if((imuR[i,10]-imuR[i-1,10]) > 100):
                turnClock -= 1
            elif (imuR[i,10]-imuR[i-1,10]) < -100:
                turnClock += 1
            else:
                pass
        # Make yaw angle for control smooth and continuous
        yaw = turnClock*360 + imuR[i,10]

        # BDot controller
        #  Proportional controller:
        # 	mu_ctrl = -K*B_dot
        #   [3x1] = [K]*[3x1]
        # 	T_ctrl = mu_ctrl X B
        # 	T_cx   [0   Bz -By][mu_x]
        # 	T_cy = [-Bz  0  Bx][mu_y]
        # 	T_cz   [By -Bx   0][mu_z]
        

# calibrate X, Y, Z magnetorquers with VN200.
# Update this file from latest version of BDotController1.py
# Remove commanded MTQ B field from VN200 measurements. Implement this 
# in BDotController code later. 
# Make new repository folder
# Start VN data collection
# zero, pause
# X mtq in steps of 0.1 up to 1.0 for 5 s. Then down to -1.0, 5 s then 
# back to 0.0. Repeat for Y and Z mtqs. Then spin at a constant slow 
# rate (wheels should increase speed accordingly during maneuver). 
# Run X Y and Z mtqs separately and record Mag values for 1 revolution 
# for each one.
# Implement as another function to subtract induced B field from VN200 
# readings while they run.














        if i == 0:
            B_dot = [0, 0, 0]
        else:
            B_dot = imuR[i,1:4]-imuR[i-1,1:4]
            B_dot = B_dot/(imuR[i,0]-imuR[i-1,0])
        
        for j in range(0,3):
            mu_ctrl[j] = -K_B*B_dot[j]
        
        Bx = imuR[i,1]
        By = imuR[i,2]
        Bz = imuR[i,3]
        
        Tcx = 0 + (Bz*mu_ctrl[1]) + (-By*mu_ctrl[2])
        Tcy = (-Bz*mu_ctrl[0]) + 0 + (-Bx*mu_ctrl[2])
        Tcz = (By*mu_ctrl[0]) + (-Bx*mu_ctrl[1]) + 0
        T_ctrl = np.array((Tcx, Tcy, Tcz))
        
        for j in range(0,3):
            if T_ctrl[j] > 1:
                T_ctrl[j] = 1.0
            elif T_ctrl[j] < -1:
                T_ctrl[j] = -1.0
            elif (T_ctrl[j] < 0.2) and (T_ctrl[j] > -0.2):
                T_ctrl[j] = 0.0
            else:
                pass
    
        print(mu_ctrl.transpose())
            
        # Use T_ctrl to control mtqs
        kit.motor1.throttle = T_ctrl[0]
        kit.motor2.throttle = T_ctrl[1]
        kit.motor3.throttle = T_ctrl[2]
        

        imuR[i,13:16] = B_dot
        imuR[i,16:19] = mu_ctrl[0:2,0]
        imuR[i,19:21] = T_ctrl[0:3,0]

        # Battery voltage check
        battery()
        i += 1

except KeyboardInterrupt:
	print("User has stopped program")
	pass
finally:
    sleep(0.1)
    stopAll()
    writer = csv.writer(f)
    writer.writerows(imuR)
    s.disconnect()
    f.close()
    print('Done')
    '''Kill motors and print done'''







