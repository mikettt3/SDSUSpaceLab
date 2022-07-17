from roboclaw_3 import Roboclaw
from time import sleep

# Accelerate each motor on each roboclaw, read the encoder speed and then slow and stop each motor.
# Page numbers in roboclaw_user_manual.pdf located at
# file:///home/spacepi2/Desktop/SDSUSpaceLabADCS_Testbed/roboclaw_python1/roboclaw_user_manual.pdf

#Test out forwards and backwards operation of a single roboclaw
address1 = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
address2 = 0x81
roboclaw = Roboclaw("/dev/ttyS0", 38400) # Create the RoboClaw object, passing the serial port and baudrate
roboclaw.Open() # Call the Open() function on the RoboClaw object and start communication

txl = 5  # extra long pause
tl = 2   # long pause time
ts = 0.5 # short pause time

print('Test Start')

# Stop motors
roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
sleep(tl)

# Read main battery voltage level Command 24 pg68
MBatt = roboclaw.ReadMainBatteryVoltage(address1)
print('Main battery level in 0.1 VDC (10 = 1 VDC)')
print(MBatt, '(add #, Volts DC)')

# Reset encoders. Command 20 Pg 87
roboclaw.ResetEncoders(address1)

# Read PID Commands 55,56,63,64. pg 98,99
M1V_PID = roboclaw.ReadM1VelocityPID(address1)
M2V_PID = roboclaw.ReadM2VelocityPID(address1)
M1P_PID = roboclaw.ReadM1PositionPID(address1)
M2P_PID = roboclaw.ReadM2PositionPID(address1)
print(M1V_PID, 'M1V_PID [P I D QPPS CRC]')
print(M2V_PID, 'M2V_PID [P I D QPPS CRC]')
print(M1P_PID, 'M1P_PID [P I D MaxI Deadzone MinPos MaxPos CRC]')
print(M2P_PID, 'M2P_PID [P I D MaxI Deadzone MinPos MaxPos CRC]')

# Command motors with SpeedAccelM1/M2/M1M2
# Command 38,39,40. Pg 93-94
# SpeedAccelM1/M2(self,address,accel,speed)
# SpeedAccelM1M2(self,address,accel,speed1,speed2)
# Values in QPPS. Accel [QPPS/Sec]; speed [QPPS]
# Recall 48 QPP for one full motor rotation, 211.2 for 1 full wheel rotation
roboclaw.SpeedAccelM1(address1,48,24)
sleep(txl)
roboclaw.SpeedAccelM2(address1,48,-211)
sleep(txl)
roboclaw.SpeedAccelM1M2(address1,48,48,96)
sleep(txl)
print('End Command motors with SpeedAccel')

# Command motors with SpeedAccelM1/M2/M1M2_2
# Command 50. Pg 97
# def SpeedAccelM1M2_2(self,address,accel1,speed1,accel2,speed2)
# Values should be in QPPS
roboclaw.SpeedAccelM1M2_2(address1,422,   1,422,   1)
sleep(tl)
roboclaw.SpeedAccelM1M2_2(address1,422,  -1,422,  -1)
sleep(tl)
roboclaw.SpeedAccelM1M2_2(address1,422,   0,422,   0)
sleep(tl)
print('End Command motors with SpeedAccel_2')

# Command motor motion with Forward/BackwardM1/M2()
# Commands 0,1,4,5. Pg63-64
# Valid data range is 0-127. 127=full speed; 64=1/2 speed; 0=stop.
# ForwardM1(self,address,val)
roboclaw.ForwardM1(address1,64)
sleep(tl)
roboclaw.ForwardM1(address1,127)
sleep(tl)
roboclaw.ForwardM2(address1,10)
sleep(tl)
roboclaw.ForwardM2(address1,0) 
roboclaw.BackwardM2(address1,10)
sleep(tl)
print('End Command motors with Fwd/BkwdM1/M2')

# Read Encoders Commands 16,17,18,19. Pg86-87
E1M1_1 = roboclaw.ReadEncM1(address1)
E1M2_1 = roboclaw.ReadEncM2(address1)
S1M1_1 = roboclaw.ReadSpeedM1(address1)
S1M2_1 = roboclaw.ReadSpeedM2(address1)
print(E1M1_1, 'Read Encoder M1 address 1')
print(E1M2_1, 'Read Encoder M2 address 1')
print(S1M1_1, 'Read Speed M1 address 1')
print(S1M2_1, 'Read Speed M2 address 1')


'''Kill motors and print done'''
roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
roboclaw.ForwardM1(address2,0)
roboclaw.ForwardM2(address2,0)
sleep(ts)
print('Done')
quit()

''' 
Not reading speeds or encoder at all.
Why does only address2_M1 not go back to 0 RPM unitl commanded by ForwardM1(add2)
Updated Roboclaws to firmware version 4.2.1 on 5/24/2022. Fast response and reads encoders now.
6/21/22 Roboclaws not using some commands correctly.
SpeedAccelM1M2 speeds don't change. Either  on or off
Update 7/17/22 New Ras Pi 2GB with fresh install of NOOBS and completed all instructions to enable
UART on this Pi and get Pi and files up to date on this Pi. Instructions work correctly.
Something must have been changed on the other main Pi or the files
On the SpacePi2GB these functions all work as is to control the motor direction but
only forward/backwardM1/M2 controls the speed.
Try Duty control
'''


'''# List of all commands from roboclaw_3
## M1FORWARD = 0      ForwardM1(self,address,val)
## M1BACKWARD = 1     BackwardM1(self,address,val)
## SETMINMB = 2       SetMinVoltageMainBattery(self,address,val)
## SETMAXMB = 3       SetMaxVoltageMainBattery(self,address,val)
## M2FORWARD = 4      ForwardM2(self,address,val)
## M2BACKWARD = 5     BackwardM2(self,address,val)
## M17BIT = 6         ForwardBackwardM1(self,address,val)
## M27BIT = 7         ForwardBackwardM2(self,address,val)
# MIXEDFORWARD = 8    ForwardMixed(self,address,val)
# MIXEDBACKWARD = 9   BackwardMixed(self,address,val)
# MIXEDRIGHT = 10     TurnRightMixed(self,address,val)
# MIXEDLEFT = 11      TurnLeftMixed(self,address,val)
# MIXEDFB = 12        ForwardBackwardMixed(self,address,val)
# MIXEDLR = 13        LeftRightMixed(self,address,val)
## GETM1ENC = 16      ReadEncM1(self,address)
## GETM2ENC = 17      ReadEncM2(self,address)
## GETM1SPEED = 18    ReadSpeedM1(self,address)
## GETM2SPEED = 19    ReadSpeedM2(self,address)
## RESETENC = 20      ResetEncoders(self,address)
# GETVERSION = 21     ReadVersion(self,address)
# SETM1ENCCOUNT = 22  SetEncM1(self,address,cnt)
# SETM2ENCCOUNT = 23  SetEncM2(self,address,cnt)
## GETMBATT = 24      ReadMainBatteryVoltage(self,address)
## GETLBATT = 25      ReadLogicBatteryVoltage(self,address,)
# SETMINLB = 26       SetMinVoltageLogicBattery(self,address,val)
# SETMAXLB = 27       SetMaxVoltageLogicBattery(self,address,val)
# SETM1PID = 28       SetM1VelocityPID(self,address,p,i,d,qpps)
# SETM2PID = 29       SetM2VelocityPID(self,address,p,i,d,qpps)
# GETM1ISPEED = 30    ReadISpeedM1(self,address)
# GETM2ISPEED = 31    ReadISpeedM2(self,address)
# M1DUTY = 32         DutyM1(self,address,val)
# M2DUTY = 33         DutyM2(self,address,val)
# MIXEDDUTY = 34      DutyM1M2(self,address,m1,m2)
## M1SPEED = 35       SpeedM1(self,address,val)
## M2SPEED = 36       SpeedM2(self,address,val)
# MIXEDSPEED = 37     SpeedM1M2(self,address,m1,m2)
# M1SPEEDACCEL = 38   SpeedAccelM1(self,address,accel,speed)
# M2SPEEDACCEL = 39   SpeedAccelM2(self,address,accel,speed)
## MIXEDSPEEDACCEL = 40 SpeedAccelM1M2(self,address,accel,speed1,speed2)
## M1SPEEDDIST = 41    SpeedDistanceM1(self,address,speed,distance,buffer)   
## M2SPEEDDIST = 42    SpeedDistanceM2(self,address,speed,distance,buffer)
## MIXEDSPEEDDIST = 43 SpeedDistanceM1M2(self,address,speed1,distance1,speed2,distance2,buffer)
# M1SPEEDACCELDIST = 44 SpeedAccelDistanceM1(self,address,accel,speed,distance,buffer)
# M2SPEEDACCELDIST = 45 SpeedAccelDistanceM2(self,address,accel,speed,distance,buffer)
## MIXEDSPEEDACCELDIST = 46 SpeedAccelDistanceM1M2(self,address,accel,speed1,distance1,speed2,distance2,buffer)??? Typos?
# GETBUFFERS = 47     ReadBuffers(self,address)
# GETPWMS = 48        ReadPWMs(self,address)
# GETCURRENTS = 49    ReadCurrents(self,address)
# MIXEDSPEED2ACCEL = 50 SpeedAccelM1M2_2(self,address,accel1,speed1,accel2,speed2)
## MIXEDSPEED2ACCELDIST = 51 SpeedAccelDistanceM1M2_2(self,address,accel1,speed1,distance1,accel2,speed2,distance2,buffer)
## M1DUTYACCEL = 52    DutyAccelM1(self,address,accel,duty)
## M2DUTYACCEL = 53    DutyAccelM2(self,address,accel,duty)
## MIXEDDUTYACCEL = 54 DutyAccelM1M2(self,address,accel1,duty1,accel2,duty2)
## READM1PID = 55      ReadM1VelocityPID(self,address)
## READM2PID = 56      ReadM2VelocityPID(self,address)
## SETMAINVOLTAGES = 57 SetMainVoltages(self,address,min, max)
## SETLOGICVOLTAGES = 58 SetLogicVoltages(self,address,min, max)
## GETMINMAXMAINVOLTAGES = 59 ReadMinMaxMainVoltages(self,address)
## GETMINMAXLOGICVOLTAGES = 60 ReadMinMaxLogicVoltages(self,address)
# SETM1POSPID = 61    SetM1PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max)
# SETM2POSPID = 62    SetM2PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max)
# READM1POSPID = 63   ReadM1PositionPID(self,address)
# READM2POSPID = 64   ReadM2PositionPID(self,address)
# M1SPEEDACCELDECCELPOS = 65 SpeedAccelDeccelPositionM1(self,address,accel,speed,deccel,position,buffer)
# M2SPEEDACCELDECCELPOS = 66 SpeedAccelDeccelPositionM2(self,address,accel,speed,deccel,position,buffer)
# MIXEDSPEEDACCELDECCELPOS = 67 SpeedAccelDeccelPositionM1M2(self,address,accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer)
# SETM1DEFAULTACCEL = 68 SetM1DefaultAccel(self,address,accel)
# SETM2DEFAULTACCEL = 69 SetM2DefaultAccel(self,address,accel)
# SETPINFUNCTIONS = 74 SetPinFunctions(self,address,S3mode,S4mode,S5mode)
# GETPINFUNCTIONS = 75 ReadPinFunctions(self,address)
# SETDEADBAND = 76     SetDeadBand(self,address,min,max)
# GETDEADBAND = 77     GetDeadBand(self,address)
# RESTOREDEFAULTS = 80 RestoreDefaults(self,address)
# GETTEMP = 82        ReadTemp(self,address)
# GETTEMP2 = 83       ReadTemp2(self,address)
# GETERROR = 90       ReadError(self,address)
# GETENCODERMODE = 91 ReadEncoderModes(self,address)
# SETM1ENCODERMODE = 92 SetM1EncoderMode(self,address,mode)
# SETM2ENCODERMODE = 93 SetM2EncoderMode(self,address,mode)
# WRITENVM = 94       WriteNVM(self,address)
# READNVM = 95        ReadNVM(self,address)
# SETCONFIG = 98      SetConfig(self,address,config)
# GETCONFIG = 99      GetConfig(self,address)
# SETM1MAXCURRENT = 133 SetM1MaxCurrent(self,address,max)
# SETM2MAXCURRENT = 134 SetM2MaxCurrent(self,address,max)
# GETM1MAXCURRENT = 135 ReadM1MaxCurrent(self,address)
# GETM2MAXCURRENT = 136 ReadM2MaxCurrent(self,address)
# SETPWMMODE = 148     SetPWMMode(self,address,mode)
# GETPWMMODE = 149     ReadPWMMode(self,address)
# READEEPROM = 252     ReadEeprom(self,address,ee_address)
# WRITEEEPROM = 253    WriteEeprom(self,address,ee_address,ee_word)
# FLAGBOOTLOADER = 255 Open(self)
'''