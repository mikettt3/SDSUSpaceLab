#from roboclaw import Roboclaw
# from roboclaw_python 
from roboclaw_3 import Roboclaw
from time import sleep

# Current path to roboclaw_3.py
# /home/pi/raspberry_pi_packet_serial

# Remember to run through the terminal all the commands in UARTCommands.txt.
# Apparently UART isn't enebled correctly for the RoboClaws out of the box.

# Original file code
# if __name__ == "__main__":
#     
#     address = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
#     roboclaw = Roboclaw("/dev/ttyS0", 38400)
#     roboclaw.Open()
#     
#     while True:
#         
#         roboclaw.ForwardM1(address,64)
#         sleep(2)
#         roboclaw.ForwardM1(address,0)
#         sleep(2)
#         
#         roboclaw.ForwardM2(address, 64)
#         sleep(2)
#         roboclaw.ForwardM2(address,0)
#         sleep(2)
       
############################################################################
############################################################################
############################################################################

#Test out forwards and backwards operation of a single roboclaw
address1 = 0x80 #128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
address2 = 0x81
roboclaw = Roboclaw("/dev/ttyS0", 38400) # Create the RoboClaw object, passing the serial port and baudrate
roboclaw.Open() # Call the Open() function on the RoboClaw object and start communication

tl = 2 # long pause time
ts = 0.5 # short pause time

print('Test Start')

roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
# roboclaw.ForwardM1(address2,0)
# roboclaw.ForwardM2(address2,0)
sleep(2)

roboclaw.SpeedAccelM1M2_2(address1,2000, 500,2000,0)
sleep(0.5)
roboclaw.SpeedAccelM1M2_2(address1,2000,-500,2000,500)
sleep(0.5)
roboclaw.SpeedAccelM1M2_2(address1,2000,   0,2000,0)
sleep(0.5)
# roboclaw.SpeedAccelM1M2_2(address2,2000, 500,2000,0)
# roboclaw.SpeedAccelM1M2_2(address2,2000, 0,2000,500)
# roboclaw.SpeedAccelM1M2_2(address2,2000, 0,2000,0)


roboclaw.ForwardM1(address1,0)
roboclaw.ForwardM2(address1,0)
# roboclaw.ForwardM1(address2,0)
# roboclaw.ForwardM2(address2,0)
sleep(2)

# SpeedAccelM1M2_2(self,address,accel1,speed1(RPM),accel2,speed2(RPM)):
roboclaw.SpeedAccelM1M2_2(address1,1000,500,1000,2220) #max rpm ~8220 @~16VDC
# roboclaw.SpeedAccelM1M2_2(address2,1000,500,1000,500)
sleep(2)

S1M2_1=roboclaw.ReadSpeedM2(address1) # Speed Roboclaw 1, motor 2, _instance#
# S2M1_1=roboclaw.ReadSpeedM1(address2)
# 	enc1 = rc.ReadEncM1(address)
# 	enc2 = rc.ReadEncM2(address)
# 	speed1 = rc.ReadSpeedM1(address)
# 	speed2 = rc.ReadSpeedM2(address)
print(S1M2_1)
# print(S2M1_1)

roboclaw.SpeedAccelM1M2_2(address1,1000,-500,2000,0)
# roboclaw.SpeedAccelM1M2_2(address2,1000,-500,2000,0)
sleep(1)
roboclaw.SpeedAccelM1M2_2(address1,1000,0,50,-1200)
# roboclaw.SpeedAccelM1M2_2(address2,1000,0,50,-1200)
sleep(1)

S1M2_2=roboclaw.ReadSpeedM2(address1) # Speed Roboclaw 1, motor 2, _instance#
# S2M1_2=roboclaw.ReadSpeedM1(address2)
print(S1M2_2)
# print(S2M1_2)

# 	rc.SpeedAccelDistanceM1(address,48000,-12000,46500,1);
# 	rc.SpeedAccelDistanceM2(address,48000,12000,46500,1);
# 	rc.SpeedAccelDistanceM1(address,48000,0,0,0);  #distance travelled is v*v/2a = 12000*12000/2*48000 = 1500
# 	rc.SpeedAccelDistanceM2(address,48000,0,0,0);  #that makes the total move in one direction 48000
# 	buffers = (0,0,0)
# 	while(buffers[1]!=0x80 and buffers[2]!=0x80):	#Loop until distance command has completed
# 		print "Buffers: ",
# 		print buffers[1],
# 		print " ",
# 		print buffers[2]
# 		displayspeed()
# 		buffers = rc.ReadBuffers(address)
#   
# 	time.sleep(1);  #When no second command is given the motors will automatically slow down to 0 which takes 1 second


roboclaw.ForwardM1(address1,0)
# roboclaw.ForwardM1(address2,0)
roboclaw.ForwardM2(address1,0)
# roboclaw.ForwardM2(address2,0)

print('Done')
# sleep(2)

# Cmd(4)
# ReadM2VelocityPID(self,address)
# SetM2VelocityPID(address,p,i,d,qpps)
# DutyM2(self,address,val):
# SpeedAccelM1M2_2(self,address,accel1,speed1(RPM),accel2,speed2(RPM)):
# SM2a=roboclaw.ReadISpeedM2(address)
# print(SM2a)
# roboclaw.ForwardM2(address,80) (0-127)
# roboclaw.ForwardBackwardM2(address,127) # Full speed fwd
# roboclaw.ForwardBackwardM2(address,64)  # 0
# roboclaw.ForwardBackwardM2(address,0)   # Full speed rev


# Change line 849 in roboclaw_3.py from
# 		return self._write4S44S4(address,self.Cmd.MIXEDSPEED2ACCEL,accel,speed1,accel2,speed2)
# to
# 		return self._write4S44S4(address,self.Cmd.MIXEDSPEED2ACCEL,accel1,speed1,accel2,speed2)