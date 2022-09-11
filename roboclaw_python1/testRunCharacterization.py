from _ast import For
from roboclaw_3 import Roboclaw
from time import sleep

class Base:
    
    def start(self):
        
        '''Kill motors before start'''
        RC.ResetEncoders(address1)
        RC.ResetEncoders(address2)
        RC.ForwardM1(address1, 0)
        RC.ForwardM1(address2, 0)
        RC.ForwardM2(address1, 0)
        RC.ForwardM2(address2, 0)
        sleep(1)
    
    
    def stop(self):
        
        '''Kill motors and print done'''
        RC.ForwardM1(address1, 0)
        RC.ForwardM1(address2, 0)
        RC.ForwardM2(address1, 0)
        RC.ForwardM2(address2, 0)
        sleep(1)
        
        
    def results(self):
        
        '''Printing the read data commands'''
        # address 1 stuff
        print()
        print("Address 1")
        print()
        print(RC.ReadNVM(address1), " -> ReadNVM") # 95 [Enc1Mode, Enc2Mode, CRC(2 bytes)] - Read all settings from non-volatile memory.
        print(RC.ReadEncM1(address1), " -> ReadEncM1") # 16 [enc reading, status(unclear), CRC]
        print(RC.ReadEncM2(address1), " -> ReadEncM2") # 17
        print(RC.ReadSpeedM1(address1), " -> ReadSpeedM1, M1") # Command 18 Filtered
        print(RC.ReadSpeedM2(address1), " -> ReadSpeedM2, M2") # Command 19 Filtered
        print(RC.ReadError(address1), " -> ReadError") # 90
        print(RC.ReadTemp(address1), "  -> ReadTemp1") # 82
        print(RC.ReadTemp2(address1), " -> ReadTemp2") # 83
        print(RC.GetDeadBand(address1), " -> GetDeadBand") # 77 
        print(RC.ReadCurrents(address1), " -> ReadCurrents") # 49 [Reverse, SForward, CRC(2 bytes)]
        print(RC.ReadMainBatteryVoltage(address1), " -> ReadMainBatteryVoltage x10")
        
        # address 2 stuff
        print()
        print("Address 2")
        print()
        print(RC.ReadNVM(address2), " -> ReadNVM") # 95 [Enc1Mode, Enc2Mode, CRC(2 bytes)]
        print(RC.ReadEncM1(address2), " -> ReadEncM1") # 16 [enc reading, status(unclear), CRC]
        print(RC.ReadEncM2(address2), " -> ReadEncM2") # 17
        print(RC.ReadSpeedM1(address2), " -> ReadSpeedM1, M1") # Command 18 Filtered
        print(RC.ReadSpeedM2(address2), " -> ReadSpeedM2, M2") # Command 19 Filtered
        print(RC.ReadError(address2), " -> ReadError") # 90
        print(RC.ReadTemp(address2), "  -> ReadTemp1") # 82
        print(RC.ReadTemp2(address2), " -> ReadTemp2") # 83
        print(RC.ReadCurrents(address2), " -> ReadCurrents") # 49 [Reverse, SForward, CRC(2 bytes)]
        print(RC.ReadCurrents(address2), " -> ReadCurrents") # 49
        print(RC.ReadMainBatteryVoltage(address2), " -> ReadMainBatteryVoltage x10\n \n")
        
        print()
        
    
if __name__ == '__main__':
    
    # Creating Objects for variables, classes and methods
    basic = Base()

    # 128 in hex == 0x80. 129 == 0x81, 130==0x82, 131==0x83
    address1 = 0x80 
    address2 = 0x81
    
    # Create Roboclaw object and Connect it to roboclaw
    RC = Roboclaw("/dev/ttyS0", 38400)
    RC.Open()
    
    # Intializing the starting sequence
    print('Starting Test\n')
    basic.start()
    
#________________________________________________________________________________________________________
    '''Setters'''
    
    # SetM1PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max) #61
    # SetM2PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max) #62
#     RC.SetM1PositionPID(address1,4.6,0,0,kimax,deadzone,min,max)
#     RC.SetM2PositionPID(address1,kp,ki,kd,kimax,deadzone,min,max)
#     RC.SetM1PositionPID(address2,4.6,0,0,kimax,deadzone,min,max)
#     RC.SetM2PositionPID(address2,kp,ki,kd,kimax,deadzone,min,max)
    
    # SetM1VelocityPID(self,address,p,i,d,qpps) #28
    # SetM2VelocityPID(self,address,p,i,d,qpps) #29
    RC.SetM1VelocityPID(address1,3,1,0,7125)
    RC.SetM2VelocityPID(address1,3,1,0,7125)
    RC.SetM1VelocityPID(address2,3,1,0,7125)
    RC.SetM2VelocityPID(address2,3,1,0,7125)
    
#________________________________________________________________________________________________________
    '''Commands'''
    
    # ForwardM1(self,address,val) # 0
    # BackwardM1(self,address,val) # 1
    RC.ForwardM1(address1,127)
    sleep(4)
    
    basic.results()
    sleep(2)
    
    ''' Find max throttle
    Run all motors together. Increase velocity from 0 to maximum allowed 
    by Roboclaws in steps of 10%. Print at each increase. Note when motors 
    are no longer accelerating.
    Can use IR tachometer in blue case for these tests or encoders reported 
    speed. This test is just to find max throttle setting, not what that 
    RPM actually is. Can also use a speed command utilizing 0-127 rather 
    than QPPS structure.'''
    # loop
    #   Throttle = 0. RPM Observed = 0
    #   Throttle = 10%. sleep(1) RPM Observed = for example 100 RPM
    #   Throttle = 20%. sleep(1) RPM Observed = 200 ...
    #   Throttle = 50%. sleep(1) RPM Observed = 2000
    #   Throttle = 60%. sleep(1) RPM Observed = 2010
    #   Throttle = 70%. sleep(1) RPM Observed = 2010
    # Max throttle somewhere around 60% of max RC throttle. Refine and 
    # continue until exact value is found.
    
    
    
    
    
    
    ''' Find deadzones from stopped
    All motors together. Increase throttle from 0 to 15 % of maximum.
    Note when all motors are first moving smoothly.'''
    # All motors increment from stopped to 20% by 1. Use 0-127 structure 
    # command. Should be around 7.
    # loop
    #   speed(0)
    #   sleep(1)
    # (no movement)
    # ...
    # loop
    #   speed(6)
    #   sleep(3)
    # (Chunky movement)
    # loop
    #   speed(7)
    #   sleep(3)
    # (Smooth movement)
    
    
    
    
    
    ''' Find deadzones from moving
    All motors together. Decrease throttle from 15 to 0 % of maximum.
    Note when first motor stops moving smoothly.'''
    # All motors increment from stopped to 20% by 1. Use 0-127 structure 
    # command. Should be around 7.
    # loop
    #   speed(20)
    #   sleep(1)
    # (Moving fine)
    # ...
    # loop
    #   speed(6)
    #   sleep(3)
    # (Chunky movement and/or full stop)

    
    
    
    ''' Find throttle to output RPM relationship
    For each motor individually. Increase in 10% steps, take 3 stabel 
    tachometer readings and write them down. Enter them into the code and 
    then have it move on if you can. Also note QPPS velocity readings 
    during this time and have the program self record them.'''
    # 10% You enter [100, 110, 105] [enter] 
    # --> Store A1M1_10Velocity taken every 0.5 seconds
    # --> Store A1M1_10TotalDist taken at the end. No need to rezero.
    # Code steps up to 20%. [200, 195, 205] [enter]
    # --> Store A1M1_20Velocity
    # --> Store A1M1_20TotalDist
    # ...
    # 100% [2200 2190 2210] [enter]
    # --> Store A1M1_100Velocity
    # --> Store A1M1_100TotalDist
    # Next motor. Repeat.
    # We will plot these later and see if the relationship is linear.
    
    
    
#________________________________________________________________________________________________________
    # Killing motors before turning off
    basic.stop()
    print('\nDone')
    quit()
    
    
    
    
    
    
