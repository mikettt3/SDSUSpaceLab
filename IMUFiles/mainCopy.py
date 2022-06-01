#!/usr/bin/
#This code was written by Caleb G. Teague in 2019

'''install "pip install pymp-pypi" in the terminal/command window'''
'''If you running on a new Rasberry Pi, download this ^^^^ '''
#Edits made by Luke Fernandes
#----------------------------------------------------------------------------------------
#https://github.com/DavidEGrayson/minimu9-ahrs/blob/master/README.md
#This is where the code came from. "main.py" which you are in and
#"MinIMU_v5_pi.py" is the code from GitHub!!!
#----------------------------------------------------------------------------------------

import csv
import numpy as np #importing package so that we can save output into a csv file
import pymp #imports the pymp package to utulize OpenMP which allows you to use all cores of pi.
import smbus
import time
from datetime import datetime
from MinIMU_v5_pi import MinIMU_v5_pi

#-------------------------------------------
#Majority of things in
'''green'''
#are commented out code that can be used
#-------------------------------------------

IMU = MinIMU_v5_pi()    # Initialize IMU communication

# Reads data from IMU and returns values in list
def imu(timeStamp):
    data = [timeStamp,"%03.5f " % IMU.readMagnetometer()[0],"%03.5f " % IMU.readMagnetometer()[1],"%03.5f " % IMU.readMagnetometer()[2],
        "%03.5f " % IMU.readAccelerometer()[0],"%03.5f " % IMU.readAccelerometer()[1],"%03.5f " % IMU.readAccelerometer()[2],
        "%03.5f " % IMU.readGyro()[0],"%03.5f " % IMU.readGyro()[1],"%03.5f " % IMU.readGyro()[2]]
    return data 

def main():
#---------------Creating a reference object to the supported MinIMU_v5_pi class------------

#---------------Initiate tracking of moment of each direction on the IMU--------------------
    IMU.trackAngle()
    IMU.readGyro()
    IMU.readAccelerometer()
    IMU.readMagnetometer()

    with pymp.Parallel(4) as p:
        while True:
            fileNow = datetime.now()    # create time stamp for the file the data will be saved to
            fileStamp = fileNow.strftime("%H:%M:%S")
    
            for x in p.range(0, 100000): #We can use for loop for now until the while loop works
                dataNow = datetime.now()    # create time stamp for each data point
                currentTime = dataNow.strftime("%H:%M:%S.%f")
                f = open("/home/pi/Desktop/IMU/IMU FILES THAT JARRED COPIED/IMU_output/IMU_" + str(fileStamp) + ".csv", 'a', newline='')

#                 f = open("/home/pi/Desktop/IMU/IMU FILES THAT JARRED COPIED/IMU.csv", 'a', newline='') # CHANGED - location and format
#                 f.open()
                writer = csv.writer(f)
# '''YAW = IMU.prevYaw[0]''' #This is for testing the yaw output
#             RoPiYa = IMU.prevAngle[0]

#--------------Prints from left to right magnet, accel, gyro in X Y Z in order!!!----------
                print(imu(currentTime))
    
    #----------------------Outputs data into a csv file----------------------------------------
                writer.writerow(imu(currentTime))

#----------------------Allocates time to sleep in between readings-------------------------
#                 f.close() # close necessary in for loop as it saves data without having to break loop
                time.sleep(0.1)        
                f.close()

if __name__ == "__main__":
    main()


