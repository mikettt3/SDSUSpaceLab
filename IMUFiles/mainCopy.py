

#----------------------------------------------------------------------
# https://github.com/DavidEGrayson/minimu9-ahrs/blob/master/README.md
# This is where the code came from. "main.py" which you are in and
# "MinIMU_v5_pi.py" is the code from GitHub.
# Portions of this code were written by Caleb G. Teague in 2019
# Edits made by Luke Fernandes and Mike Stromecki
#----------------------------------------------------------------------

import csv
import numpy as np
import pymp #imports the pymp package to utulize OpenMP which allows you to use all cores of pi.
import smbus
import time
from datetime import datetime
from MinIMU_v5_pi import MinIMU_v5_pi

IMU = MinIMU_v5_pi()    # Initialize IMU communication

# Reads data from IMU and returns values in list
def imu(timeStamp):
    data = [timeStamp,"%03.5f " % IMU.readMagnetometer()[0],"%03.5f " % IMU.readMagnetometer()[1],"%03.5f " % IMU.readMagnetometer()[2],
        "%03.5f " % IMU.readAccelerometer()[0],"%03.5f " % IMU.readAccelerometer()[1],"%03.5f " % IMU.readAccelerometer()[2],
        "%03.5f " % IMU.readGyro()[0],"%03.5f " % IMU.readGyro()[1],"%03.5f " % IMU.readGyro()[2]]
    return data 

# Initiate tracking of moment of each direction on the IMU
IMU.trackAngle()
IMU.readMagnetometer()
IMU.readAccelerometer()
IMU.readGyro()

fileName="/home/pi/Desktop/Mike/SDSUSpaceLabADCS_Testbed/IMUFiles/IMUTest16Dec22.csv"
f = open(fileName, 'w', newline='')

for x in range(0, 100): # We can use for loop for now until the while loop works
    dataNow = datetime.now()    # create time stamp for each data point
    currentTime = dataNow.strftime("%H:%M:%S.%f")
    # Print from left to right mag, accel, gyro in X Y Z order
    print(imu(currentTime))

    # Output data into a csv file
    writer = csv.writer(f)
    writer.writerow(imu(currentTime))

    time.sleep(0.01)        
    
f.close()


# '''YAW = IMU.prevYaw[0]''' #This is for testing the yaw output
#             RoPiYa = IMU.prevAngle[0]
