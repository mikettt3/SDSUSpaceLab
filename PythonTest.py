import time
from time import sleep
from datetime import datetime
from datetime import timedelta
import csv
from MinIMU_v5_pi import MinIMU_v5_pi

# minute = datetime.min
# print(minute)

# t1 = datetime.now()
# sleep(2)
# t2 = datetime.now()
# print(t2-t1)
# t1 = t1.strftime("%z")
# print(t1)


IMU = MinIMU_v5_pi()

IMU.trackAngle()	# Line 421 in MinIMU_v5_pi.py
'''Creates another thread which calls updateAngle every 4ms (250Hz)
    to track the current roll, pitch, and yaw'''
IMU.updateAngle()
IMU.readGyro()
IMU.readAccelerometer()
IMU.readMagnetometer()

def imu(timeStamp):
    data = [timeStamp,"%03.4f " % IMU.readMagnetometer()[0],
        "%03.4f " % IMU.readMagnetometer()[1],"%03.4f " % IMU.readMagnetometer()[2],
        "%03.4f " % IMU.readAccelerometer()[0],"%03.4f " % IMU.readAccelerometer()[1],
        "%03.4f " % IMU.readAccelerometer()[2],"%03.4f " % IMU.readGyro()[0],
        "%03.4f " % IMU.readGyro()[1],"%03.4f " % IMU.readGyro()[2],
        "%03.4f " % IMU.updateAngle()[0],"%03.4f " % IMU.updateAngle()[1],
        "%03.4f " % IMU.updateAngle()[2] ]
#     angle = ["%03.4f " % IMU.updateAngle()[0],
#         "%03.4f " % IMU.updateAngle()[1],"%03.4f " % IMU.updateAngle()[2]]
#     return data, angle
    return data

timeStamp2 = datetime.now().strftime("%H%M%S.%s")[0:6]
imuR = imu(timeStamp2)   # Read IMU

f=open('MotionTest22Dec20_1718.csv','w',newline='')
csv.writer(f).writerow(imuR)

f.close()












    # K_rxn = 0.1
    # if yawGyroErr < 0:
    #     DriveDir = 90   # Drive forward
    # elif yawGyroErr > 0:
    #     DriveDir = 0    # Drive reverse
    # elif yawGyroErr == 0:
    #     DriveDir = 64   # Stop motors
    # else:
    #     print('Error in DriveWheels.')

    # '''Need way to scale gyro error to motor drive values'''
    # yawDrive = yawGyroErr/150
    # if yawDrive < 0:
        # DriveDir = 90   # Drive forward
    # elif yawDrive > 0:
        # DriveDir = 30    # Drive reverse
    # elif yawGyroErr == 0:
        # DriveDir = 64   # Stop motors
    # else:
        # print('Error in DriveWheels.')
        # print(yawGyroErr)
        # print(yawDrive)

    # mDrive = round(K_rxn*DriveDir*yawDrive)
    # # Stop drive for deadzones
    # # Unreliable motor actuation below this value (Deadzone)
    # # Limit drive to allowable Roboclaw input values
    # if (mDrive >=58) and (mDrive <= 70):
        # mDrive = 64
    # elif mDrive > 127:
        # mDrive = 90
    # elif mDrive < 0:
        # mDrive = 30
    # elif ((mDrive>=0)and(mDrive<58))or((mDrive>70)and(mDrive<=127)):
        # # This is or normal operation.
        # return
    # else:
        # print('Error in mDrive limit loop')
        # print(mDrive)




# while (abs(imu(timeStamp)[1:9]-ZSA))>(abs(3*ZSAstd)):
# while True:
#     imuR = float(imu(timeStamp))   # Read IMU
#     if (abs(imuR-ZSA))>=(abs(6*ZSAstd)):
#         # Correct attitude
#         attErr = imuR[1:9]-ZSA  # Estimate error
#         yawGyroErr = attErr(9)
#         yawMagErr  = attErr(2)
#         # Send error amount and direction to drive rxn wheels or mtqs
#         DriveWheels(yawGyroErr)
#         # DriveMTQ1(yawMagErr)
#     elif (abs(imuR-ZSA))<(abs(6*ZSAstd)):
#         yawGyroErr = int(0)
#         yawMagErr  = int(0)
#     else:
#         print('Error in while loop')

# start_time = time.time()
# main()
# print("--- %s seconds ---" % (time.time() - start_time))
# timeNow = datetime.now()
# timeNow = timeNow.strftime("%H%M%S.%s")
# timeNow = float(timeNow[0:10])
# timeCheck = round((timeNow - timeStamp))
# if round((datetime.now() - timeStart)) > 001500
#     print('Check runtime')
#     sleep(tl)
#     if time.time() -
#     if round((datetime.now() - timeStart)) > 001500
#         print('Check runtime')
#         sleep(tl)
