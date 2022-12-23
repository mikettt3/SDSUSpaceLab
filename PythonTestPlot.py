import numpy as np
import matplotlib.pyplot as plt

# yawGyroErr = np.array[-200:200:1]
yawGyroErr  = np.linspace(-200,200,4001)
# mDrive = np.empty((1,4001))
mDrive  = np.linspace(-1,1,4001)
mDrive1 = np.linspace(-1,1,4001)
mDrive2 = np.linspace(-1,1,4001)

for i in range(4001):
    if yawGyroErr[i] <= -150:
        mDrive[i] = 127
    elif (yawGyroErr[i] > -150) and (yawGyroErr[i] <= -16):
        mDrive[i] = 127-((yawGyroErr[i]+150)*64/150)
    elif (yawGyroErr[i] > -16) and (yawGyroErr[i] < 16):
        mDrive[i] = 64
    elif (yawGyroErr[i] < 150) and (yawGyroErr[i] >= 16):
        mDrive[i] = 64-((yawGyroErr[i])*64/150)
    elif yawGyroErr[i] >= 150:
        mDrive[i] = 0
    else:
        print('Error in YGE loop');
        
for i in range(4001):
    if yawGyroErr[i] <= -150:
        mDrive2[i] = 127
    elif (yawGyroErr[i] > -150) and (yawGyroErr[i] <= -16):
        mDrive2[i] = 63-(yawGyroErr[i]*64/150)
    elif (yawGyroErr[i] > -16) and (yawGyroErr[i] < 16):
        mDrive2[i] = 64
    elif (yawGyroErr[i] < 150) and (yawGyroErr[i] >= 16):
        mDrive2[i] = 64-((yawGyroErr[i])*64/150)
    elif yawGyroErr[i] >= 150:
        mDrive2[i] = 0
    else:
        print('Error in YGE loop');
        
for ii in range(4001):
    if yawGyroErr[ii] <= -150:
        mDrive1[ii] = 0
    elif (yawGyroErr[ii] > -150) and (yawGyroErr[ii] <= -16):
        mDrive1[ii] = (yawGyroErr[ii]*64/150) + 64
    elif (yawGyroErr[ii] > -16) and (yawGyroErr[ii] < 16):
        mDrive1[ii] = 64
    elif (yawGyroErr[ii] >= 16) and (yawGyroErr[ii] < 150):
        mDrive1[ii] = (yawGyroErr[ii]*64/150) + 63
    elif yawGyroErr[ii] >= 150:
        mDrive1[ii] = 127
    else:
        print('Error in YGE loop');
        
# for ii in range(4001):
#     if yawGyroErr[ii] <= -150:
#         mDrive2[ii] = 0
#     elif (yawGyroErr[ii] > -150) and (yawGyroErr[ii] <= -16):
#         mDrive2[ii] = (yawGyroErr[ii]+150)*64/150
#     elif (yawGyroErr[ii] > -16) and (yawGyroErr[ii] < 16):
#         mDrive2[ii] = 64
#     elif (yawGyroErr[ii] >= 16) and (yawGyroErr[ii] < 150):
#         mDrive2[ii] = (yawGyroErr[ii]*64/150) + 63
#     elif yawGyroErr[ii] >= 150:
#         mDrive2[ii] = 127
#     else:
#         print('Error in YGE loop');        


plt.figure(1)
plt.plot(yawGyroErr,mDrive, yawGyroErr,mDrive2) # , yawGyroErr,mDrive1
plt.show()
