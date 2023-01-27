# VN200Testing
# Runing this program in .../vnproglib/python
# In order to run these commands, you must first install the vnpy 
# library by cd into the approriate folder .../vproglib/python
# and running this command:
# sudo python3 setup.py install 
# You can find this instruction by first going to the index in the 
# vn python help folder. *Then you can go to the GettingStarted and 
# examples pages. Because that makes sens. Index first, then "getting 
# started".

from vnpy import *
from vnpy.libvncxx import VnSensor
# from vnpy.sensors import *
import time

s = VnSensor()
s.connect('/dev/ttyUSB0', 115200)

print(s.read_angular_rate_measurements())

# # print(s.read_model_number())
# # print(s.read_yaw_pitch_roll())
# ypr = s.read_yaw_pitch_roll()
# # print(ypr)
# print(ypr.x, ypr.y, ypr.z)
# # ''' print(ypr)
# # 	  print(ypr.x, ypr.y, ypr.z)
# # 	returns:
# # 	  vec3f([ -75.511, 4.101, -2.628 ])
# # 	  -75.51100158691406 4.10099983215332 -2.628000020980835'''

# Reads the Async Data Output Frequency register.
# Hz = s.read_async_data_output_frequency('/dev/ttyUSB0')
    # # return _libvncxx.VnSensor_read_async_data_output_frequency(self, *args)
# # NotImplementedError: Wrong number or type of arguments for overloaded function 'VnSensor_read_async_data_output_frequency'.
  # # Possible C/C++ prototypes are:
    # # vn::sensors::VnSensor::readAsyncDataOutputFrequency(uint8_t)
    # # vn::sensors::VnSensor::readAsyncDataOutputFrequency()
# Hz = s.readAsyncDataOutputFrequency('/dev/ttyUSB0')
# # AttributeError: 'VnSensor' object has no attribute 'readAsyncDataOutputFrequency'

'''
s.write_async_data_output_frequency(40) # Refresh rate (40 Hz) default for async data
Hz = s.read_async_data_output_frequency()
print(Hz)

80 Hz 27.9 - 30.2 ms recorded
50 Hz 27.9 - 31.6 ms recorded
40 	  27.9 - 32.5 ms
20    27.8 - 32.0 ms
10    27.9 - 30.5 ms
I guess data frequency doesn't matter
'''


# for i in range(0, 100):
	# print(time.time())
	# # Luke We need YPR, MAGXYZ, angular rates (omega_dot)
	# reg = s.read_yaw_pitch_roll_magnetic_acceleration_and_angular_rates()

	# print(reg.accel.x, reg.accel.y, reg.accel.z)
	# print(reg.gyro.x, reg.gyro.y, reg.gyro.z)
	# print(reg.mag.x, reg.mag.y, reg.mag.z)
	# print(reg.yaw_pitch_roll.x, reg.yaw_pitch_roll.y, reg.yaw_pitch_roll.z)	
	
	
	
	
	
	



# reg = vs.read_yaw_pitch_roll_magnetic_acceleration_and_angular_rates()
# dir(reg)
# ['__class__', '__del__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__',
# '__format__', '__ge__', '__getattr__','__getattribute__', '__gt__', '__hash__',
# '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__',
# '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__',
# '__subclasshook__', '__swig_destroy__', '__swig_getmethods__', '__swig_setmethods__'
# , '__weakref__', 'accel', 'gyro', 'mag', 'this', 'yaw_pitch_roll']
# reg.accel.x
# reg.accel.y
# reg.accel.z


# # What works so far in terminal. (prior to installign the package)
# # Installed minicom
# # sudo apt install minicom
# sudo minicom -o -D /dev/ttyUSB0 
# # streams data from VN200. Angles work.
# # 25 ms polling time delta
# # time, Y, P, R, Ax, Ay, Az, Mx, My, Mz


# # Configure VectorNav - turn off async messages on both ports TX1 and TX2
# $VNWRG,06,0,1*XX
# $VNWRG,06,0,2*XX

# # Configure VectorNav - set baud rates of port TX1
# $VNWRG,05,115200,1*XX

# # Configure VectorNav - set baud rates of port TX2
# $VNWRG,05,115200,2*XX

# # Configure VectorNav - Only turn on standard NMEA (GPRMC) messages
# $VNWRG,101,3,1,0,0,01*XX

s.disconnect()

''' Notes
file:///home/pi/Desktop/Mike/SDSUSpaceLabADCS_Testbed/vnproglib/python/help/vnpyapi.html
# Some select commands from the above page

class vnpy.CompositeData
Mega class that consolidates all data output types available from VectorNav sensors.

** class vnpy.EzAsyncData
Wraps class VnSensor and provides easy methods for accessing the sensor’s asynchronous data messages.
**

*class vnpy.VnSensor
Base class for interfacing with VectorNav sensors.
*

*** class vnpy.Attitude
Represents attitude of an object that may be represented in either yaw, pitch, roll, quaternion, or direction cosine matrix.
***

class vnpy.Packet
Wraps a VectorNav UART packet with associated tools for parsing.

class vnpy.vec3f
Represents a 3-component vector based on single-precision floats.
# (x, y, z)

class vnpy.vec4f
Represents a 4-component vector based on single-precision floats.
# (x, y, z, w)

class vnpy.mat3f
Represents a 3x3 matrix based on single-precision floats.



class vnpy.EzAsyncData
This class internally wraps a VnSensor object and provides easy methods for accessing asynchronous data from a VectorNav sensor. All low-level access functionality is still available through the property EzAsyncData.sensor.

Typically a call to EzAsyncData.connect() is used to create this object.

Class attributes are:

EzAsyncData.sensor
The wrapped VnSensor object which can be used to query the VectorNav sensor.

EzAsyncData.currentdata
The most recently cached asynchronous data processed.

Class methods are:

classmethod EzAsyncData.connect(portname, baudrate)
Immediately connects to a VectorNav sensor using the provided port parameters and returns an EzAsyncData object.

EzAsyncData.disconnect()
Disconnects from the VectorNav sensors.




VnSensor.magnetic_disturbance_present(disturbance_present)
Command to inform the VectorNav Sensor if there is a magnetic 
disturbance present. Set disturbance_present to True when a 
disturbance is present. Set it to False when the disturbance 
is gone.

VnSensor.read_attitude_quaternion()
Reads the Attitude Quaternion register.

VnSensor.read_acceleration_compensation()
Reads the Acceleration Compensation register.

VnSensor.read_acceleration_measurements()¶
Reads the Acceleration Measurements register.

VnSensor.read_angular_rate_measurements()
Reads the Angular Rate Measurements register.

VnSensor.read_async_data_output_frequency(port=None)
Reads the Async Data Output Frequency register.

VnSensor.read_async_data_output_type(port=None)
Reads the Async Data Output Type register.

VnSensor.read_imu_measurements()
Reads the IMU Measurements register.

VnSensor.read_ins_solution_ecef()
Reads the INS Solution - ECEF register.

VnSensor.read_ins_solution_lla()
Reads the INS Solution - LLA register.

VnSensor.read_ins_state_ecef()
Reads the INS State - ECEF register.

VnSensor.read_ins_state_lla()
Reads the INS State - LLA register.

VnSensor.read_magnetic_acceleration_and_angular_rates()
Reads the Magnetic, Acceleration and Angular Rates register.

VnSensor.read_magnetic_and_gravity_reference_vectors()
Reads the Magnetic and Gravity Reference Vectors register.

VnSensor.read_magnetic_measurements()
Reads the Magnetic Measurements register.

VnSensor.read_quaternion_magnetic_acceleration_and_angular_rates()
Reads the Quaternion, Magnetic, Acceleration and Angular Rates register.

VnSensor.read_yaw_pitch_roll()
Reads the Yaw Pitch Roll register.

VnSensor.read_yaw_pitch_roll_magnetic_acceleration_and_angular_rates()
Reads the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.

VnSensor.read_yaw_pitch_roll_true_body_acceleration_and_angular_rates_register()
Reads the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.

VnSensor.read_yaw_pitch_roll_true_inertial_acceleration_and_angular_rates_register()
Reads the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.

VnSensor.reset()
Issues a Reset command to the VectorNav sensor.

VnSensor.tare()
Issues a tare command to the VectorNav Sensor.


class vnpy.Attitude
Represents an orientation of an object regardless of the underlying 
data type (i.e. yaw, pitch, roll, quaternion, direction cosine matrix). 
Allows easy conversion between these types.

Class attributes are:

Attitude.ypr_degs
Yaw, pitch, roll representation in degrees.

Attitude.ypr_rads
Yaw, pitch, roll representation in radians.

Attitude.quat
Quaternion representation.

Attitude.dcm
Direction cosine matrix representation.

Class methods are:

classmethod Attitude.no_rotation()¶
Represents a object with no rotation.

classmethod Attitude.from_quat(quat)
Creates an Attitude object from the provided quaternion representation.

classmethod Attitude.from_ypr_degs(ypr)
Creates an Attitude object from the provided yaw, pitch, roll representation in degrees.

classmethod Attitude.from_ypr_rads(ypr)
Creates an Attitude object from the provided yaw, pitch, roll representation in radians.

classmethod Attitude.from_dcm(dcm)
Creates an Attitude object from the provided direction cosine matrix representation.


'''
