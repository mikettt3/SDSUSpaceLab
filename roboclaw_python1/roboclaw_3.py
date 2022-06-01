import random
import serial
import struct
import time

class Roboclaw:
	'Roboclaw Interface Class'
	
	def __init__(self, comport, rate, timeout=0.01, retries=3):
		self.comport = comport
		self.rate = rate
		self.timeout = timeout;
		self._trystimeout = retries
		self._crc = 0;

	#Command Enums
	class Cmd():
		M1FORWARD = 0      # ForwardM1(self,address,val)       Line 653
		M1BACKWARD = 1     # BackwardM1(self,address,val)
		SETMINMB = 2       # SetMinVoltageMainBattery(self,address,val)
		SETMAXMB = 3       # SetMaxVoltageMainBattery(self,address,val)
		M2FORWARD = 4      # ForwardM2(self,address,val)
		M2BACKWARD = 5     # BackwardM2(self,address,val)
		M17BIT = 6         # ForwardBackwardM1(self,address,val)
		M27BIT = 7         # ForwardBackwardM2(self,address,val)
		MIXEDFORWARD = 8   # ForwardMixed(self,address,val)
		MIXEDBACKWARD = 9  # BackwardMixed(self,address,val)   Line 680
		MIXEDRIGHT = 10    # TurnRightMixed(self,address,val)
		MIXEDLEFT = 11     # TurnLeftMixed(self,address,val)
		MIXEDFB = 12       # ForwardBackwardMixed(self,address,val)
		MIXEDLR = 13       # LeftRightMixed(self,address,val)
		GETM1ENC = 16      # ReadEncM1(self,address)
		GETM2ENC = 17      # ReadEncM2(self,address)
		GETM1SPEED = 18    # ReadSpeedM1(self,address)
		GETM2SPEED = 19    # ReadSpeedM2(self,address)         Line 704
		RESETENC = 20      # ResetEncoders(self,address)
		GETVERSION = 21    # ReadVersion(self,address)         Line 710
		SETM1ENCCOUNT = 22 # SetEncM1(self,address,cnt)
		SETM2ENCCOUNT = 23 # SetEncM2(self,address,cnt)
		GETMBATT = 24      # ReadMainBatteryVoltage(self,address) Line 747
		GETLBATT = 25      # ReadLogicBatteryVoltage(self,address,)
		SETMINLB = 26      # SetMinVoltageLogicBattery(self,address,val)
		SETMAXLB = 27      # SetMaxVoltageLogicBattery(self,address,val)
		SETM1PID = 28      # SetM1VelocityPID(self,address,p,i,d,qpps)
		SETM2PID = 29      # SetM2VelocityPID(self,address,p,i,d,qpps)    Line 763
		GETM1ISPEED = 30   # ReadISpeedM1(self,address)
		GETM2ISPEED = 31   # ReadISpeedM2(self,address)
		M1DUTY = 32        # DutyM1(self,address,val)
		M2DUTY = 33        # DutyM2(self,address,val)
		MIXEDDUTY = 34     # DutyM1M2(self,address,m1,m2)
		M1SPEED = 35       # SpeedM1(self,address,val)
		M2SPEED = 36       # SpeedM2(self,address,val)
		MIXEDSPEED = 37    # SpeedM1M2(self,address,m1,m2)
		M1SPEEDACCEL = 38  # SpeedAccelM1(self,address,accel,speed)
		M2SPEEDACCEL = 39  # SpeedAccelM2(self,address,accel,speed)       Line 794
		MIXEDSPEEDACCEL = 40 # SpeedAccelM1M2(self,address,accel,speed1,speed2)
		M1SPEEDDIST = 41     # SpeedDistanceM1(self,address,speed,distance,buffer)   
		M2SPEEDDIST = 42     # SpeedDistanceM2(self,address,speed,distance,buffer)
		MIXEDSPEEDDIST = 43  # SpeedDistanceM1M2(self,address,speed1,distance1,speed2,distance2,buffer)
		M1SPEEDACCELDIST = 44 # SpeedAccelDistanceM1(self,address,accel,speed,distance,buffer)
		M2SPEEDACCELDIST = 45 # SpeedAccelDistanceM2(self,address,accel,speed,distance,buffer)
		MIXEDSPEEDACCELDIST = 46 # SpeedAccelDistanceM1M2(self,address,accel,speed1,distance1,speed2,distance2,buffer)
		GETBUFFERS = 47    # ReadBuffers(self,address)
		GETPWMS = 48       # ReadPWMs(self,address)
		GETCURRENTS = 49   # ReadCurrents(self,address)                   Line 836
		MIXEDSPEED2ACCEL = 50 # SpeedAccelM1M2_2(self,address,accel1,speed1,accel2,speed2) Line 848
		MIXEDSPEED2ACCELDIST = 51 # SpeedAccelDistanceM1M2_2(self,address,accel1,speed1,distance1,accel2,speed2,distance2,buffer)
		M1DUTYACCEL = 52    # DutyAccelM1(self,address,accel,duty)
		M2DUTYACCEL = 53    # DutyAccelM2(self,address,accel,duty)
		MIXEDDUTYACCEL = 54 # DutyAccelM1M2(self,address,accel1,duty1,accel2,duty2)
		READM1PID = 55      # ReadM1VelocityPID(self,address)
		READM2PID = 56      # ReadM2VelocityPID(self,address)
		SETMAINVOLTAGES = 57 # SetMainVoltages(self,address,min, max)
		SETLOGICVOLTAGES = 58 # SetLogicVoltages(self,address,min, max)
		GETMINMAXMAINVOLTAGES = 59 # ReadMinMaxMainVoltages(self,address)
		GETMINMAXLOGICVOLTAGES = 60 # ReadMinMaxLogicVoltages(self,address)
		SETM1POSPID = 61   # SetM1PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max)
		SETM2POSPID = 62   # SetM2PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max)
		READM1POSPID = 63  # ReadM1PositionPID(self,address)
		READM2POSPID = 64  # ReadM2PositionPID(self,address)
		M1SPEEDACCELDECCELPOS = 65 # SpeedAccelDeccelPositionM1(self,address,accel,speed,deccel,position,buffer)
		M2SPEEDACCELDECCELPOS = 66 # SpeedAccelDeccelPositionM2(self,address,accel,speed,deccel,position,buffer)
		MIXEDSPEEDACCELDECCELPOS = 67 # SpeedAccelDeccelPositionM1M2(self,address,accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer)
		SETM1DEFAULTACCEL = 68 # SetM1DefaultAccel(self,address,accel)
		SETM2DEFAULTACCEL = 69 # SetM2DefaultAccel(self,address,accel)
		SETPINFUNCTIONS = 74 # SetPinFunctions(self,address,S3mode,S4mode,S5mode)
		GETPINFUNCTIONS = 75 # ReadPinFunctions(self,address)
		SETDEADBAND = 76   # SetDeadBand(self,address,min,max)
		GETDEADBAND = 77   # GetDeadBand(self,address)
		RESTOREDEFAULTS = 80 # RestoreDefaults(self,address)
		GETTEMP = 82       # ReadTemp(self,address)
		GETTEMP2 = 83      # ReadTemp2(self,address)
		GETERROR = 90      # ReadError(self,address)
		GETENCODERMODE = 91 # ReadEncoderModes(self,address)
		SETM1ENCODERMODE = 92 # SetM1EncoderMode(self,address,mode)
		SETM2ENCODERMODE = 93 # SetM2EncoderMode(self,address,mode)
		WRITENVM = 94      # WriteNVM(self,address)
		READNVM = 95       # ReadNVM(self,address)
		SETCONFIG = 98     # SetConfig(self,address,config)
		GETCONFIG = 99     # GetConfig(self,address)
		SETM1MAXCURRENT = 133 # SetM1MaxCurrent(self,address,max)
		SETM2MAXCURRENT = 134 # SetM2MaxCurrent(self,address,max)
		GETM1MAXCURRENT = 135 # ReadM1MaxCurrent(self,address)
		GETM2MAXCURRENT = 136 # ReadM2MaxCurrent(self,address)
		SETPWMMODE = 148   # SetPWMMode(self,address,mode)
		GETPWMMODE = 149   # ReadPWMMode(self,address)
		READEEPROM = 252   # ReadEeprom(self,address,ee_address)
		WRITEEEPROM = 253  # WriteEeprom(self,address,ee_address,ee_word)
		FLAGBOOTLOADER = 255 # Open(self)
			
	#Private Functions
	def crc_clear(self):
		self._crc = 0
		return
		
	def crc_update(self,data):
		self._crc = self._crc ^ (data << 8)
		for bit in range(0, 8):
			if (self._crc&0x8000)  == 0x8000:
				self._crc = ((self._crc << 1) ^ 0x1021)
			else:
				self._crc = self._crc << 1
		return

	def _sendcommand(self,address,command):
		self.crc_clear()
		self.crc_update(address)
#		self._port.write(chr(address))
		self._port.write(address.to_bytes(1, 'big'))
		self.crc_update(command)
#		self._port.write(chr(command))
		self._port.write(command.to_bytes(1, 'big'))
		return

	def _readchecksumword(self):
		data = self._port.read(2)
		if len(data)==2:
#			crc = (ord(data[0])<<8) | ord(data[1])
			crc = (data[0]<<8) | data[1]
			return (1,crc)	
		return (0,0)
		
	def _readbyte(self):
		data = self._port.read(1)
		if len(data):
			val = ord(data)
			self.crc_update(val)
			return (1,val)	
		return (0,0)
		
	def _readword(self):
		val1 = self._readbyte()
		if val1[0]:
			val2 = self._readbyte()
			if val2[0]:
				return (1,val1[1]<<8|val2[1])
		return (0,0)

	def _readlong(self):
		val1 = self._readbyte()
		if val1[0]:
			val2 = self._readbyte()
			if val2[0]:
				val3 = self._readbyte()
				if val3[0]:
					val4 = self._readbyte()
					if val4[0]:
						return (1,val1[1]<<24|val2[1]<<16|val3[1]<<8|val4[1])
		return (0,0)	

	def _readslong(self):
		val = self._readlong()
		if val[0]:
			if val[1]&0x80000000:
				return (val[0],val[1]-0x100000000)
			return (val[0],val[1])
		return (0,0)

	def _writebyte(self,val):
		self.crc_update(val&0xFF)
#		self._port.write(chr(val&0xFF))
		self._port.write(val.to_bytes(1, 'big'))

	def _writesbyte(self,val):
		self._writebyte(val)

	def _writeword(self,val):
		self._writebyte((val>>8)&0xFF)
		self._writebyte(val&0xFF)
		
	def _writesword(self,val):
		self._writeword(val)

	def _writelong(self,val):
		self._writebyte((val>>24)&0xFF)
		self._writebyte((val>>16)&0xFF)
		self._writebyte((val>>8)&0xFF)
		self._writebyte(val&0xFF)

	def _writeslong(self,val):
		self._writelong(val)

	def _read1(self,address,cmd):
		trys = self._trystimeout
		while 1:
			self._port.flushInput()
			self._sendcommand(address,cmd)
			val1 = self._readbyte()
			if val1[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF!=crc[1]&0xFFFF:
						return (0,0)
					return (1,val1[1])
			trys-=1
			if trys==0:
				break
		return (0,0)

	def _read2(self,address,cmd):
		trys = self._trystimeout
		while 1:
			self._port.flushInput()
			self._sendcommand(address,cmd)
			val1 = self._readword()
			if val1[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF!=crc[1]&0xFFFF:
						return (0,0)
					return (1,val1[1])
			trys-=1
			if trys==0:
				break
		return (0,0)

	def _read4(self,address,cmd):
		trys = self._trystimeout
		while 1:
			self._port.flushInput()
			self._sendcommand(address,cmd)
			val1 = self._readlong()
			if val1[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF!=crc[1]&0xFFFF:
						return (0,0)
					return (1,val1[1])
			trys-=1
			if trys==0:
				break
		return (0,0)

	def _read4_1(self,address,cmd):
		trys = self._trystimeout
		while 1:
			self._port.flushInput()
			self._sendcommand(address,cmd)
			val1 = self._readslong()
			if val1[0]:
				val2 = self._readbyte()
				if val2[0]:
					crc = self._readchecksumword()
					if crc[0]:
						if self._crc&0xFFFF!=crc[1]&0xFFFF:
							return (0,0)
						return (1,val1[1],val2[1])
			trys-=1
			if trys==0:
				break
		return (0,0)

	def _read_n(self,address,cmd,args):
		trys = self._trystimeout
		while 1:
			self._port.flushInput()
			trys-=1
			if trys==0:
				break
			failed=False
			self._sendcommand(address,cmd)
			data = [1,]
			for i in range(0,args):
				val = self._readlong()
				if val[0]==0:
					failed=True
					break
				data.append(val[1])
			if failed:
				continue
			crc = self._readchecksumword()
			if crc[0]:
				if self._crc&0xFFFF==crc[1]&0xFFFF:
					return (data);
		return (0,0,0,0,0)

	def _writechecksum(self):
		self._writeword(self._crc&0xFFFF)
		val = self._readbyte()
		if(len(val)>0):
			if val[0]:
				return True
		return False

	def _write0(self,address,cmd):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write1(self,address,cmd,val):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writebyte(val)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write11(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writebyte(val1)
			self._writebyte(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write111(self,address,cmd,val1,val2,val3):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writebyte(val1)
			self._writebyte(val2)
			self._writebyte(val3)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write2(self,address,cmd,val):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writeword(val)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS2(self,address,cmd,val):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writesword(val)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write22(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writeword(val1)
			self._writeword(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS22(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writesword(val1)
			self._writeword(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS2S2(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writesword(val1)
			self._writesword(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS24(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writesword(val1)
			self._writelong(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS24S24(self,address,cmd,val1,val2,val3,val4):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writesword(val1)
			self._writelong(val2)
			self._writesword(val3)
			self._writelong(val4)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4(self,address,cmd,val):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS4(self,address,cmd,val):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writeslong(val)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write44(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writelong(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4S4(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writeslong(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS4S4(self,address,cmd,val1,val2):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writeslong(val1)
			self._writeslong(val2)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write441(self,address,cmd,val1,val2,val3):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writelong(val2)
			self._writebyte(val3)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS441(self,address,cmd,val1,val2,val3):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writeslong(val1)
			self._writelong(val2)
			self._writebyte(val3)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4S4S4(self,address,cmd,val1,val2,val3):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writeslong(val2)
			self._writeslong(val3)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4S441(self,address,cmd,val1,val2,val3,val4):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writeslong(val2)
			self._writelong(val3)
			self._writebyte(val4)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4444(self,address,cmd,val1,val2,val3,val4):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writelong(val2)
			self._writelong(val3)
			self._writelong(val4)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4S44S4(self,address,cmd,val1,val2,val3,val4):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writeslong(val2)
			self._writelong(val3)
			self._writeslong(val4)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write44441(self,address,cmd,val1,val2,val3,val4,val5):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writelong(val2)
			self._writelong(val3)
			self._writelong(val4)
			self._writebyte(val5)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _writeS44S441(self,address,cmd,val1,val2,val3,val4,val5):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writeslong(val1)
			self._writelong(val2)
			self._writeslong(val3)
			self._writelong(val4)
			self._writebyte(val5)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4S44S441(self,address,cmd,val1,val2,val3,val4,val5,val6):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writeslong(val2)
			self._writelong(val3)
			self._writeslong(val4)
			self._writelong(val5)
			self._writebyte(val6)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4S444S441(self,address,cmd,val1,val2,val3,val4,val5,val6,val7):
		trys=self._trystimeout
		while trys:
			self._sendcommand(self,address,cmd)
			self._writelong(val1)
			self._writeslong(val2)
			self._writelong(val3)
			self._writelong(val4)
			self._writeslong(val5)
			self._writelong(val6)
			self._writebyte(val7)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write4444444(self,address,cmd,val1,val2,val3,val4,val5,val6,val7):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writelong(val2)
			self._writelong(val3)
			self._writelong(val4)
			self._writelong(val5)
			self._writelong(val6)
			self._writelong(val7)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	def _write444444441(self,address,cmd,val1,val2,val3,val4,val5,val6,val7,val8,val9):
		trys=self._trystimeout
		while trys:
			self._sendcommand(address,cmd)
			self._writelong(val1)
			self._writelong(val2)
			self._writelong(val3)
			self._writelong(val4)
			self._writelong(val5)
			self._writelong(val6)
			self._writelong(val7)
			self._writelong(val8)
			self._writebyte(val9)
			if self._writechecksum():
				return True
			trys=trys-1
		return False

	#User accessible functions
	def SendRandomData(self,cnt):
		for i in range(0,cnt):
			byte = random.getrandbits(8)
#			self._port.write(chr(byte))
			self._port.write(byte.to_bytes(1, 'big'))
		return

	def ForwardM1(self,address,val):
		return self._write1(address,self.Cmd.M1FORWARD,val)

	def BackwardM1(self,address,val):
		return self._write1(address,self.Cmd.M1BACKWARD,val)

	def SetMinVoltageMainBattery(self,address,val):
		return self._write1(address,self.Cmd.SETMINMB,val)

	def SetMaxVoltageMainBattery(self,address,val):
		return self._write1(address,self.Cmd.SETMAXMB,val)

	def ForwardM2(self,address,val):
		return self._write1(address,self.Cmd.M2FORWARD,val)

	def BackwardM2(self,address,val):
		return self._write1(address,self.Cmd.M2BACKWARD,val)

	def ForwardBackwardM1(self,address,val):
		return self._write1(address,self.Cmd.M17BIT,val)

	def ForwardBackwardM2(self,address,val):
		return self._write1(address,self.Cmd.M27BIT,val)

	def ForwardMixed(self,address,val):
		return self._write1(address,self.Cmd.MIXEDFORWARD,val)

	def BackwardMixed(self,address,val):
		return self._write1(address,self.Cmd.MIXEDBACKWARD,val)

	def TurnRightMixed(self,address,val):
		return self._write1(address,self.Cmd.MIXEDRIGHT,val)

	def TurnLeftMixed(self,address,val):
		return self._write1(address,self.Cmd.MIXEDLEFT,val)

	def ForwardBackwardMixed(self,address,val):
		return self._write1(address,self.Cmd.MIXEDFB,val)

	def LeftRightMixed(self,address,val):
		return self._write1(address,self.Cmd.MIXEDLR,val)

	def ReadEncM1(self,address):
		return self._read4_1(address,self.Cmd.GETM1ENC)

	def ReadEncM2(self,address):
		return self._read4_1(address,self.Cmd.GETM2ENC)

	def ReadSpeedM1(self,address):
		return self._read4_1(address,self.Cmd.GETM1SPEED)

	def ReadSpeedM2(self,address):
		return self._read4_1(address,self.Cmd.GETM2SPEED)

	def ResetEncoders(self,address):
		return self._write0(address,self.Cmd.RESETENC)

	def ReadVersion(self,address):
		trys=self._trystimeout
		while 1:
			self._port.flushInput()
			self._sendcommand(address,self.Cmd.GETVERSION)
			str = ""
			passed = True
			for i in range(0,48):
				data = self._port.read(1)
				if len(data):
					val = ord(data)
					self.crc_update(val)
					if(val==0):
						break
#					str+=data[0]
					str+=chr(data[0])
				else:
					passed = False
					break
			if passed:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF==crc[1]&0xFFFF:
						return (1,str)
					else:
						time.sleep(0.01)
			trys-=1
			if trys==0:
				break
		return (0,0)

	def SetEncM1(self,address,cnt):
		return self._write4(address,self.Cmd.SETM1ENCCOUNT,cnt)

	def SetEncM2(self,address,cnt):
		return self._write4(address,self.Cmd.SETM2ENCCOUNT,cnt)

	def ReadMainBatteryVoltage(self,address):
		return self._read2(address,self.Cmd.GETMBATT)

	def ReadLogicBatteryVoltage(self,address,):
		return self._read2(address,self.Cmd.GETLBATT)

	def SetMinVoltageLogicBattery(self,address,val):
		return self._write1(address,self.Cmd.SETMINLB,val)

	def SetMaxVoltageLogicBattery(self,address,val):
		return self._write1(address,self.Cmd.SETMAXLB,val)

	def SetM1VelocityPID(self,address,p,i,d,qpps):
#		return self._write4444(address,self.Cmd.SETM1PID,long(d*65536),long(p*65536),long(i*65536),qpps)
		return self._write4444(address,self.Cmd.SETM1PID,d*65536,p*65536,i*65536,qpps)

	def SetM2VelocityPID(self,address,p,i,d,qpps):
#		return self._write4444(address,self.Cmd.SETM2PID,long(d*65536),long(p*65536),long(i*65536),qpps)
		return self._write4444(address,self.Cmd.SETM2PID,d*65536,p*65536,i*65536,qpps)

	def ReadISpeedM1(self,address):
		return self._read4_1(address,self.Cmd.GETM1ISPEED)

	def ReadISpeedM2(self,address):
		return self._read4_1(address,self.Cmd.GETM2ISPEED)

	def DutyM1(self,address,val):
		return self._writeS2(address,self.Cmd.M1DUTY,val)

	def DutyM2(self,address,val):
		return self._writeS2(address,self.Cmd.M2DUTY,val)

	def DutyM1M2(self,address,m1,m2):
		return self._writeS2S2(address,self.Cmd.MIXEDDUTY,m1,m2)

	def SpeedM1(self,address,val):
		return self._writeS4(address,self.Cmd.M1SPEED,val)

	def SpeedM2(self,address,val):
		return self._writeS4(address,self.Cmd.M2SPEED,val)

	def SpeedM1M2(self,address,m1,m2):
		return self._writeS4S4(address,self.Cmd.MIXEDSPEED,m1,m2)

	def SpeedAccelM1(self,address,accel,speed):
		return self._write4S4(address,self.Cmd.M1SPEEDACCEL,accel,speed)

	def SpeedAccelM2(self,address,accel,speed):
		return self._write4S4(address,self.Cmd.M2SPEEDACCEL,accel,speed)

	def SpeedAccelM1M2(self,address,accel,speed1,speed2):
		return self._write4S4S4(address,self.Cmd.MIXEDSPEEDACCEL,accel,speed1,speed2)

	def SpeedDistanceM1(self,address,speed,distance,buffer):
		return self._writeS441(address,self.Cmd.M1SPEEDDIST,speed,distance,buffer)

	def SpeedDistanceM2(self,address,speed,distance,buffer):
		return self._writeS441(address,self.Cmd.M2SPEEDDIST,speed,distance,buffer)

	def SpeedDistanceM1M2(self,address,speed1,distance1,speed2,distance2,buffer):
		return self._writeS44S441(address,self.Cmd.MIXEDSPEEDDIST,speed1,distance1,speed2,distance2,buffer)

	def SpeedAccelDistanceM1(self,address,accel,speed,distance,buffer):
		return self._write4S441(address,self.Cmd.M1SPEEDACCELDIST,accel,speed,distance,buffer)

	def SpeedAccelDistanceM2(self,address,accel,speed,distance,buffer):
		return self._write4S441(address,self.Cmd.M2SPEEDACCELDIST,accel,speed,distance,buffer)

	def SpeedAccelDistanceM1M2(self,address,accel,speed1,distance1,speed2,distance2,buffer):
		return self._write4S44S441(address,self.Cmd.MIXEDSPEEDACCELDIST,accel,speed1,distance1,speed2,distance2,buffer)

	def ReadBuffers(self,address):
		val = self._read2(address,self.Cmd.GETBUFFERS)
		if val[0]:
			return (1,val[1]>>8,val[1]&0xFF)
		return (0,0,0)

	def ReadPWMs(self,address):
		val = self._read4(address,self.Cmd.GETPWMS)
		if val[0]:
			pwm1 = val[1]>>16
			pwm2 = val[1]&0xFFFF
			if pwm1&0x8000:
				pwm1-=0x10000
			if pwm2&0x8000:
				pwm2-=0x10000
			return (1,pwm1,pwm2)
		return (0,0,0)

	def ReadCurrents(self,address):
		val = self._read4(address,self.Cmd.GETCURRENTS)
		if val[0]:
			cur1 = val[1]>>16
			cur2 = val[1]&0xFFFF
			if cur1&0x8000:
				cur1-=0x10000
			if cur2&0x8000:
				cur2-=0x10000
			return (1,cur1,cur2)
		return (0,0,0)

	def SpeedAccelM1M2_2(self,address,accel1,speed1,accel2,speed2):
		return self._write4S44S4(address,self.Cmd.MIXEDSPEED2ACCEL,accel1,speed1,accel2,speed2)

	def SpeedAccelDistanceM1M2_2(self,address,accel1,speed1,distance1,accel2,speed2,distance2,buffer):
		return self._write4S444S441(address,self.Cmd.MIXEDSPEED2ACCELDIST,accel1,speed1,distance1,accel2,speed2,distance2,buffer)

	def DutyAccelM1(self,address,accel,duty):
		return self._writeS24(address,self.Cmd.M1DUTYACCEL,duty,accel)

	def DutyAccelM2(self,address,accel,duty):
		return self._writeS24(address,self.Cmd.M2DUTYACCEL,duty,accel)

	def DutyAccelM1M2(self,address,accel1,duty1,accel2,duty2):
		return self._writeS24S24(address,self.Cmd.MIXEDDUTYACCEL,duty1,accel1,duty2,accel2)
		
	def ReadM1VelocityPID(self,address):
		data = self._read_n(address,self.Cmd.READM1PID,4)
		if data[0]:
			data[1]/=65536.0
			data[2]/=65536.0
			data[3]/=65536.0
			return data
		return (0,0,0,0,0)

	def ReadM2VelocityPID(self,address):
		data = self._read_n(address,self.Cmd.READM2PID,4)
		if data[0]:
			data[1]/=65536.0
			data[2]/=65536.0
			data[3]/=65536.0
			return data
		return (0,0,0,0,0)

	def SetMainVoltages(self,address,min, max):
		return self._write22(address,self.Cmd.SETMAINVOLTAGES,min,max)
		
	def SetLogicVoltages(self,address,min, max):
		return self._write22(address,self.Cmd.SETLOGICVOLTAGES,min,max)
		
	def ReadMinMaxMainVoltages(self,address):
		val = self._read4(address,self.Cmd.GETMINMAXMAINVOLTAGES)
		if val[0]:
			min = val[1]>>16
			max = val[1]&0xFFFF
			return (1,min,max)
		return (0,0,0)

	def ReadMinMaxLogicVoltages(self,address):
		val = self._read4(address,self.Cmd.GETMINMAXLOGICVOLTAGES)
		if val[0]:
			min = val[1]>>16
			max = val[1]&0xFFFF
			return (1,min,max)
		return (0,0,0)

	def SetM1PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max):
#		return self._write4444444(address,self.Cmd.SETM1POSPID,long(kd*1024),long(kp*1024),long(ki*1024),kimax,deadzone,min,max)
		return self._write4444444(address,self.Cmd.SETM1POSPID,kd*1024,kp*1024,ki*1024,kimax,deadzone,min,max)

	def SetM2PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max):
#		return self._write4444444(address,self.Cmd.SETM2POSPID,long(kd*1024),long(kp*1024),long(ki*1024),kimax,deadzone,min,max)
		return self._write4444444(address,self.Cmd.SETM2POSPID,kd*1024,kp*1024,ki*1024,kimax,deadzone,min,max)

	def ReadM1PositionPID(self,address):
		data = self._read_n(address,self.Cmd.READM1POSPID,7)
		if data[0]:
			data[1]/=1024.0
			data[2]/=1024.0
			data[3]/=1024.0
			return data
		return (0,0,0,0,0,0,0,0)
		
	def ReadM2PositionPID(self,address):
		data = self._read_n(address,self.Cmd.READM2POSPID,7)
		if data[0]:
			data[1]/=1024.0
			data[2]/=1024.0
			data[3]/=1024.0
			return data
		return (0,0,0,0,0,0,0,0)

	def SpeedAccelDeccelPositionM1(self,address,accel,speed,deccel,position,buffer):
		return self._write44441(address,self.Cmd.M1SPEEDACCELDECCELPOS,accel,speed,deccel,position,buffer)

	def SpeedAccelDeccelPositionM2(self,address,accel,speed,deccel,position,buffer):
		return self._write44441(address,self.Cmd.M2SPEEDACCELDECCELPOS,accel,speed,deccel,position,buffer)

	def SpeedAccelDeccelPositionM1M2(self,address,accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer):
		return self._write444444441(address,self.Cmd.MIXEDSPEEDACCELDECCELPOS,accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer)

	def SetM1DefaultAccel(self,address,accel):
		return self._write4(address,self.Cmd.SETM1DEFAULTACCEL,accel)

	def SetM2DefaultAccel(self,address,accel):
		return self._write4(address,self.Cmd.SETM2DEFAULTACCEL,accel)

	def SetPinFunctions(self,address,S3mode,S4mode,S5mode):
		return self._write111(address,self.Cmd.SETPINFUNCTIONS,S3mode,S4mode,S5mode)

	def ReadPinFunctions(self,address):
		trys = self._trystimeout
		while 1:
			self._sendcommand(address,self.Cmd.GETPINFUNCTIONS)
			val1 = self._readbyte()
			if val1[0]:
				val2 = self._readbyte()
				if val1[0]:
					val3 = self._readbyte()
					if val1[0]:
						crc = self._readchecksumword()
						if crc[0]:
							if self._crc&0xFFFF!=crc[1]&0xFFFF:
								return (0,0)
							return (1,val1[1],val2[1],val3[1])
			trys-=1
			if trys==0:
				break
		return (0,0)

	def SetDeadBand(self,address,min,max):
		return self._write11(address,self.Cmd.SETDEADBAND,min,max)

	def GetDeadBand(self,address):
		val = self._read2(address,self.Cmd.GETDEADBAND)
		if val[0]:
			return (1,val[1]>>8,val[1]&0xFF)
		return (0,0,0)
		
	#Warning(TTL Serial): Baudrate will change if not already set to 38400.  Communications will be lost
	def RestoreDefaults(self,address):
		return self._write0(address,self.Cmd.RESTOREDEFAULTS)

	def ReadTemp(self,address):
		return self._read2(address,self.Cmd.GETTEMP)

	def ReadTemp2(self,address):
		return self._read2(address,self.Cmd.GETTEMP2)

	def ReadError(self,address):
		return self._read4(address,self.Cmd.GETERROR)

	def ReadEncoderModes(self,address):
		val = self._read2(address,self.Cmd.GETENCODERMODE)
		if val[0]:
			return (1,val[1]>>8,val[1]&0xFF)
		return (0,0,0)
		
	def SetM1EncoderMode(self,address,mode):
		return self._write1(address,self.Cmd.SETM1ENCODERMODE,mode)

	def SetM2EncoderMode(self,address,mode):
		return self._write1(address,self.Cmd.SETM2ENCODERMODE,mode)

	#saves active settings to NVM
	def WriteNVM(self,address):
		return self._write4(address,self.Cmd.WRITENVM,0xE22EAB7A)

	#restores settings from NVM
	#Warning(TTL Serial): If baudrate changes or the control mode changes communications will be lost
	def ReadNVM(self,address):
		return self._write0(address,self.Cmd.READNVM)

	#Warning(TTL Serial): If control mode is changed from packet serial mode when setting config communications will be lost!
	#Warning(TTL Serial): If baudrate of packet serial mode is changed communications will be lost!
	def SetConfig(self,address,config):
		return self._write2(address,self.Cmd.SETCONFIG,config)

	def GetConfig(self,address):
		return self._read2(address,self.Cmd.GETCONFIG)

	def SetM1MaxCurrent(self,address,max):
		return self._write44(address,self.Cmd.SETM1MAXCURRENT,max,0)

	def SetM2MaxCurrent(self,address,max):
		return self._write44(address,self.Cmd.SETM2MAXCURRENT,max,0)

	def ReadM1MaxCurrent(self,address):
		data = self._read_n(address,self.Cmd.GETM1MAXCURRENT,2)
		if data[0]:
			return (1,data[1])
		return (0,0)

	def ReadM2MaxCurrent(self,address):
		data = self._read_n(address,self.Cmd.GETM2MAXCURRENT,2)
		if data[0]:
			return (1,data[1])
		return (0,0)

	def SetPWMMode(self,address,mode):
		return self._write1(address,self.Cmd.SETPWMMODE,mode)

	def ReadPWMMode(self,address):
		return self._read1(address,self.Cmd.GETPWMMODE)

	def ReadEeprom(self,address,ee_address):
		trys = self._trystimeout
		while 1:
			self._port.flushInput()
			self._sendcommand(address,self.Cmd.READEEPROM)
			self.crc_update(ee_address)
			self._port.write(chr(ee_address))
			val1 = self._readword()
			if val1[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF!=crc[1]&0xFFFF:
						return (0,0)
					return (1,val1[1])
			trys-=1
			if trys==0:
				break
		return (0,0)

	def WriteEeprom(self,address,ee_address,ee_word):
		retval = self._write111(address,self.Cmd.WRITEEEPROM,ee_address,ee_word>>8,ee_word&0xFF)
		if retval==True:
			trys = self._trystimeout
			while 1:
				self._port.flushInput()
				val1 = self._readbyte()
				if val1[0]:
					if val1[1]==0xaa:
						return True
				trys-=1
				if trys==0:
					break
		return False	
		
	def Open(self):
		try:
			self._port = serial.Serial(port=self.comport, baudrate=self.rate, timeout=1, interCharTimeout=self.timeout)
		except:
			return 0
		return 1

