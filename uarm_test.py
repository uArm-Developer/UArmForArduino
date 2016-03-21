import serial
import sys
import time
import binascii

port = sys.argv[1]
ser = serial.Serial(port=port,baudrate=57600)

servo_rot_num = 1
servo_left_num = 2
servo_right_num = 3
servo_hand_rot_num = 4

START_SYSEX = 0xF0
ENCODER_DATA = 0xAA
END_SYSEX = 0xF7

def readAngle():
	print 'once'
	servo_add = int (sys.argv[2])
	data_offset = ( (int(sys.argv[3]) == 1) and 1 or 0)
	msg = bytearray([0xF0, ENCODER_DATA, 0x10])
	msg.extend(getValueAsOne7bitBytes(servo_add))
	msg.extend(getValueAsOne7bitBytes(data_offset))
	msg.append(0xF7)
	
	ser.write(msg)
	while ser.readable():
		readData = ord(ser.read(1))
		received_data = []
		if (readData == START_SYSEX):
			readData = ord(ser.read(1))
			if (readData == ENCODER_DATA):
				readData = ord(ser.read(1))
				while readData != END_SYSEX:
					received_data.append(readData)
					readData = ord(ser.read(1))
				return received_data[1]*128+received_data[0]+received_data[2]/100.0

def readEEPROM(val):
	print 'once'
	eeprom_add = int (val)#(sys.argv[2])
	msg = bytearray([0xF0, ENCODER_DATA, 0x1A])
	msg.extend(getValueAsTwo7bitBytes(eeprom_add))
	msg.append(0xF7)
	
	ser.write(msg)
	while ser.readable():
		readData = ord(ser.read(1))
		received_data = []
		if (readData == START_SYSEX):
			readData = ord(ser.read(1))
			if (readData == ENCODER_DATA):
				readData = ord(ser.read(1))
				while readData != END_SYSEX:
					received_data.append(readData)
					readData = ord(ser.read(1))
				return received_data[1]*128+received_data[0]			

def readDigital(val,pin_mode):
	print 'once'
	pin_number = int (val)#(sys.argv[2])
	pin_mode = pin_mode  # 1means pullup 0 means input
	msg = bytearray([0xF0, ENCODER_DATA, 0x14])
	msg.extend(getValueAsOne7bitBytes(pin_number))
	msg.extend(getValueAsOne7bitBytes(pin_mode))
	msg.append(0xF7)
	
	ser.write(msg)
	while ser.readable():
		readData = ord(ser.read(1))
		received_data = []
		if (readData == START_SYSEX):
			readData = ord(ser.read(1))
			if (readData == ENCODER_DATA):
				readData = ord(ser.read(1))
				while readData != END_SYSEX:
					received_data.append(readData)
					readData = ord(ser.read(1))
				return received_data[0]	

def readAnalog(pin_num):
	print 'once'

	msg = bytearray([0xF0, ENCODER_DATA, 0x16])
	msg.extend(getValueAsOne7bitBytes(pin_num))
	msg.append(0xF7)
	
	ser.write(msg)
	while ser.readable():
		readData = ord(ser.read(1))
		received_data = []
		if (readData == START_SYSEX):
			readData = ord(ser.read(1))
			if (readData == ENCODER_DATA):
				readData = ord(ser.read(1))
				while readData != END_SYSEX:
					received_data.append(readData)
					readData = ord(ser.read(1))
				return received_data[1]*128+received_data[0]			

def readCoords():
	print 'once'
	start_trigger = 1;
	msg = bytearray([0xF0, ENCODER_DATA, 0x12])
	msg.extend(getValueAsOne7bitBytes(start_trigger))
	msg.append(0xF7)
	
	ser.write(msg)
	while ser.readable():
		readData = ord(ser.read(1))
		received_data = []
		coords = []
		coords_val = []
		if (readData == START_SYSEX):
			readData = ord(ser.read(1))
			if (readData == ENCODER_DATA):
				readData = ord(ser.read(1))
				while readData != END_SYSEX:
					received_data.append(readData)
					readData = ord(ser.read(1))
				for i in range(0,3):
					coords_sign = (received_data[i*4] == 1 and 1 or -1)
					if i == 1 or i == 2:
						coords_sign = -coords_sign
					coords_val = received_data[2+4*i]*128+received_data[1+4*i]+received_data[3+4*i]/100.0
					coords.append(coords_sign*coords_val)
				return coords





def eepromWrite(eeprom_add,eeprom_val):
	msg = bytearray([0xF0, ENCODER_DATA, 0x1B])
	msg.extend(getValueAsTwo7bitBytes(eeprom_add))
	msg.extend(getValueAsTwo7bitBytes(eeprom_val))
	msg.append(0xF7)
	#time.sleep(3)
	print binascii.hexlify(bytearray(msg))
	print 'once'
	ser.write(msg)




def writeAnalog():
	pin_number = int (sys.argv[2])
	pin_value = int (sys.argv[3])

	msg = bytearray([0xF0, ENCODER_DATA, 0x17])
	msg.extend(getValueAsOne7bitBytes(pin_number))
	msg.extend(getValueAsTwo7bitBytes(pin_value))
	msg.append(0xF7)
	time.sleep(3)
	print binascii.hexlify(bytearray(msg))
	print 'once'
	ser.write(msg)



def writeDigital():
	pin_number = int (sys.argv[2])
	pin_mode = ((int (sys.argv[3])) ==1) and 1 or 0

	msg = bytearray([0xF0, ENCODER_DATA, 0x15])
	msg.extend(getValueAsOne7bitBytes(pin_number))
	msg.extend(getValueAsOne7bitBytes(pin_mode))
	print pin_mode
	msg.append(0xF7)
	time.sleep(3)
	print binascii.hexlify(bytearray(msg))
	print 'once'
	ser.write(msg)


def servoAttach(val):
	servo_status = int (val)
	msg = bytearray([0xF0, ENCODER_DATA, 0x1C])
	msg.extend(getValueAsOne7bitBytes(servo_status))
	msg.append(0xF7)
	time.sleep(3)
	print 'once'
	ser.write(msg)




def writeServoAngle():
	servo_angle = float(sys.argv[2])
	servo_number = int(sys.argv[3])
	writeWithoffset = int(sys.argv[4])
	msg = bytearray([0xF0, ENCODER_DATA, 0x11])
	msg.extend(getValueAsOne7bitBytes(servo_number))
	msg.extend(getValueAsThree7bitBytes(servo_angle))
	msg.extend(getValueAsOne7bitBytes(writeWithoffset))
	msg.append(0xF7)
	time.sleep(5)
	print binascii.hexlify(bytearray(msg))
	print 'once'
	ser.write(msg)
	time.sleep(2)


def moveTotest():

	x,y,z = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
	moveTo(x,y,z)

def pumpTest(val):
	pump_status = int (val)

	msg = bytearray([0xF0, ENCODER_DATA, 0x1D])
	msg.extend(getValueAsOne7bitBytes(pump_status))
	msg.append(0xF7)
	time.sleep(3)
	print 'once'
	ser.write(msg)




def moveToOpts(x,y,z,hand_angle,relative_flags,time_spend,path_type,ease_type):
	print 'once'
	msg = bytearray([0xF0, ENCODER_DATA, 0x13])
	msg.extend(getValueAsFour7bitBytes(x))
	msg.extend(getValueAsFour7bitBytes(y))
	msg.extend(getValueAsFour7bitBytes(z))
	msg.extend(getValueAsThree7bitBytes(hand_angle))
	msg.extend(getValueAsOne7bitBytes(relative_flags))
	msg.extend(getValueAsThree7bitBytes(time_spend))
	msg.extend(getValueAsOne7bitBytes(path_type))
	msg.extend(getValueAsOne7bitBytes(ease_type))
	msg.append(0xF7)
	#time.sleep(4)
	print binascii.hexlify(bytearray(msg))
	time.sleep(0.01)
	ser.write(msg)

def moveToSimple(x,y,z,hand_angle,relative_flags,time_spend):
	moveToOpts(x,y,z,hand_angle,relative_flags,time_spend,0,0)


def getValueAsOne7bitBytes(val):
	return bytearray([val]) 

def getValueAsTwo7bitBytes(val):
	int_val = int(val)
	return bytearray([abs(int_val) % 128, abs(int_val) >> 7])

def getValueAsThree7bitBytes(val):
	int_val = int(val)
	decimal_val = int(round((val - int_val) * 100))
	return bytearray([abs(int_val) % 128, abs(int_val) >> 7, abs(decimal_val)])

def getValueAsFour7bitBytes(val):
	int_val = int(val)
	decimal_val = int(round((val - int_val) * 100))
	return bytearray([(int_val > 0 and 1 or 0),abs(int_val) % 128, abs(int_val) >> 7, abs(decimal_val)]) 


def writeStretch():
	length = float(sys.argv[2])
	height = float(sys.argv[3])

	msg = bytearray([0xF0, ENCODER_DATA, 0x1E])
	msg.extend(getValueAsThree7bitBytes(length))
	msg.extend(getValueAsThree7bitBytes(height))
	msg.append(0xF7)

	print binascii.hexlify(bytearray(msg))

	ser.write(msg)
	time.sleep(2)




if __name__ == "__main__":
	time.sleep(5)
	writeStretch()
	







	'''
	x = float (sys.argv[2])
	y = float (sys.argv[3])
	z = float (sys.argv[4])
	hand_angle = float (sys.argv[5])
	
	relative_flags = ((int(sys.argv[6])) == 1 and 1 or 0)
	time_spend = float (sys.argv[7])


	moveToSimple(x,y,z,hand_angle,relative_flags,time_spend)
	'''
