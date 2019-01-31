# -*- coding: utf-8 -*-
# Created by Yeisonint, Communication between pc and two dynamixel motors with different communication protocol

# Necessary libraries
import os
from dynamixel_sdk import *		# Uses Dynamixel SDK library
import time

if os.name == 'nt':
	import msvcrt
	def getch():
		return msvcrt.getch().decode()
else:
	import sys, tty, termios
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	def getch():
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

# Definitions
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 2300		       	# Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2500         	    # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
DXL_IDS = [54, 44]
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]

pos = [[2500, 420], [2300, 480]]

# Control table address
# Control table address is different in Dynamixel model
ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_MX_GOAL_POSITION       = 2
LEN_MX_PRESENT_POSITION    = 4

index = 0

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)
# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

# Open port
if portHandler.openPort():
	print("Succeeded to open the port")
	# Set port baudrate
	if portHandler.setBaudRate(BAUDRATE):
		print("Succeeded to change the baudrate")
	else:
		print("Failed to change the baudrate")
		quit()
else:
	print("Failed to open the port")
	quit()

# Try to ping the Dynamixel
for DXL_ID in DXL_IDS:
	dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID, dxl_model_number))

# Enable Motors Torque
for DXL_ID in DXL_IDS:
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("Dynamixel#%d has been successfully connected (Torque ON)" % DXL_ID)

for i, DXL_ID in enumerate(DXL_IDS):
		dxl_addparam_result = groupSyncWrite.addParam(DXL_ID, [0,0])
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID)
			quit()
groupSyncWrite.txPacket()

for i in range(0,10):
	# Allocate goal position value into byte array

	# Add Dynamixel Motors goal position value to the Syncwrite parameter storage
	for i, DXL_ID in enumerate(DXL_IDS):
		param_goal_position = \
			[DXL_LOBYTE(DXL_LOWORD(pos[index][i])), \
			DXL_HIBYTE(DXL_LOWORD(pos[index][i])) \
			]
		dxl_addparam_result = groupSyncWrite.changeParam(DXL_ID, param_goal_position)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID)
			quit()

	# Syncwrite goal position
	dxl_comm_result = groupSyncWrite.txPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

	# Clear syncwrite parameter storage
	# groupSyncWrite.clearParam()

	# Read Dynamixel present position
	for DXL_ID in DXL_IDS:
		dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		print("ID="+str(DXL_ID)+" POS="+str(dxl_present_position))

	# Change goal position
	if index == 0:
		index = 1
	else:
		index = 0
	time.sleep(1)

# Disable Motors Torque
for DXL_ID in DXL_IDS:
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("Dynamixel#%d has been successfully connected (Torque OFF)" % DXL_ID)

# Close port
portHandler.closePort()