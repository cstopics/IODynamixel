import serial
from threading import Thread, Timer
import time
import numpy as np
import dynamixel_sdk
import json

import signal
import sys

class IODynamixel:
	"Library to control Dynamixel motors"

	ADDR_MX_TORQUE_ENABLE = 24
	LEN_MX_TORQUE_ENABLE = 1
	ADDR_MX_GOALPOS_ENABLE = 30
	LEN_MX_GOALPOS_ENABLE = 2
	ADDR_MX_PRESENT_POSITION = 36

	def __init__(self, json_file, freq=50, port='/dev/ttyUSB0', baudrate=1000000, protocol=1.0, debug=True):
		with open(json_file) as f:
			self.motors = json.load(f)
		for motor in self.motors:
			self.motors[motor]['currentPosition'] = int(-1)
			self.motors[motor]['motorAngle'] = float('nan')
			self.motors[motor]['robotAngle'] = float('nan')
			self.motors[motor]['torqueEnable'] = int(0)
			self.motors[motor]['goalPosition'] = int(-1)
			self.motors[motor]['motorGoal'] = float('nan')
			self.motors[motor]['robotGoal'] = self.motors[motor]['initPos']
		self.freq = freq
		self.period = 1/self.freq
		self.port = port
		self.baudrate = baudrate
		self.protocol = protocol
		self.debug = debug
		self.running = False
		self.threadOn = False
		self.testTime = time.time()
		self.sendTorque = False
		# Initialize PortHandler instance
		self.portHandler = dynamixel_sdk.PortHandler(self.port)
		# Initialize PacketHandler instance
		self.packetHandler = dynamixel_sdk.PacketHandler(self.protocol)

		if not self.openPort():
			print("Failed opening port")
		if not self.ping():
			print("Failed doing ping")

		# Initialize GroupSyncWrite instances
		self.groupSyncTorque = dynamixel_sdk.GroupSyncWrite(\
			self.portHandler, self.packetHandler, \
			IODynamixel.ADDR_MX_TORQUE_ENABLE, IODynamixel.LEN_MX_TORQUE_ENABLE)
		self.groupSyncGoalPos = dynamixel_sdk.GroupSyncWrite(\
			self.portHandler, self.packetHandler, \
			IODynamixel.ADDR_MX_GOALPOS_ENABLE, IODynamixel.LEN_MX_GOALPOS_ENABLE)
		for motor in self.motors:
			self.groupSyncTorque.addParam(self.motors[motor]['id'], [0])
		self.groupSyncTorque.txPacket()
			

		#self.syncWriteTorque()
		#self.groupSyncWrite = dynamixel_sdk.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

	def syncWriteTorque(self):
		for motor in self.motors:
			self.groupSyncTorque.changeParam(self.motors[motor]['id'], \
				[self.motors[motor]['torqueEnable']])
		self.groupSyncTorque.txPacket()

	def syncWriteGoalPos(self):
		self.groupSyncGoalPos.clearParam()
		for motor in self.motors:
			if self.motors[motor]['torqueEnable']:
				param_goal_position = \
					[dynamixel_sdk.DXL_LOBYTE(dynamixel_sdk.DXL_LOWORD(self.motors[motor]['goalPosition'])), \
					dynamixel_sdk.DXL_HIBYTE(dynamixel_sdk.DXL_LOWORD(self.motors[motor]['goalPosition'])) \
					]
				self.groupSyncGoalPos.addParam(self.motors[motor]['id'], param_goal_position)
		self.groupSyncGoalPos.txPacket()

	def enableTorque(self, names):
		for name in names:
			self.motors[name]['torqueEnable'] = int(1)
		self.sendTorque = True

	def disableTorque(self, names):
		for name in names:
			self.motors[name]['torqueEnable'] = int(0)
		self.sendTorque = True

	def txrx(self):
		while(self.threadOn):
			pass
		self.threadOn = True
		if self.running:
			Timer(self.period, self.txrx).start()
		# print(1/(time.time() - self.testTime))
		self.testTime = time.time()
		# TX
		for motor in self.motors:
			if self.motors[motor]['invert']:
				self.motors[motor]['motorGoal'] = \
					self.motors[motor]['offset'] - self.motors[motor]['robotGoal']
			else:
				self.motors[motor]['motorGoal'] = \
					self.motors[motor]['offset'] + self.motors[motor]['robotGoal']

			if self.motors[motor]['type'] == "MX-28":
				self.motors[motor]['goalPosition'] = \
					int((self.motors[motor]['motorGoal']+180)*4096.0/360.0)
				if self.motors[motor]['goalPosition'] < 0:
					self.motors[motor]['goalPosition'] = 0
				elif self.motors[motor]['goalPosition'] > 4095:
					self.motors[motor]['goalPosition'] = 4095
			elif self.motors[motor]['type'] == "RX-28" or self.motors[motor]['type'] == "AX-12":
				self.motors[motor]['goalPosition'] = \
					int((self.motors[motor]['motorGoal']+150)*1024.0/300.0)
				if self.motors[motor]['goalPosition'] < 0:
					self.motors[motor]['goalPosition'] = 0
				elif self.motors[motor]['goalPosition'] > 1023:
					self.motors[motor]['goalPosition'] = 1023
		self.syncWriteGoalPos()
		if self.sendTorque:
			self.syncWriteTorque()
			self.sendTorque = False
		# RX
		for name in self.motors:
			DXL_ID = self.motors[name]['id']
			dxl_present_position, dxl_comm_result, dxl_error = \
				self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, IODynamixel.ADDR_MX_PRESENT_POSITION)
			if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
				self.motors[name]['currentPosition'] = float('nan')
				self.motors[name]['motorAngle'] = float('nan')
				# print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				self.motors[name]['currentPosition'] = float('nan')
				self.motors[name]['motorAngle'] = float('nan')
				# print("%s" % packetHandler.getRxPacketError(dxl_error))
			else: 
				self.motors[name]['currentPosition'] = float(dxl_present_position)

				if self.motors[name]['type'] == "MX-28":
					self.motors[name]['motorAngle'] = float(dxl_present_position)*360.0/4096.0 - 180.0
				elif self.motors[name]['type'] == "RX-28" or self.motors[name]['type'] == "AX-12":
					self.motors[name]['motorAngle'] = float(dxl_present_position)*300.0/1024.0 - 150.0
				else:
					self.motors[name]['motorAngle'] = float('nan')

				if self.motors[name]['invert']:
					self.motors[name]['robotAngle'] = -(self.motors[name]['motorAngle'] - self.motors[name]['offset'])
				else:
					self.motors[name]['robotAngle'] = self.motors[name]['motorAngle'] - self.motors[name]['offset']
			# print("ID="+str(DXL_ID)+" POS="+str(dxl_present_position))
		self.threadOn = False
		

	def start(self):
		self.running = True
		Timer(self.period, self.txrx).start()

	def stop(self):
		self.running = False

	def ping(self):
		for name in self.motors:
			DXL_ID = self.motors[name]['id']
			dxl_model_number, dxl_comm_result, dxl_error = \
				self.packetHandler.ping(self.portHandler, DXL_ID)
			if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
				return False
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
				return False
			else:
				if self.debug:
					print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID, dxl_model_number))
		return True

	def openPort(self):
		if self.portHandler.openPort():
			if self.debug:
				print("Succeeded opening port " + self.port)
			if self.portHandler.setBaudRate(self.baudrate):
				if self.debug:
					print("Succeeded changing the baudrate")
				return True
			else:
				if self.debug:
					print("Failed changing the baudrate")
				return False
		else:
			print("Failed opening the port")
			return False

def signal_handler(sig, frame):
        dxl.stop()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

dxl = IODynamixel(json_file="motor_definitions.json", freq=40)

#while(True):
#	dxl.txrx()

dxl.start()

# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
