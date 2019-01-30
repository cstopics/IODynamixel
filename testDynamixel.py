import serial
from threading import Thread, Timer
import time
import numpy as np
import dynamixel_sdk

class IODynamixel:
	"Library to control Dynamixel motors"

	def __init__(self, motors, freq=50, port='/dev/ttyUSB0', baudrate=1000000, protocol=1.0, debug=True):
		self.motors = motors
		self.freq = freq
		self.period = 1/self.freq
		self.port = port
		self.baudrate = baudrate
		self.protocol = protocol
		self.debug = debug
		self.running = False
		# Initialize PortHandler instance
		self.portHandler = dynamixel_sdk.PortHandler(self.port)
		# Initialize PacketHandler instance
		self.packetHandler = dynamixel_sdk.PacketHandler(self.protocol)
		# Initialize GroupSyncWrite instance
		#self.groupSyncWrite = dynamixel_sdk.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
		if not self.openPort():
			print("Failed opening port")
		if not self.ping():
			print("Failed doing ping")

	def txrx(self):
		print('Hola')
		if self.running:
			Timer(self.period, self.txrx).start()

	def start(self):
		self.running = True
		Timer(self.period, self.txrx).start()

	def stop(self):
		self.running = False

	def ping(self):
		for DXL_ID in self.motors:
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

motors = {
		'abs_z':{'id':33, 'type':'MX-28'}
		}
motors = [33, 34, 35, 36, 37, 41, 42, 43, 44, 51, 52, 53, 54]

dxl = IODynamixel(motors=motors, freq=10)
dxl.start()
time.sleep(5)
dxl.stop()