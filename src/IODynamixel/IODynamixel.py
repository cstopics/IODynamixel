import serial
import os
from threading import Thread, Timer, Lock
from vrep import vrep
import time
import numpy as np
import dynamixel_sdk
import json

"""IODynamixel: Package to control dynamixel motors"""

class IODynamixel:
    """IODynamixel controller"""
    ADDR_MX_TORQUE_ENABLE = 24
    LEN_MX_TORQUE_ENABLE = 1
    ADDR_MX_GOAL_POSITION = 30
    LEN_MX_GOAL_POSITION = 2
    ADDR_MX_MOVING_SPEED = 32
    LEN_MX_MOVING_SPEED = 2
    ADDR_MX_PRESENT_POSITION = 36

    def __init__(self, creature, freq=40.0, port='/dev/ttyUSB0', baudrate=1000000, protocol=1.0,
                 simulator='none'):
        """simulators: none, vrep"""
        self.correct = False
        self.simulator = simulator
        self.simulate = False if self.simulator == 'none' else True
        if creature[-5:]!='.json':
            print("Error: Incorrect creature file extension.")
            return
        self.creatureJson = creature.format(os.path.dirname(os.path.realpath(__file__)))
        with open(self.creatureJson) as f:
            self.motors = json.load(f)
        for motor in self.motors:
            self.motors[motor]['currentPosition'] = int(-1)
            self.motors[motor]['motorAngle'] = float('nan')
            self.motors[motor]['robotAngle'] = float('nan')
            self.motors[motor]['torqueEnable'] = int(0)
            self.motors[motor]['goalPosition'] = int(-1)
            self.motors[motor]['motorGoal'] = float('nan')
            self.motors[motor]['robotGoal'] = self.motors[motor]['initPos']
            self.motors[motor]['movingSpeed'] = int(200)
            if self.simulator == 'vrep':
                self.motors[motor]['vrep_handler'] = 0
        self.freq = freq
        self.period = 1/self.freq
        self.port = port
        self.baudrate = baudrate
        self.protocol = protocol
        self.running = False
        self.threadOn = False
        self.testTime = time.time()
        self.sendTorque = False
        self.playing = False
        self.recording = False
        self.movement = []
        self.playFrame = 0
        self.motorsToRec = []
        self.lockPlaying = Lock()
        if self.simulator=='none':
            # Initialize PortHandler instance
            self.portHandler = dynamixel_sdk.PortHandler(self.port)
            # Initialize PacketHandler instance
            self.packetHandler = dynamixel_sdk.PacketHandler(self.protocol)
            if not self.openPort():
                return
            if not self.ping():
                return
            # Initialize GroupSyncWrite instances
            self.groupSyncTorque = dynamixel_sdk.GroupSyncWrite(\
                self.portHandler, self.packetHandler, \
                IODynamixel.ADDR_MX_TORQUE_ENABLE, IODynamixel.LEN_MX_TORQUE_ENABLE)
            for motor in self.motors:
                self.groupSyncTorque.addParam(self.motors[motor]['id'], [0])
            self.groupSyncTorque.txPacket()

            self.groupSyncGoalPos = dynamixel_sdk.GroupSyncWrite(\
                self.portHandler, self.packetHandler, \
                IODynamixel.ADDR_MX_GOAL_POSITION, IODynamixel.LEN_MX_GOAL_POSITION)

            self.groupSyncMovSpeed = dynamixel_sdk.GroupSyncWrite(\
                self.portHandler, self.packetHandler, \
                IODynamixel.ADDR_MX_MOVING_SPEED, IODynamixel.LEN_MX_MOVING_SPEED)
            for motor in self.motors:
                self.groupSyncMovSpeed.addParam(self.motors[motor]['id'], [0, 0])
            self.syncWriteMovingSpeed()
        elif self.simulator=='vrep':
            vrep.simxFinish(-1)
            self.clientID = vrep.simxStart('127.0.0.1',19997,True,True,500,5)
            if self.clientID==-1:
                print("Error: Not connected to remote API server")
                return
            ret = 0
            fileName = self.creatureJson[:-4]+'ttt'
            ret += vrep.simxLoadScene(self.clientID, fileName, 1, vrep.simx_opmode_blocking)
            # ret += vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
            for motor in self.motors:
                ret0, handler = vrep.simxGetObjectHandle(self.clientID, motor, vrep.simx_opmode_blocking)
                self.motors[motor]['vrep_handler'] = handler
                ret += ret0
            if ret != 0:
                print("Error: Error with the remote API server")
                return
        else:
            print("Error: Simulator '" + self.simulator + "' not recognized.")
            return

        self.callbackPre = []
        self.callbackPost = []

        self.correct = True


    def syncWriteMovingSpeed(self):
        for motor in self.motors:
            param_goal_position = \
                [dynamixel_sdk.DXL_LOBYTE(dynamixel_sdk.DXL_LOWORD(self.motors[motor]['movingSpeed'])), \
                dynamixel_sdk.DXL_HIBYTE(dynamixel_sdk.DXL_LOWORD(self.motors[motor]['movingSpeed'])) \
                ]
            self.groupSyncMovSpeed.changeParam(self.motors[motor]['id'], param_goal_position)
        self.groupSyncMovSpeed.txPacket()

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

    def tx(self):
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
        if not self.simulate:
            self.syncWriteGoalPos()
            if self.sendTorque:
                self.syncWriteTorque()
                self.sendTorque = False
        elif self.simulator == 'vrep':
            for motor in self.motors:
                ret = vrep.simxSetJointTargetPosition(self.clientID,
                    self.motors[motor]['vrep_handler'],
                    np.radians(self.motors[motor]['motorGoal']), vrep.simx_opmode_streaming)

    def rx(self):
        for motor in self.motors:
            DXL_ID = self.motors[motor]['id']
            dxl_present_position, dxl_comm_result, dxl_error = \
                self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, IODynamixel.ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
                self.motors[motor]['currentPosition'] = float('nan')
                self.motors[motor]['motorAngle'] = float('nan')
                # print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.motors[motor]['currentPosition'] = float('nan')
                self.motors[motor]['motorAngle'] = float('nan')
                # print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                self.motors[motor]['currentPosition'] = float(dxl_present_position)

                if self.motors[motor]['type'] == "MX-28":
                    self.motors[motor]['motorAngle'] = float(dxl_present_position)*360.0/4096.0 - 180.0
                elif self.motors[motor]['type'] == "RX-28" or self.motors[motor]['type'] == "AX-12":
                    self.motors[motor]['motorAngle'] = float(dxl_present_position)*300.0/1024.0 - 150.0
                else:
                    self.motors[motor]['motorAngle'] = float('nan')

                if self.motors[motor]['invert']:
                    self.motors[motor]['robotAngle'] = -(self.motors[motor]['motorAngle'] - self.motors[motor]['offset'])
                else:
                    self.motors[motor]['robotAngle'] = self.motors[motor]['motorAngle'] - self.motors[motor]['offset']

    def rx_sim(self):
        for motor in self.motors:
            ret, pos = vrep.simxGetJointPosition(self.clientID, self.motors[motor]['vrep_handler'],
                                      vrep.simx_opmode_streaming)
            self.motors[motor]['currentPosition'] = float('nan')
            if ret != 0:
                self.motors[motor]['motorAngle'] = float('nan')
            else:
                self.motors[motor]['motorAngle'] = np.degrees(pos)

                if self.motors[motor]['invert']:
                    self.motors[motor]['robotAngle'] = -(self.motors[motor]['motorAngle'] - self.motors[motor]['offset'])
                else:
                    self.motors[motor]['robotAngle'] = self.motors[motor]['motorAngle'] - self.motors[motor]['offset']

    def _playMovement(self):
        for motor in self.movement['data']:
            self.motors[motor]['robotGoal'] = self.movement['data'][motor][self.playFrame]
        self.playFrame += 1
        if self.playFrame >= len(self.movement['data'][list(self.movement['data'].keys())[0]]):
            self.playing = False
            try:
                self.lockPlaying.release()
            except:
                pass

    def _recordMovement(self):
        for motor in self.motorsToRec:
            self.movement['data'][motor].append(self.motors[motor]['robotAngle'])

    def txrx(self):
        if self.threadOn:
            #print('Warning: Thread is not running at ' + str(self.freq) + ' FPS')
            while(self.threadOn):
                pass
        self.threadOn = True
        if self.running:
            Timer(self.period, self.txrx).start()
        # print(1/(time.time() - self.testTime))
        self.testTime = time.time()

        if self.callbackPre != []:
            self.callbackPre(self.motors)

        if self.playing:
            self._playMovement()
        self.tx()
        if self.simulate:
            self.rx_sim()
        else:
            self.rx()
        if self.recording:
            self._recordMovement()

        if self.callbackPost != []:
            self.callbackPost(self.motors)

        self.threadOn = False


    def start(self):
        self.running = True
        if self.simulator == 'vrep':
            ret = vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
            if ret != 0:
                print('Error: Could not start simulation')
        Timer(self.period, self.txrx).start()

    def stop(self):
        if self.simulator == 'vrep':
            ret = vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
            if ret != 0:
                print('Error: Could not stop simulation')
        self.running = False

    def ping(self):
        for name in self.motors:
            DXL_ID = self.motors[name]['id']
            dxl_model_number, dxl_comm_result, dxl_error = \
                self.packetHandler.ping(self.portHandler, DXL_ID)
            if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
                error = self.packetHandler.getTxRxResult(dxl_comm_result)
                print("Error: Failed doing ping to motor ID=%d. Comm error: %s" % (DXL_ID, error) )
                return False
            elif dxl_error != 0:
                error = self.packetHandler.getRxPacketError(dxl_error)
                print("Error: Failed doing ping to motor ID=%d. Dxl error: %s" % (DXL_ID, error) )
                return False
        return True

    def openPort(self):
        try:
            if self.portHandler.openPort():
                if not self.portHandler.setBaudRate(self.baudrate):
                    print("Error: Failed changing the baudrate of the port " + self.port)
                    return False
                return True
            else:
                print("Error: Failed opening the port "  + self.port)
                return False
        except:
            print("Error: Port "  + self.port + ' not found')
            return False

    # USER INTERFACE

    def get_motor_names(self):
        return list(self.motors.keys())

    def enableTorque(self, names):
        for name in names:
            self.motors[name]['torqueEnable'] = int(1)
        self.sendTorque = True

    def disableTorque(self, names):
        for name in names:
            self.motors[name]['torqueEnable'] = int(0)
        self.sendTorque = True

    def setAngle(self, names, angle):
        for i, name in enumerate(names):
            self.motors[name]['robotGoal'] = int(angle[i])

    def getAngle(self, names):
        angles = [0]*len(names)
        for i, name in enumerate(names):
            angles[i]=self.motors[name]['motorAngle']
        return angles 

    def playMovement(self, movement):
        if self.playing:
            print('Error: Currently running other movement')
            return
        if self.recording:
            print('Error: Currently recording movement')
            return
        else:
            print('Starting playing movement...')
            self.movement = movement.copy()
            self.playFrame = 0
            self.playing = True

    def playMovementBlock(self, movement):
        self.playMovement(movement)
        self.lockPlaying.acquire(False)
        self.lockPlaying.acquire(True)

    def recordMovement(self, motors):
        if self.playing:
            print('Error: Currently running other movement')
            return
        if self.recording:
            print('Error: Currently recording movement')
            return
        else:
            print('Starting recording movement...')
            self.motorsToRec = motors.copy()
            self.disableTorque(self.motorsToRec)
            self.movement = {'fps': 40, 'data': {}}
            for motor in self.motorsToRec:
                self.movement['data'][motor]=[]
            self.recording = True

    def stopRecording(self):
        self.recording = False
        return self.movement.copy()

    def setMovementInit(self, movement):
        for motor in movement['data']:
                self.motors[motor]['robotGoal'] = movement['data'][motor][0]

    #def saveMovement(self, movement: dict, file_name: str):
    def saveMovement(self, movement, file_name):
        with open(file_name, 'w') as file:
            json.dump(movement, file)

    #def loadMovement(self, file_name: str) -> dict:
    def loadMovement(self, file_name):
        file_name = file_name.format(os.path.dirname(os.path.realpath(__file__)))
        with open(file_name) as file:
            return json.load(file)
        return {}

    def setCallbackPre(self, callback):
        self.callbackPre = callback

    def setCallbackPost(self, callback):
        self.callbackPost = callback

    def interpolate(self, route):
        period = 1/self.freq
        times = np.array(route['times'])
        times_new = np.arange(times.min(), times.max()+period, period)
        movement = {}
        movement['fps'] = self.freq
        movement['data'] = {}
        for motor in route['motors']:
            if motor=='r_shoulder_x':
                data = np.degrees(-np.interp(times_new, times, np.array(route['motors'][motor])))
            else:
                data = np.degrees(np.interp(times_new, times, np.array(route['motors'][motor])))
            if self.motors[motor]['invert']:
                movement['data'][motor] = list(-(data - self.motors[motor]['offset']))
            else:
                movement['data'][motor] = list(data - self.motors[motor]['offset'])
            #movement['data'][motor] = list(np.degrees(np.interp(times_new, times, np.array(route['motors'][motor]))))
        return movement

