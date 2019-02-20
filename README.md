# IODynamixel

Library to control Dynamixel motors vía *creature* scheme.

Do not forget to visit our web page: https://cstopics.github.io/cstopics/

## Dependencies

* Python >= 3.5

Pip dependencies:

* pyserial (for real robot)
* dynamixel_sdk (for real robot) (https://github.com/ROBOTIS-GIT/DynamixelSDK)

Other software:

* V-rep (for simulation)

## Installation

Clone the repository:

``` bash
$ cd /{working_folder}/
$ git clone https://github.com/cstopics/IODynamixel
```

There are two alternatives to install this library:

**Add the library to the Python path**

Add the *src* folder to the *PYTHONPATH* enviroment variable:

``` bash
$ export PYTHONPATH="/{working_folder}/IODynamixel/src:$PYTHONPATH"
```
You must do it each time you launch a terminal, if you don't want that, you can add this line to your *~/.bashrc* file.

**Install the library on your system**

Go to the cloned repository and install it:
``` bash
$ cd /{working_folder}/IODynamixel
$ python setup.py install # Or python3, depending on your system (remember that must be Python >= 3.5)
```

## USB port configuration (for testing with real robot)

This library was tested with the *U2D2* controller (*Robotis*). In order to reach good update frequencies, it is necesary to reduce the latency delay, as follows (check that you controller is *ttyUSB0*):

``` bash
$ sudo su
# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# exit
```

It must be done each time you connect the U2D2 controler.

## V-rep configuration

...

## Testing

Open an interactive *Python 3* terminal and test.

Import the library:
``` python
>>> from IODynamixel.IODynamixel import IODynamixel
```
Create the controller object. For real robot:
``` python
>>> dxl = IODynamixel(creature="{}/creatures/poppy_torso_sim.json")
```
For simulation (*V-REP must be running*):
``` python
>>> dxl = IODynamixel(creature="{}/creatures/poppy_torso_sim.json", simulator='vrep')
```
Start the controller thread (by default it runs at 40.0Hz, you can change it with the *freq* parameter in the *IODynamixel* class constructor):
``` python
>>> dxl.start()
```
Get the name of robot motors:
``` python
>>> dxl.get_motor_names()
['r_shoulder_y', 'l_shoulder_y', 'abs_z', 'head_y', 'l_shoulder_x', 'head_z', 'r_elbow_y', 'r_shoulder_x', 'bust_y', 'bust_x', 'l_arm_z', 'l_elbow_y', 'r_arm_z']
```
Enable torque of set of motors (only takes effect in real robot):
``` python
>>> dxl.enableTorque([r_shoulder_y', 'l_arm_z', 'r_arm_z'])
```
Disable torqu of set of motors (only takes effect in real robot):
``` python
>>> dxl.disableTorque(['l_arm_z', 'r_arm_z'])
```
Enable torque of all motors (only takes effect in real robot):
``` python
>>> dxl.enableTorque(dxl.get_motor_names())
```
Move motors to specific angle:
``` python
>>> dxl.setAngle(['r_shoulder_y'],[30.0])
>>> dxl.setAngle(['r_shoulder_y', 'l_shoulder_y', 'l_arm_z'],[10.0, 20.0, 30.0])
```
Load predefined movement and play it ({} means the library path):
``` python
>>> mov1 = dxl.loadMovement('{}/movements/saludo.json')
>>> dxl.setMovementInit(mov1) # Go to the initial position of the movement.
>>> dxl.playMovement(mov1)
```
Play movement and wait until it finishes:
``` python
>>> dxl.playMovementBlock(mov1)
```
Record, save and play custom movement (only with real robot):
``` python
>>> mov2 = dxl.recordMovement(['r_shoulder_y', 'r_elbow_y', 'r_shoulder_x', 'r_arm_z'])
>>> # Move the selected joints as you want.
>>> dxl.stopRecording()
>>> dxl.saveMovement(mov2, '{path to the output json file, including the extension}')
>>> dxl.playMovementBlock(mov2)
```

## Predefined movements

* saludo.json
* macarena.json

## Thanks!
