import sys
import os
import atexit
from IODynamixel import IODynamixel

try:
    if sys.ps1: interpreter = True
except AttributeError:
    interpreter = False
    if sys.flags.interactive: interpreter = True

if not interpreter:
	print('Error: Run this script in interactive mode')
	exit()

dxl = IODynamixel(creature="creatures/poppy_torso_sim.json", simulator='vrep')

if not dxl.correct:
	print('Error creating IODynamixel object')
	os._exit(1)

dxl.start()

def quit_gracefully():
	dxl.stop()

atexit.register(quit_gracefully)



