import sys


# Set the interpreter bool
try:
    if sys.ps1: interpreter = True
except AttributeError:
    interpreter = False
    if sys.flags.interactive: interpreter = True

# Use the interpreter bool
if interpreter: print('We are in the Interpreter')
else: print('We are running from the command line')