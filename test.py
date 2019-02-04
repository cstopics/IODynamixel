import numpy as np
from IODynamixel import IODynamixel

x = np.degrees()

dxl = IODynamixel()

dxl.loadMovement(5)

x = {x:"5"}

dxl.saveMovement(x,'A')