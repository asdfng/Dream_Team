#Test code for reading data from the Encoders
#Written by Nicholas Gregg

import time 
import os
from a_star import AStar

a_star = AStar()
a_star.motors(50,-50)

try:
    while True:
        encoders = a_star.read_encoders()
        print(encoders[0], encoders[1])
        time.sleep(.1)
        os.system('clear')
except KeyboardInterrupt:
    a_star.motors(0,0)