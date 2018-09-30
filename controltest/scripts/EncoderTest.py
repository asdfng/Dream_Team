#Test code for reading data from the Encoders
#Written by Nicholas Gregg

import time 
import os
from a_star import AStar

a_star = AStar()

while True:
    encoders = a_star.read_encoders()
    print(encoders)
    time.sleep(.1)
    os.system('clear')
