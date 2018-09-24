import sys
sys.path.insert(0, '/home/pi/pololu-rpi-slave-arduino-library-2.0.0/pi')
import time
from a_star import AStar

a_star = AStar()

while True:

    a_star.motors(int(200),int(200))
    time.sleep(5)
    a_star.motors(int(-200),int(-200))
    time.sleep(5)
    a_star.motors(int(-200),int(200))
    time.sleep(5)
    a_star.motors(int(200),int(-200))
    time.sleep(5)
    a_star.motors(int(0),int(0))
    time.sleep(5)
