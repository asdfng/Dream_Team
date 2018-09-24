import sys
sys.path.insert(0, '/home/pi/pololu-rpi-slave-arduino-library-2.0.0/pi')
import keyboard
from a_star import AStar

a_star = AStar()

if keyboard.is_pressed('w'):
    a_star.motors(int(200),int(200))
elif keyboard.is_pressed('s'):
    a_star.motors(int(-200),int(-200))
elif keyboard.is_pressed('a'):
    a_star.motors(int(-200),int(200))
elif keyboard.is_pressed('d'):
    a_star.motors(int(200),int(-200))
else:
    a_star.motors(int(0),int(0))

