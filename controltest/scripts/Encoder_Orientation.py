import time 
import os
import math
from a_star import AStar
a_star = AStar()
a_star.motors(50,-50)
theta_initial = 0
dist_between_wheels = 0.541339
def displacement(self): #velocity: ft/s, position: ft
    self.right_wheel_rotations = right_encoder/1440                  #converts encoder counts to rotations
    self.left_wheel_rotations = left_encoder/1440                    
    self.right_displacement = right_wheel_rotations*2*pi*.114829     #calculates displacement of right, left and center wheels
    self.left_displacement = left_wheel_rotations*2*pi*.114829
    self.center_displacement = (right_displacement + left_displacement)/2
    self.alpha_left_turn_radians = (right_displacement - left_displacement)/dist_between_wheels  #calculates the change of the angle by a turn
    self.alpha_left_turn_degrees = alpha_left_turn_radians * 180/pi                              #converts to degrees
    self.theta_new = theta_initial + alpha_left_turn_degrees                                     #appends initial theta to new theta
    if theta_new >= 360:
        theta_new = theta_new - 360
        theta_initial = theta_new
    elif theta_new <= 0:
        theta_new = 360 + theta_new
        theta_initial = theta_new
    else
        theta_initial = theta_new
                                                                                                #saves the new theta as the initial theta for next execution
try:
    while True:
        encoders = a_star.read_encoders()
        print(encoders[0], encoders[1])
        right_encoder = encoders[0]
        left_encoder = encoders[1]
        displacement()
        print("orientation = %s degrees",displacement.theta_new)                               #prints angle every 100ms 
        theta_initial = displacement.theta_new
        time.sleep(.1)
        os.system('clear')
except KeyboardInterrupt:
    a_star.motors(0,0)
    