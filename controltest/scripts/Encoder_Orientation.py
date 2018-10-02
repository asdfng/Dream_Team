import time 
import os
import math
from a_star import AStar
a_star = AStar()
a_star.motors(50,-50)
theta_initial = 0
dist_between_wheels = 0.541339
def displacement(self): #velocity: ft/s, position: ft
    pi = math.pi
    self.right_wheel_rotations = right_encoder/1440                  #converts encoder counts to rotations
    self.left_wheel_rotations = left_encoder/1440                    
    self.right_displacement = self.right_wheel_rotations*2*pi*.114829     #calculates displacement of right, left and center wheels
    self.left_displacement = self.left_wheel_rotations*2*pi*.114829
    self.center_displacement = (self.right_displacement + self.left_displacement)/2
    self.alpha_left_turn_radians = (self.right_displacement - self.left_displacement)/dist_between_wheels  #calculates the change of the angle by a turn
    self.alpha_left_turn_degrees = self.alpha_left_turn_radians * 180/pi                              #converts to degrees
    self.theta_new = self.theta_initial + self.alpha_left_turn_degrees                                     #appends initial theta to new theta
    if self.theta_new >= 360:
        self.theta_new = self.theta_new - 360
        self.theta_initial = self.theta_new
    elif self.theta_new <= 0:
        self.theta_new = 360 + self.slef.theta_new
        self.theta_initial = self.theta_new
    else:
        self.theta_initial = self.theta_new #saves the new theta as the initial theta for next execution
    print("orientation = %s degrees",self.theta_new)                               #prints angle every 100ms
try:
    while True:
        encoders = a_star.read_encoders()
        print(encoders[0], encoders[1])
        right_encoder = encoders[0]
        left_encoder = encoders[1]
        displacement() 
        time.sleep(.1)
        os.system('clear')
except KeyboardInterrupt:
    a_star.motors(0,0)
