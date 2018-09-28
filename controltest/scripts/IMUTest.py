#code to use the IMU
#Written by: Nicholas Gregg

import time
from lms6 import LSM6

#Initiate the IMU and enable it
imu = LSM6()
imu.enable()

while True:
    
    #read the IMU, print and wait
    imu.read()

    imudata = {
        "gx": imu.g.x,
        "gy": imu.g.y,
        "gz": imu.g.z,
        "ax": imu.a.x,
        "ay": imu.a.y,
        "az": imu.a.z
    }

    print("G - x: %(gx)s y:%(gy)s z:%(gz)s /nA - x:%(ax)s y:%(ay)s z:%(az)s" % imudata)
    time.sleep(1)

