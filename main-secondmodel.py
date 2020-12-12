#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time
from mainmodeltest import Parts
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

brickthing = Parts()

'''test---for x in range(20):
    brickthing.leftambient = brickthing.left.ambient()
    brickthing.rightambient = brickthing.right.ambient()
    print("LEFT: %d | RIGHT: %d" % (brickthing.leftambient, brickthing.rightambient))
    time.sleep(2)'''

#prototype 2print((self.currentdelta+self.currentdelta2)/2)
time.sleep(5)
for x in range(20):
    while brickthing.check() != 0:
        if (brickthing.check() > 10): #setting, left clockwise, right counter-clockwise
            brickthing.motorleft.run_angle(10, 10)
            brickthing.motorright.run_angle(10, 10)
        elif (brickthing.check() < -10): #rising, right clockwise, left counter-clockwise
            brickthing.motorleft.run_angle(10, -10)
            brickthing.motorright.run_angle(10, -10)

    print("STABLE")
'''prototype 1
time.sleep(5)
for x in range (20):
    checkedo = brickthing.check()
    print(checkedo)
    while brickthing.check() != "STABLE":
        if (brickthing.check() > 10):
            print("TURNING")
            brickthing.motorleft.run_angle(10, 10)
            brickthing.motorright.run_angle(10, 10) #rising, right
        elif (brickthing.check() < -10):
            brickthing.motorleft.run_angle(10, -10)
            brickthing.motorright.run_angle(10, -10) #sinking, left
    print("BREAK")
    time.sleep(5)'''
#BRUHPLEASEWORK = Motor(Port.A)
#BRUHPLEASEWORK.run_time(200, 10, then=Stop.HOLD, wait=True)
#BRUHPLEASEWORK.run_target(500, 1000)
#time.sleep(5)
#BRUHPLEASEWORK.dc(100)
#time.sleep(10)
