#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

class Parts():
    def __init__(self):
        self.ev3 = EV3Brick()
        self.motorleft = Motor(Port.D)
        self.motorright = Motor(Port.A)
        self.right = ColorSensor(Port.S2)
        self.left = ColorSensor(Port.S3)
        self.leftambient = 0
        self.rightambient = 0
        self.currentambient = self.right.ambient()
        self.currentdelta = 0
        self.currentdelta2 = 0
    def check(self):
        self.leftambient = self.left.ambient()
        self.rightambient = self.right.ambient()
        self.currentdelta = self.currentambient - self.rightambient
        self.currentdelta = self.currentambient - self.leftambient
        return self.currentdelta2
        #DATA
        '''if self.currentdelta > 4: #setting
            #left, clockwise
            #right, counter-clockwise
            return "setting"
        elif self.currentdelta < 4: #rising
            #left, counter-clockwise
            #right, clockwise
            return "rising"
        elif self.currentdelta == 0: #stable
            #stable
            return "stable"'''
        #if self.currentdelta < 4 or self.currentdelta > -4:
        #    return "STABLE"
        #else:
        #    self.currentambient = (self.leftambient + self.rightambient)/2    
        #    return self.currentdelta
        #print("LEFT: %d | RIGHT: %d" % (self.leftambient, self.rightambient))

'''for x in range(10):
    brickthing.motorlef5t.run_angle(10, 10)
    brickthing.motorright.run_angle(10, -10) #rising, right

for x in range(10):
    brickthing.motorleft.run_angle(10, -10)
    brickthing.motorright.run_angle(10, -10) #sinking, left'''

'''time.sleep(5)
for x in range (20):
    checkedo = brickthing.check()
    print(checkedo)
    while brickthing.check() != "STABLE":
        brickthing.currentdelta = (brickthing.currentambient - ((brickthing.left.ambient() + brickthing.right.ambient())/2))
        #print(str(brickthing.check()) + "--1")
        if (brickthing.check() > 10):
            print("TURNING")
            brickthing.motorleft.run_angle(10, 10)
            brickthing.motorright.run_angle(10, 10) #rising, right
        elif (brickthing.check() < -10):
            brickthing.motorleft.run_angle(10, -10)
            brickthing.motorright.run_angle(10, -10) #sinking, left
    print("BREAK")
    time.sleep(5)
#BRUHPLEASEWORK = Motor(Port.A)
#BRUHPLEASEWORK.run_time(200, 10, then=Stop.HOLD, wait=True)
#BRUHPLEASEWORK.run_target(500, 1000)
#time.sleep(5)
#BRUHPLEASEWORK.dc(100)
#time.sleep(10)'''