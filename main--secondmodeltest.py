#!/usr/bin/env pybricks-micropython
#layout, ev3 brick to the east, sensors to the west (in front of house)
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
        #self.gyro = GyroSensor(Port.S4)
        self.leftambient = 0
        self.rightambient = 0
        self.currentambient = (self.left.ambient() + self.right.ambient())/2
        #self.delta = 0
        self.currentdelta = 0
    def check(self):
        self.leftambient = self.left.ambient() #the ambience of the port 3 color sensor
        self.rightambient = self.right.ambient() #the ambience of the port 2 color sensor
        #self.delta = self.leftambient - self.rightambient
        self.currentdelta = self.currentambient - (self.leftambient + self.rightambient)/2 #currentambient is the ambience once the adjusting of the solar panel stops. it's checking how much the ambience has changed, and returns that
        return self.currentdelta
        #print("LEFT: %d | RIGHT: %d" % (self.leftambient, self.rightambient))
brickthing = Parts()

time.sleep(5)

for x in range (20):
    #checkedo = brickthing.check()
    #print(checkedo)
    while brickthing.check() > 4 or brickthing.check() < -4: #while the absolute difference between the currentambience then and the currentambience now is less than 4
        print(brickthing.check()) #prints the check ; debug
        #print(str(brickthing.check()) + "--1")
        if (brickthing.check() < -5): #if the difference is negative
            brickthing.motorleft.run_angle(10, 10) #-10 turn clockwise
            brickthing.motorright.run_angle(10, 10) #-10 turn clockwise
        elif (brickthing.check() > 5): #if the difference is positive
            brickthing.motorleft.run_angle(10, -10) #10 turn counter-clockwise
            brickthing.motorright.run_angle(10, -10) #10 turn counter-clockwise
    brickthing.currentambient = (brickthing.left.ambient() + brickthing.right.ambient())/2 #set the current ambience to the average of the two current ambiences once the adjust happened
    print("BREAK") #print break to show that the adjust has happened
    time.sleep(5) #sleep for 5 seconds
#BRUHPLEASEWORK = Motor(Port.A)
#BRUHPLEASEWORK.run_time(200, 10, then=Stop.HOLD, wait=True)
#BRUHPLEASEWORK.run_target(500, 1000)
#time.sleep(5)
#BRUHPLEASEWORK.dc(100)
#time.sleep(10)