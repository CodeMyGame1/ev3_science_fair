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
        #self.motorleft = Motor(Port.D)
        self.motorright = Motor(Port.A)
        self.right = ColorSensor(Port.S3)
        self.left = ColorSensor(Port.S2)
        #self.gyro = GyroSensor(Port.S4)
        self.leftambient = 0
        self.rightambient = 0
        self.delta = 0
        #self.rightlimit = 0
        #self.leftlimit = 0
        self.middle = 0
        #self.savefile = open("testcheck.txt", "a")
        self.rightlimitreal = 50
        self.leftlimitreal = -50
        self.buffer = 2
        self.lll = 0
        self.rrr = 0
    def check(self):
        self.leftambient = self.left.ambient()
        self.rightambient = self.right.ambient()
        self.delta = self.leftambient - self.rightambient
        return self.delta
        #print("LEFT: %d | RIGHT: %d" % (self.leftambient, self.rightambient))        
    def checkGyro(self):
        print(self.gyro)
    def calibrate(self):
        self.motorright.run_until_stalled(10, then=Stop.BRAKE, duty_limit=None)
        #self.rightlimit = self.motorright.angle()
        self.motorright.run_until_stalled(-10, then=Stop.BRAKE, duty_limit=None)
        self.leftlimit = self.motorright.angle()
        #self.middle = (self.leftlimit + self.rightlimit) / 2
        #print(self.rightlimit)
        print(self.leftlimit)
        print(self.middle)
        #if self.middle positive then self.rightlimit > self.leftlimit, aka the set was leaning left
        #if self.middle negative then self.leftlimit > self.rightlimit, aka the set was leaning right
        #however, both measurements would end with the set being left, so you do not have to turn the command into a conditional
        while abs(self.motorright.angle() - self.middle) > 5:
            self.motorright.run_angle(10, 10)
        '''self.motorright.reset_angle(0)
        self.motorright.run_until_stalled(10, then=Stop.BRAKE, duty_limit=None)
        self.rightlimit = self.motorright.angle()
        self.motorright.run_until_stalled(-10, then=Stop.BRAKE, duty_limit=None)
        self.leftlimit = self.motorright.angle()
        self.middle = (self.leftlimit + self.rightlimit) / 2
        while abs(self.motorright.angle() - self.middle) > 5:
            self.motorright.run_angle(10, 10)'''

brickthing = Parts()

def mainloop():
    #print(brickthing.rightlimit)
    #print(brickthing.leftlimit)
    #print(brickthing.middle)
    print(brickthing.motorright.angle())
    print(brickthing.check())
    #for x in range (20):
    while True:
        #checkedo = brickthing.check()
        #print(checkedo)
        '''if (checkedo == "TURN LEFT"):
            brickthing.motorleft.run_time(3, 1, Stop.HOLD, wait=True)
        elif (checkedo == "TURN RIGHT"):
            brickthing.motorleft.run_time(3, 1, Stop.HOLD, wait=True)
        elif (checkedo == "STAY"):
            print("STAY")'''
        '''if (checkedo > 4):
            brickthing.motorleft.run_target(500, 90)
        elif (checkedo < -4):
            brickthing.motorright.run_target(500, 90)'''
        while True:#brickthing.check() > 1 or brickthing.check() < -1:
            #print("LEFT: " + str(brickthing.left.ambient()))
            #print("RIGHT: " + str(brickthing.right.ambient()))
            print("DELTA: " + str(brickthing.check()))
            #print("ANGLE MOTORRIGHT: " + str(brickthing.motorright.angle())) #91 and -4 
            #print(str(brickthing.check()) + "--1")
            #brickthing.motorright.run_angle(10, 10) #done
            #time.sleep(2) #done
            if (brickthing.check() > brickthing.buffer): #turn left
                if (brickthing.motorright.angle() <= brickthing.leftlimitreal):
                    print("OVERRIDE")
                    continue
                else:
                #brickthing.motorleft.run_angle(10, -10)
                    brickthing.motorright.run_angle(10, -10) #10 left
                    print("TURNED THIS WAY")
                    brickthing.lll += 1
                    #if ((brickthing.lll + brickthing.rrr > 3) and (brickthing.lll - 1 == brickthing.rrr)):
                    if (brickthing.lll + brickthing.rrr > 3):
                        brickthing.lll = 0
                        brickthing.rrr = 0
                        break
                    #else:
                    #    brickthing.lll = 0
                    #    brickthing.rrr = 0
            elif (brickthing.check() < -brickthing.buffer): #turn right
                if (brickthing.motorright.angle() >= brickthing.rightlimitreal):
                    print("OVERRIDE")
                    continue
                else:
                    #brickthing.motorleft.run_angle(10, 10)
                    brickthing.motorright.run_angle(10, 10) #-10 right
                    print("TURNED THAT WAY")
                    brickthing.rrr += 1
                    if ((brickthing.lll + brickthing.rrr > 3) and (brickthing.rrr - 1 == brickthing.lll)):
                        brickthing.lll = 0
                        brickthing.rrr = 0
                        break
                    #else:
                    #    brickthing.lll = 0
                    #    brickthing.rrr = 0
            #brickthing.savefile.write("LEFT: %d | RIGHT: %d" % (brickthing.leftambient, brickthing.rightambient))'''
        print(brickthing.check())
        print("BREAK")
        time.sleep(5)
    #BRUHPLEASEWORK = Motor(Port.A)
    #BRUHPLEASEWORK.run_time(200, 10, then=Stop.HOLD, wait=True)
    #BRUHPLEASEWORK.run_target(500, 1000)
    #time.sleep(5)
    #BRUHPLEASEWORK.dc(100)
    #time.sleep(10)

#brickthing.motorright.run_angle(10, 20)
#brickthing.motorright.run_angle(10, -20)
#mainloop()

time.sleep(5)
#mainloop()
#brickthing.calibrate()
mainloop()
'''while True:
    print(brickthing.motorright.angle())
    time.sleep(2)'''