# -*- coding: utf-8 -*-
"""
Created on Thu Oct 18 04:18:11 2018
## Pololu Maestro Exploration for DRM
@author: Dave
"""
print ('Pololu Maestro Testing')
# Code from https://github.com/frc4564/maestro

import serial
import maestro

servo = maestro.Controller('COM22')

#servo = maestro.Controller(m)
print ('set accel')
servo.setAccel(5,4)      #set servo 0 acceleration to 4
print ('set position')
servo.setTarget(5,6000)  #set servo to move to center position
print ('set speed')
servo.setSpeed(5,10)     #set speed of servo 1
print ('get position')
x = servo.getPosition(5) #get the current position of servo 1
print (x)
servo.close()