# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 05:47:18 2017

Initiated on 10/4/2017

DRM_Control

This module will control the servo, take pictures, do image processing and
log data and statistics for a given set of run parameters.

Revision History
Rev 0.1 10/8/2017:  First operable version.  Used patched Python 3.0 code from
website to get Adafruit board running in Python 3.X


@author: Dave
"""
### Libraries
import os
import math
#import pandas as pd
#import pandas_datareader as pdr
#import matplotlib as plt
#import numpy as np
from datetime import datetime

# For servo control
from Adafruit_PWM_Servo_Driver import PWM
import time

# Camera
from picamera import PiCamera
from time import sleep

### Functions

def setServoPulse(channel, pulse):
      pulseLength = 1000000                   # 1,000,000 us per second
      pulseLength /= 60                       # 60 Hz
      print ("%d us per period" % pulseLength)
      pulseLength /= 4096                     # 12 bits of resolution
      print ("%d us per bit" % pulseLength)
      pulse *= 1000
      pulse /= pulseLength
      pwm.setPWM(channel, 0, pulse)

# Take Picture
def savePic(CamName,Text):
    CamName.start_preview()
    CamName.annotate_text = Text
    CamName.capture('/home/pi/Desktop/DRM_Images/' + Text + '.jpg')
    CamName.stop_preview()
# Image Processing

# Move Servo
def servoMove(ServoName,channel, SetOn, SetOff):
    ServoName.setPWM(channel, SetOn, SetOff)   # Change speed of continuous servo on channel O

# Input grab
def GetInput(Caption):
    input_name = input(Caption+': ')   
    return input_name
# MatPlotLib plotting

## Initialize servos, camera, etc.
camera=PiCamera()

# Initialise the PCA9685 using the default address (0x40).
pwm = PWM(0x40)

# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)
# Plate Servo
platechannel = 0
plateservoMin = 394  # Side 1 - Min pulse length (Default 150)
plateservoMax = 500 # Side 2 - Max pulse length (Default 600)

# Robot Servos

rbase_channel = 1
rbase_servoMin = 150
rbase_servoMax = 600

Larm_channel = 2
Larm_servoMin = 150
Larm_servoMax = 600

Uarm_channel = 3
Uarm_servoMin = 150
Uarm_servoMax = 600

Earm_channel = 4
Earm_servoMin = 150
Earm_servoMax = 600


pwm.setPWMFreq(60) # Set frequency to 60 Hz

## Get run parameters - # flips, die type, etc.
DiceType = GetInput('Dice Type')
NumFlips = int(GetInput('Number of Flips'))
WaitTime = 2 # Pause before flip
DiceNotes = GetInput('Dice Description')
RunName = GetInput('Run Name')
date_now = datetime.now()
date_now = date_now.strftime('%Y%m%d-%H%M%S')

### Main Loop
## Initialize
FlipCount=1
servoCmd = plateservoMin # Always start on side #1
servoMove(pwm,platechannel,0,servoCmd)

sleep(WaitTime)

while FlipCount <= NumFlips:
## Take and save picture
    date_now = datetime.now()
    date_now = date_now.strftime('%Y%m%d-%H%M%S')
    FlipSt = str(FlipCount)
    FlipStr = FlipSt.zfill(int(math.log10(NumFlips)+1)) # add leading zeros to FlipCount
    PicLabel = RunName + '_' + date_now +'_' + FlipStr 
    savePic(camera,PicLabel)
    # Image processing
    sleep(WaitTime)  
    ## Flip Box
    if servoCmd == plateservoMin:
        servoCmd = plateservoMax
    else:
        servoCmd = plateservoMin
    #Flip Box now    
    servoMove(pwm,platechannel,0,servoCmd)
    print(servoCmd)
    sleep(WaitTime) # Let die settle in box
    
    # Display count
    print ('Flip #' + str(FlipCount))
    FlipCount+=1

    # End Main Loop
# Update stats/distribution
print ('Run Complete')

# Write log file
