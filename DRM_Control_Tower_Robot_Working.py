# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 05:47:18 2017

Initiated on 2/11/2018

DRM_Control

This module will control the plate and robot servos, take pictures, do image processing and
log data and statistics for a given set of run parameters.

Revision History
Rev A: 3/17/2018 - Added pincer servo, reset for Rev C Table

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

### Variables

# Locations in BCS (Base Coordinate System, all in cm)

# Robot Base
RbX = 24.5
RbY = 3.5
RbZ = -1

# Camera
CamX = 12.5
CamY = 19.5
CamZ = 0 # on top plane of base

# Tower
TowX = 4
TowY = 6
TowZ = 18

# Robot Arm Dimensions
Larm_Z = 14 # Z offset of Larm axis from Rb
Larm_L = 14 # Length of Larm
Uarm_L = 15 # Length of Uarm
Pinc_X = 2.5 # X offset of pincer tip from end of Uarm
Pinc_Y = -8 # Y offset o pincer tip from end of Uarm

# Plate Servo
plate_channel = -1
plateservoMin = 394  # Side 1 - Min pulse length (Default 150)
plateservoMax = 500 # Side 2 - Max pulse length (Default 600)

# Robot Servo Parameters

rbase_chnl = 3
rbase_servoMin = 120
rbase_servoMax = 550 # full left
rbase_servoDef = 350

Larm_chnl = 1
Larm_servoMin = 250
Larm_servoMax = 470 # fully retracted
Larm_servoDef = 350

Uarm_chnl = 4
Uarm_servoMin = 270 # fully retracted
Uarm_servoMax = 390 # fully extended
Uarm_servoDef = 350

Earm_chnl = 0
Earm_servoMin = 150
Earm_servoMax = 620 # black side facing out
Earm_servoDef = 350

Pinc_chnl = 2
Pinc_servoMin = 230 # touching
Pinc_servoMax = 450 # end of gear teeth
Pinc_servoDef = 350

### Functions

def RobotHome():
      servoCmd = 380
      servoMove(pwm,Uarm_chnl,0,servoCmd)

      servoCmd = 470
      servoMove(pwm,Larm_chnl,0,servoCmd)

      servoCmd = 250
      servoMove(pwm,rbase_chnl,0,servoCmd)

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

def RobotEx():
    R_cmd = int(GetInput('Earm(0), Larm(1), Pinc(2), Rbase(3), Uarm(4),quit(5)'))

    if R_cmd == 5:
          print('abort')
          return
    else:
          print(R_cmd)
          R_pos = int(GetInput('Servo PWM value (150 - 600)'))
          servoMove(pwm,R_cmd,0,R_pos)
          return

# Take a picture
def TakePicture(FlipCount):
    date_now = datetime.now()
    date_now = date_now.strftime('%Y%m%d-%H%M%S')
    FlipSt = str(FlipCount)
    FlipStr = FlipSt.zfill(int(math.log10(NumFlips)+1)) # add leading zeros to FlipCount
    PicLabel = RunName + '_' + date_now +'_' + FlipStr 
    savePic(camera,PicLabel)
    return(PicLabel)

# Main Dice Loop
def DiceLoop():
    sleep(WaitTime)
    FlipCount=1  
    plateCmd = plateservoMin
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
        if plateCmd == plateservoMin:
            plateCmd = plateservoMax
        else:
            plateCmd = plateservoMin
        #Flip Box now    
        servoMove(pwm,plate_channel,0,plateCmd)
        print(plateCmd)
        sleep(WaitTime) # Let die settle in box
        
        # Display count
        print ('Flip #' + str(FlipCount))
        FlipCount+=1

    # End Main Loop

# MatPlotLib plotting


def Initialize():    
    # Initialize Robot Arm
    RobotHome()

    # TBD - Check with Camera    
    print('Robot Initialized')

    # 0. (If on first run, pick up die and move to tower)

###
### Main Loop
###
    
## Initialize servos, camera, etc.
camera=PiCamera()

# Initialise the PCA9685 using the default address (0x40).
pwm = PWM(0x40)

# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)


pwm.setPWMFreq(60) # Set frequency to 60 Hz

## Get run parameters - # flips, die type, etc.
DiceType = GetInput('Dice Type')
NumFlips = int(GetInput('Number of Flips'))
WaitTime = 2 # Pause before flip
DiceNotes = GetInput('Dice Description')
RunName = GetInput('Run Name')
date_now = datetime.now()
date_now = date_now.strftime('%Y%m%d-%H%M%S')

FlipCount=0

while FlipCount <= NumFlips:
# 1. Take null picture and do validity checks
    NullPhoto = TakePicture(FlipCount)
# 2. Register points
# 3. Drop die
    servoCmd = Pinc_servoMax
    servoMove(pwm,Pinc_chnl,0,servoCmd)
# 4. Take picture
    DiePhoto = TakePicture(FlipCount)
# 5. Image analysis
# 6. Log result
# 7. Calculate die orientation
# 8. Plan motion path
# 9. Execute motion to pick up die and return to tower

# Repeat unless error flag is thrown or run is over

    print('Throws in work')
    FlipCount = FlipCount+1
# Robot Exploration Loop
cont = 'y'

while cont == 'y':
      RobotEx()
      cont = GetInput('Continue (y/n)?')

# Home the Robot

## TBD - Put die away
RobotHome()

# Update stats/distribution
print ('Run Complete')

# Write log file
