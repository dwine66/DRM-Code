# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 05:47:18 2017

Initiated on 2/11/2018

DRM_Control

This module will control the plate and robot servos, take pictures, do image processing and
log data and statistics for a given set of run parameters.

Revision History
Rev TBD

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

def RobotEx():
    R_cmd = int(GetInput('rbase(1), Larm(2), Uarm(3), Earm(4),quit(0)'))

    if R_cmd == 0:
          print('abort')
          return
    else:
          print(R_cmd)
          R_pos = int(GetInput('Servo PWM value (150 - 600)'))
          servoMove(pwm,R_cmd,0,R_pos)
          return

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

## Initialize servos, camera, etc.
camera=PiCamera()

# Initialise the PCA9685 using the default address (0x40).
pwm = PWM(0x40)

# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)
# Plate Servo
plate_channel = 0
plateservoMin = 394  # Side 1 - Min pulse length (Default 150)
plateservoMax = 500 # Side 2 - Max pulse length (Default 600)

# Robot Servos

rbase_chnl = 1
rbase_servoMin = 150
rbase_servoMax = 600
rbase_servoDef = 350

Larm_chnl = 2
Larm_servoMin = 150
Larm_servoMax = 600
Larm_servoDef = 350

Uarm_chnl = 3
Uarm_servoMin = 150
Uarm_servoMax = 600
Uarm_servoDef = 350

Earm_chnl = 4
Earm_servoMin = 150
Earm_servoMax = 600
Earm_servoDef = 350

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
## Initialize Plate

servoCmd = plateservoMin # Always start on side #1
servoMove(pwm,plate_channel,0,servoCmd)

# Initialize Robot Arm

servoCmd = rbase_servoDef 
servoMove(pwm,rbase_chnl,0,servoCmd)

servoCmd = Larm_servoDef 
servoMove(pwm,Larm_chnl,0,servoCmd)

servoCmd = Uarm_servoDef 
servoMove(pwm,Uarm_chnl,0,servoCmd)

servoCmd = Earm_servoDef 
servoMove(pwm,Earm_chnl,0,servoCmd)

print('Robot Initialized')
# Robot Exploration Loop
cont = 'y'

while cont == 'y':
      RobotEx()
      cont = GetInput('Continue?')


# Update stats/distribution
print ('Run Complete')

# Write log file
