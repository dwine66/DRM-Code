# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 05:47:18 2017

Initiated on 2/11/2018

DRM_Control

This module will control the plate and robot servos, take pictures, do image processing and
log data and statistics for a given set of run parameters.

Notes:
1. All dimensions in cm.

Revision History
Rev A: 3/17/2018 - Added Pincer servo, reset for Rev C Table

@author: Dave
"""
### Libraries
import os
import math
import pandas as pd
#import pandas_datareader as pdr
#import matplotlib as plt
import numpy as np
from datetime import datetime

# For servo control
#from Adafruit_PWM_Servo_Driver import PWM
import time

# Camera
#from picamera import PiCamera
from time import sleep

### Variables

### Functions
def readcsv(fname):
    vname = pd.DataFrame(pd.read_csv(fname,na_values='n/a'))
    return vname

def RobotHome():
      servoCmd = 380
      servoMove(pwm,U_arm_Ch,0,servoCmd)

      servoCmd = 470
      servoMove(pwm,L_arm_Ch,0,servoCmd)

      servoCmd = 250
      servoMove(pwm,Rbase_Ch,0,servoCmd)

# Input grab
def GetInput(Caption):
    input_name = input(Caption+': ')   
    return input_name

def Poly345(tau):
    s_tau = 6*tau**5-15*tau**4+10*tau**3
    return(s_tau)

def Poly4567(tau):
    s_tau = -20*tau**7+70*tau**6-84*tau**5+35*tau**4
    return(s_tau)
    
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
 
def RobotMove_IK():
    IK_run = GetInput('(q)uit, (c)ontinue')

    if IK_run == 'q':
          print('abort')
          return
    else:
        user_input = input("Input X, Y, Z (cm) in CCS frame: ")
        input_list = user_input.split(',')
        numbers = [int(x.strip()) for x in input_list]
        IK_x = numbers[0]
        IK_y = numbers[1]
        IK_z = numbers[2]
        return (IK_x,IK_y,IK_z)

# Convert 2D coordinates
def ThreeD_conv(P_i,th):
    th = math.radians(th)
    RotMax = [[np.cos(th),-np.sin(th),0],[np.sin(th),np.cos(th),0],[0,0,1]]
    P_0 = np.dot(P_i,RotMax)
    return P_0

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
### Main Code
###

### Setup
    
## Read in data from files

# Camera & Image Configuration
#WKdir="S:\\Dave\QH\\BBP\\Dice Rolling Machine\\Python Code\\DRM_Images"
#file_names = os.listdir(WKdir) # Get list of photo names

# Robot Configuration
WKdir="C:\\Users\\Dave\\Desktop"
os.chdir(WKdir)
DHP_file = 'DRM_Sainsmart_DHP.csv'
DHP_df=readcsv(DHP_file) # Read in Config File
DHP_df.set_index('Name',inplace = True)

# Zero_Th is the angle in the range of motion of the servo that is defined as zero in the local CS

#GT_df=DHP_df.drop(Run_df.index[0:6])
#DHP_df['Value']=pd.to_numeric(DHP_df['Value']

# Robot Parameters
Rbase_BC = np.array(DHP_df['Rbase'][9:12]).T
Rbase_Ch = DHP_df['Rbase']['Channel']
Rbase_SMin = DHP_df['Rbase'][2]
Rbase_SMax = DHP_df['Rbase'][3]

L_arm_Ch = DHP_df['L_arm']['Channel']
L_arm_SMin = DHP_df['L_arm'][2]
L_arm_SMax = DHP_df['L_arm'][3]
L_arm_X = DHP_df['L_arm'][7]
L_arm_Z = DHP_df['L_arm'][8]

U_arm_Ch = DHP_df['U_arm']['Channel']
U_arm_SMin = DHP_df['U_arm'][2]
U_arm_SMax = DHP_df['U_arm'][3]
U_arm_X = DHP_df['U_arm'][7]
U_arm_Z = DHP_df['U_arm'][8]

E_arm_Ch = DHP_df['E_arm']['Channel']
E_arm_SMin = DHP_df['E_arm'][2]
E_arm_SMax = DHP_df['E_arm'][3]
E_arm_X = DHP_df['E_arm'][7]
E_arm_Z = DHP_df['E_arm'][8]

Pinc_Ch = DHP_df['Pinc']['Channel']
Pinc_SMin = DHP_df['Pinc'][2]
Pinc_SMax = DHP_df['Pinc'][3]
Pinc_X = DHP_df['Pinc'][7]
Pinc_Z = DHP_df['Pinc'][8]

# Tower Location Data
Tower_BC = np.array(DHP_df['Tower'][9:12]).T

# CCS Location Data (Camera Coordinate System)
Camera_BC = np.array(DHP_df['Camera'][9:12]).T
CCS_Th = DHP_df['Camera'][4]

# Convert Base coordinates to Robot Base coordinates
# Rotation, then translation
Rbase_rot = ThreeD_conv(Rbase_BC,CCS_Th)
Camera_rot = ThreeD_conv(Camera_BC,CCS_Th)
Tower_rot = ThreeD_conv(Tower_BC,CCS_Th)

Camera_RB = Camera_rot - Rbase_rot
Tower_RB = Tower_rot - Rbase_rot

print ("Rbase, Camera, Tower locations in Rbase coords:")
print (Rbase_rot)
print (Camera_RB)
print (Tower_RB)

# Get angle to servo command matrix
# Intercepts are zero servo points (home)
# Need to put zero-angles in for each CF

Rbase_SC = np.array(DHP_df['Rbase'][0:5]).T
Rbase_SC_m = (Rbase_SC[2]-Rbase_SC[3])/(Rbase_SC[0]-Rbase_SC[1])
Rbase_SC_b = int(Rbase_SC[2]-Rbase_SC_m*(Rbase_SC[0]-Rbase_SC[4]))

L_arm_SC = np.array(DHP_df['L_arm'][0:5]).T
L_arm_SC_m = (L_arm_SC[2]-L_arm_SC[3])/(L_arm_SC[0]-L_arm_SC[1])
L_arm_SC_b = int(L_arm_SC[2]-L_arm_SC_m*(L_arm_SC[0]-L_arm_SC[4]))

U_arm_SC = np.array(DHP_df['U_arm'][0:5]).T
U_arm_SC_m = (U_arm_SC[2]-U_arm_SC[3])/(U_arm_SC[0]-U_arm_SC[1])
U_arm_SC_b = int(U_arm_SC[2]-U_arm_SC_m*(U_arm_SC[0]-U_arm_SC[4]))

E_arm_SC = np.array(DHP_df['E_arm'][0:5]).T
E_arm_SC_m = (E_arm_SC[2]-E_arm_SC[3])/(E_arm_SC[0]-E_arm_SC[1])
E_arm_SC_b = int(E_arm_SC[2]-E_arm_SC_m*(E_arm_SC[0]-E_arm_SC[4]))

Pinc_SC = np.array(DHP_df['Pinc'][0:5]).T
Pinc_SC_m = (Pinc_SC[2]-Pinc_SC[3])/(Pinc_SC[0]-Pinc_SC[1])
Pinc_SC_b = int(Pinc_SC[2]-Pinc_SC_m*(Pinc_SC[0]-Pinc_SC[4]))

# Home_Servo is the zero angle for all servos - should be centered width-wise in camera frame and perp. to robot base.
Home_Servo = [Rbase_SC_b,L_arm_SC_b,U_arm_SC_b,E_arm_SC_b]
# Pinc is not a location - used only to grip/release

# Home the Robot
## TBD - Put die away

Rbase_CurPos = Rbase_SC_b
L_arm_CurPos = L_arm_SC_b
U_arm_CurPos = U_arm_SC_b

print ('Zero positions of Rbase, L_arm, and U_arm')
# Force user to set position before proceeding
print (Rbase_CurPos, L_arm_CurPos, U_arm_CurPos, '\n')
## Inverse Kinematics - Input X,Y,Z in base frame

# input desired EE location in RB coordinates (get from camera)
XB = 10.0
YB = 17.0
ZB = 10.0

## Loop this
cont = 'y'

while cont == 'y':
    XB,YB,ZB = RobotMove_IK()
    
    a1_IK = L_arm_Z
    a2_IK = U_arm_X
    a3_IK = E_arm_X
      
    r1_IK = math.hypot(XB,YB)
    r2_IK = ZB-a1_IK
    r3_IK = math.hypot(r1_IK,r2_IK)
    phi1_IK = math.acos((a3_IK**2-a2_IK**2-r3_IK**2)/(-2*a2_IK*r3_IK))
    phi2_IK = math.atan2(r2_IK,r1_IK)
    phi3_IK = math.acos((r3_IK**2-a2_IK**2-a3_IK**2)/(-2*a2_IK*a3_IK))
    
    Theta1_IK = math.atan2(YB,XB)
    Theta2_IK = phi1_IK+phi2_IK
    Theta3_IK = (math.pi)-phi3_IK
    
    Theta1_IK_d = math.degrees(Theta1_IK)
    Theta2_IK_d = math.degrees(Theta2_IK)
    Theta3_IK_d = math.degrees(Theta3_IK)
    
    print ('Current IK angles (degrees):')
    print (Theta1_IK_d,Theta2_IK_d,Theta3_IK_d)
    
    # Translate these into servo commands
    Rbase_NewPos = int(Theta1_IK_d*Rbase_SC_m+Rbase_SC_b)
    L_arm_NewPos = int(Theta1_IK_d*L_arm_SC_m+L_arm_SC_b)
    U_arm_NewPos = int(Theta1_IK_d*U_arm_SC_m+U_arm_SC_b)
    
    # Compare to current servo commands
    Rbase_Diff = Rbase_CurPos-Rbase_NewPos
    L_arm_Diff = L_arm_CurPos-L_arm_NewPos
    U_arm_Diff = U_arm_CurPos-U_arm_NewPos
    # Figure out dynamics
    
    # Run movement loop
    tstep=20 # number of subdivisions
    T = 10 # Total time for move in seconds
    t_int = T/tstep
    
    # Make sure Gripper is open
    
    # Move Robot!
    for i in range(0,tstep+1):
        
        s = Poly345(i/tstep) # Get intermediate steps per Angeles p.236
        
        Theta_1_j = int(Rbase_CurPos+(Rbase_NewPos-Rbase_CurPos)*s)
        Theta_2_j = int(L_arm_CurPos+(L_arm_NewPos-L_arm_CurPos)*s)
        Theta_3_j = int(U_arm_CurPos+(U_arm_NewPos-U_arm_CurPos)*s) 
        
        print (i, Theta_1_j,Theta_2_j,Theta_3_j)
        # Command Servos    
        '''
        servoMove(pwm,Rbase_Ch,0,Theta_1_j)
        servoMove(pwm,L_arm_Ch,0,Theta_2_j)
        servoMove(pwm,U_arm_Ch,0,Theta_3_j)
        '''
        # Rotate Gripper appropriately
        # Close Gripper

    cont = GetInput('Continue (y/n)?')

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

DiceNotes = GetInput('Dice Description')
RunName = GetInput('Run Name')

## Other Parameters
WaitTime = 2 # Pause before flip
date_now = datetime.now()
date_now = date_now.strftime('%Y%m%d-%H%M%S')

## Initialize Main Loop
FlipCount=0

while FlipCount <= NumFlips:
# 1. Take null picture and do validity checks
    NullPhoto = TakePicture(FlipCount)
# 2. Register points
# 3. Drop die
    servoCmd = Pinc_SMax
    servoMove(pwm,Pinc_Ch,0,servoCmd)
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



# Update stats/distribution
print ('Run Complete')

# Write log file

## Forward Kinematics Testing - Modify PT based on input

# Define PT from dataframe
PT = [[0,math.radians(90.0),L_arm_X,L_arm_Z],[0,0,U_arm_X,0],[0,math.radians(-90.0),E_arm_X,0],[0,0,0,Pinc_Z]]

Theta1 = 0.0 # Rotation of Base
Theta2 = 90.0 # Larm Rotation
Theta3 = -90.0 # Uarm Rotation

PT = [[0.0 + math.radians(Theta1),math.radians(90.0),L_arm_X,L_arm_Z],[0.0 + math.radians(Theta2),0.0,U_arm_X,0.0],[0.0 + math.radians(Theta3),
       math.radians(90.0),U_arm_X,0.0],[0.0,0.0,0.0,Pinc_Z]]

# Build HTMs
i = 0
HTM_0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

i = 1
HTM_1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

i = 2
HTM_2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

i = 3
HTM_3_4 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

HTM_0_2 = np.dot(HTM_0_1,HTM_1_2)
HTM_0_3 = np.dot(HTM_0_2,HTM_2_3)
HTM_0_4 = np.dot(HTM_0_3,HTM_3_4) # Location of point given in CF4 in CF0

Larm_Base = [0,0,0,1] # Location of Larm axis in CF2
Larm_End = [0,0,0,1] # Location of end of Larm in CF3
Uarm_End = [0,0,0,1] # Location of end of Uarm in CF4
Pinc_End = [0,0,0,1] # Location of Pincer end in CF5

EndPt = np.dot(HTM_0_4,Pinc_End)
print (EndPt)