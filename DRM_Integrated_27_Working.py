# -*- coding: utf-8 -*-
"""
#################
DRM_Integrated_27
#################

This module will control the plate and robot servos, take pictures, do image processing and
log data and statistics for a given set of run parameters.

Notes:
    1. All dimensions in cm.

Revision History:
    Rev A - 7/12/2018: Combined code from DRM_Imaging_27 and DRM_Control_Tower_Robot_27

@author: Dave Wine
"""
### Libraries
import os
import re
import math
import sys
import csv
import collections
import pprint as pp

import scipy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
#import pandas_datareader as pdr
#import matplotlib as plt

from datetime import datetime
import time
from time import sleep

# Imaging modules
import cv2

from PIL import Image, ImageChops, ImageOps,ImageEnhance
from skimage import data, color, exposure
from skimage.util import img_as_ubyte, img_as_float

### Functions

def Angle_to_SC(SC_Vec):
    SC_Vec_m = (float(SC_Vec[2]) - float(SC_Vec[3])) / (float(SC_Vec[0]) - float(SC_Vec[1]) ) 
    SC_Vec_b = float(SC_Vec[2])-SC_Vec_m*(float(SC_Vec[0])-float(SC_Vec[4]))
    return SC_Vec_m,SC_Vec_b

def Get_Loc(filename):

    default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\diff21.jpg'

    src = cv.imread(filename, cv.IMREAD_GRAYSCALE)
    src_col = cv.imread(filename)
    
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
    #    return -1
    
    # Contours
    retval, threshold = cv.threshold(src,60, 255, cv.THRESH_BINARY)  
    srcc, contours, hierarchy = cv.findContours(threshold,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        
    # from https://www.quora.com/How-I-detect-rectangle-using-OpenCV
    C_count=0
    S_count=0
    for index, cnt in enumerate(contours):
        print 'Contour',index, 'has length',len(cnt)
        if len(cnt) > 10:
            approx = cv.approxPolyDP(cnt,0.03*cv.arcLength(cnt,True),True)
            print 'Approximation Length', len(approx)
            if len(approx)==4:
                print "square"
                cv.drawContours(src_col,[cnt],0,(255,0,0),2)
                S_count = S_count+1

                # Find centroid
                M=cv.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print 'Centroid:',cx,cy
                
                #Find angle
                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                print 'Bounding Box Coordinates:',box
                cv.drawContours(src_col,[box],0,(0,128,255),1)
                try:
                    slope = (box[1][1]-box[0][1])/(box[1][0]-box[0][0])
                except:
                    slope = 0.0
                    print 'infinite slope - reset to 0'
                Die_Angle = math.degrees(math.atan(slope))
                print 'Angle:' , Die_Angle
                
            elif len(approx) > 4:
                #area=cv.contourArea(cnt)
                x,y,w,h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/float(h)
                print 'Aspect Ratio: ',aspect_ratio
                if abs(aspect_ratio - 1) < 0.2: #check to see if it's really a circle
                    print "circle"
                    C_count= C_count+1
                    cv.drawContours(src_col,[cnt],0,(0,255,255),2)
                    
                else:
                    cv.drawContours(src_col,[cnt],0,(64,128,64),8)    
                    
            else:
                print 'contour',index,' is complex - in red'
                cv.drawContours(src_col,[cnt],0,(0,0,255),1)

            cv.imshow("Contours", src_col)
            #cv.waitKey()

        else:
            print 'contour length tiny - ignored (in orange)'
            cv.drawContours(src_col,[cnt],0,(0,128,255),8)
            
    #cv.waitKey()
    cv.destroyAllWindows()
    
    if S_count != 1:
        cx=0
        cy=0
        Die_Angle=0
        C_count=-100
            
    return (cx,cy,Die_Angle,C_count,S_count)

def diff_images(fileempty,file):
    img1=Image.open(fileempty)
    img2=Image.open(file)
    
    img1=img1.crop((685,50,1006,756))
    img2=img2.crop((685,50,1006,756))
        
    diff12=ImageChops.subtract(img1,img2)
    diff21=ImageChops.subtract(img2,img1)
    
#    # Convert to Grayscale
#    diff12 = rgb2gray(diff12)
#    diff21 = rgb2gray(diff21)
#    
#    # Increase Gamma a lot
#    # Increase contrast a bit
##    
#    contr1 = ImageEnhance.Contrast(img1)    
#    contr2 = ImageEnhance.Contrast(img2)
#    
#    img1=contr1.enhance(20)
#    img2=contr2.enhance(20)
    
    diff12.save(ImDir+"diff12.jpg")
    diff21.save(ImDir+"diff21.jpg")
    print ('differential image created')

def For_Kin(LAX,LAZ,UAX,EAX,EAZ,PZ,th1,th2,th3):
    ## Forward Kinematics Testing - Modify PT based on input

    # Define PT from dataframe
    PT = [[0,math.radians(90.0),LAX,LAZ],[0,0,UAX,0],[0,math.radians(-90.0),EAX,0],[0,0,0,PZ]]

    Theta1 = th1 # Rotation of Base
    Theta2 = th2 # Larm Rotation
    Theta3 = th3 # Uarm Rotation

    PT = [[0.0 + math.radians(Theta1),math.radians(90.0),LAX,LAZ],[0.0 + math.radians(Theta2),0.0,UAX,0.0],[0.0 - math.radians(Theta3),
           math.radians(90.0),UAX,0.0],[0.0,0.0,0.0,PZ]]

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
    print 'Forward Kinematics End Point: ',EndPt[0:3],'\n'
    return EndPt[0:3]

def GetInput(Caption):
    # Dumb user input grab
    input_name = raw_input(Caption + ': ')   
    return input_name

def IK_Calc(XB,YB,ZB):
        
    a1_IK = L_arm_Z
    a2_IK = U_arm_X
    a3_IK = E_arm_X
      
    r1_IK = math.hypot(XB,YB)
    r2_IK = ZB-a1_IK
    r3_IK = math.hypot(r1_IK,r2_IK)
    
    phi1_IK = math.acos((a3_IK**2-a2_IK**2-r3_IK**2)/(-2*a2_IK*r3_IK))
    phi2_IK = math.atan2(r2_IK,r1_IK)
    phi3_IK = math.acos((r3_IK**2-a2_IK**2-a3_IK**2)/(-2*a2_IK*a3_IK))
    
    phi1_IK_d = math.degrees(phi1_IK)
    phi2_IK_d = math.degrees(phi2_IK)
    phi3_IK_d = math.degrees(phi3_IK)
    
    Theta1_IK = math.atan2(YB,XB)
    Theta2_IK = phi1_IK + phi2_IK
    Theta3_IK = (math.pi) - phi3_IK - Theta2_IK
    
    Theta1_IK_d = math.degrees(Theta1_IK)
    Theta2_IK_d = math.degrees(Theta2_IK)
    Theta3_IK_d = math.degrees(Theta3_IK)
    
    return Theta1_IK_d,Theta2_IK_d,Theta3_IK_d

def Poly345(tau):
    # 5th-order polynomial fit from Angeles
    s_tau = 6*tau**5-15*tau**4+10*tau**3
    return(s_tau)

def Poly4567(tau):
    # 7th-order polynomial fit from Angeles (no jerk)
    s_tau = -20*tau**7+70*tau**6-84*tau**5+35*tau**4
    return(s_tau)

def readcsv(fname):
    vname = pd.DataFrame(pd.read_csv(fname,na_values='n/a'))
    return vname
    
def RobotEx():
    # Simple manual single-channel robot command function
    R_cmd = int(GetInput('Earm(0), Larm(1), Pinc(2), Rbase(3), Uarm(4),quit(5)'))

    if R_cmd == 5:
          print('abort')
          return
    else:
          print(R_cmd)
          R_pos = int(GetInput('Servo PWM value (150 - 600)'))
          servoMove(pwm,R_cmd,0,R_pos)
          return
 
def RobotHome():
    if PiFlag == True:
    # Initialize Robot (parallel to long edge, fully extended)
        servoCmd = U_arm_SC_b
        servoMove(pwm,U_arm_Ch,0,servoCmd)
        
        servoCmd = L_arm_SC_b
        servoMove(pwm,L_arm_Ch,0,servoCmd)
        
        servoCmd = Rbase_SC_b
        servoMove(pwm,Rbase_Ch,0,servoCmd)
        
        servoCmd = E_arm_SC_b # centered
        servoMove(pwm,E_arm_Ch,0,servoCmd)

        servoCmd = Pinc_SC_b # closed
        servoMove(pwm,Pinc_Ch,0,servoCmd)
        
        # TBD - Check with Camera    
        print'Robot Initialized','\n'

def RobotMove_IK():
    IK_run = GetInput('(q)uit, (c)ontinue')

    if IK_run == 'q':
          print('abort')
          return
    else:
        user_input = raw_input("Input X, Y, Z (cm) in RB frame, and Pincer rotation (deg) and state (deg): ")
        input_list = user_input.split(',')
        numbers = [float(x.strip()) for x in input_list]
        IK_x = numbers[0]
        IK_y = numbers[1]
        IK_z = numbers[2]
        IK_rot = numbers[3]
        IK_pinc = numbers[4]
        return (IK_x,IK_y,IK_z,IK_rot,IK_pinc)


def Robot_Move(XC,YC,ZC,RC,PC):
    global Rbase_CurPos,L_arm_CurPos,U_arm_CurPos,E_arm_CurPos,Pinc_CurPos
    
    MoveRobot = True
    
    print 'Command To: ',XC,YC,ZC
    
    Th1,Th2,Th3 = IK_Calc(XC,YC,ZC)    
    
    print 'New IK theta angles (degrees):', Th1,Th2,Th3
    
    # Translate these into servo commands
    Rbase_NewPos = int(Th1 * Rbase_SC_m + Rbase_SC_b)
    L_arm_NewPos = int(Th2 * L_arm_SC_m + L_arm_SC_b)
    U_arm_NewPos = int(Th3 * U_arm_SC_m + U_arm_SC_b)
    
    # Do end of arm stuff
    E_arm_NewPos = int(R_command * E_arm_SC_m + E_arm_SC_b) #-Rbase_NewPos
    Pinc_NewPos = int(P_command * Pinc_SC_m + Pinc_SC_b)
    
    print 'New Position Servo Commands: ',Rbase_NewPos,L_arm_NewPos, U_arm_NewPos, E_arm_NewPos, Pinc_NewPos
    
    FK = For_Kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th1,Th2,Th3)
    
    # Check to make sure the robot isn't going to crash...
    if Rbase_NewPos < Rbase_SMin or Rbase_NewPos > Rbase_SMax:
        print'RBase IK out of bounds: ',Rbase_NewPos
        MoveRobot = False   
    if L_arm_NewPos < L_arm_SMin or L_arm_NewPos > L_arm_SMax:
        print 'L_arm IK out of bounds: ',L_arm_NewPos        
        MoveRobot = False
    if U_arm_NewPos < U_arm_SMin or U_arm_NewPos > U_arm_SMax:
        print 'U_arm IK out of bounds: ', U_arm_NewPos   
        MoveRobot = False
    if E_arm_NewPos < E_arm_SMin or E_arm_NewPos > E_arm_SMax:
        print 'E_arm IK out of bounds: ', E_arm_NewPos   
        MoveRobot = False
    if Pinc_NewPos < Pinc_SMin or Pinc_NewPos > Pinc_SMax:
        print 'Pinc IK out of bounds: ', Pinc_NewPos   
        MoveRobot = False
                
    if MoveRobot == True: # Only move if possible

        # Compare to current servo commands
        Rbase_Diff = Rbase_CurPos-Rbase_NewPos
        L_arm_Diff = L_arm_CurPos-L_arm_NewPos
        U_arm_Diff = U_arm_CurPos-U_arm_NewPos
        E_arm_Diff = E_arm_CurPos-E_arm_NewPos
        Pinc_Diff = Pinc_CurPos - Pinc_NewPos
        # Figure out dynamics
        
        # Run movement loop
        tstep = 10 # number of subdivisions
        T = 10 # Total time for move in seconds
        t_int = T/tstep
        
        # Make sure Gripper is open
        
        # Move Robot!
        # Calculate step increments (do this all first to speed up movement)
        PolyStep = []
        for i in range(0,tstep+1):
            PolyStep.append(Poly345(float(i)/float(tstep))) # Get intermediate steps per Angeles p.236
        print 'Motion Steps Defined'
        
        # Then move the robot with it    
        for i in range(0,tstep+1):   
            
            s = PolyStep[i]
            
            Theta_1_j = int(Rbase_CurPos+(Rbase_NewPos-Rbase_CurPos)*s)
            Theta_2_j = int(L_arm_CurPos+(L_arm_NewPos-L_arm_CurPos)*s)
            Theta_3_j = int(U_arm_CurPos+(U_arm_NewPos-U_arm_CurPos)*s) 
            Theta_4_j = int(E_arm_CurPos+(E_arm_NewPos-E_arm_CurPos)*s) 
            
            print 'Motion Step #:',i, Theta_1_j,Theta_2_j,Theta_3_j,Theta_4_j,s #,'\n'
            # Command Servos    
            if PiFlag == True:
                
                servoMove(pwm,Rbase_Ch,0,Theta_1_j)
                servoMove(pwm,L_arm_Ch,0,Theta_2_j)
                servoMove(pwm,U_arm_Ch,0,Theta_3_j)
                servoMove(pwm,E_arm_Ch,0,Theta_4_j)
    
            # Then open or close pincer appropriately
                servoMove(pwm,Pinc_Ch,0,P_command)        
    
        # Update position
        Rbase_CurPos = Rbase_NewPos
        L_arm_CurPos = L_arm_NewPos
        U_arm_CurPos = U_arm_NewPos
        E_arm_CurPos = E_arm_NewPos
        Pinc_CurPos = Pinc_NewPos
        print 'Pincer state updated to', Pinc_NewPos,'from', Pinc_CurPos
    
    else:
        print ('Invalid move parameters')

def servoMove(ServoName,channel, SetOn, SetOff):
    # Move Servo - from Adafruit
    ServoName.setPWM(channel, int(SetOn), int(SetOff))   # Change speed of continuous servo on channel O
    
def setServoPulse(channel, pulse):
    # Servo setup - from Adafruit
      pulseLength = 1000000                   # 1,000,000 us per second
      pulseLength /= 60                       # 60 Hz
      print ("%d us per period" % pulseLength)
      pulseLength /= 4096                     # 12 bits of resolution
      print ("%d us per bit" % pulseLength)
      pulse *= 1000
      pulse /= pulseLength
      pwm.setPWM(channel, 0, pulse)

def SC_to_Angle(CurPos,b,m):
    # Convert servo command to angle (degrees)
    Angle = (CurPos - b)/m
    return Angle
    
# Convert 2D coordinates
def ThreeD_conv(P_i,th):
    th = math.radians(th)
    RotMax = [[np.cos(th),-np.sin(th),0],[np.sin(th),np.cos(th),0],[0,0,1]]
    P_0 = np.dot(P_i,RotMax)
    return P_0

# Take Picture
def savePic(CamName,Text):
    # Save picture to Pi desktop
    CamName.start_preview()
    CamName.annotate_text = Text
    CamName.capture(PiDir + Text + '.jpg')
    CamName.stop_preview()
# Image Processing

# Take a picture
def TakePicture(RollCount):
    # Take a picture with the Pi camera and datestamp it
    if PiFlag is True:
        date_now = datetime.now()
        date_now = date_now.strftime('%Y%m%d-%H%M%S')
        RollSt = str(RollCount)
        RollStr = RollSt.zfill(int(math.log10(NumRolls)+1)) # add leading zeros to FlipCount
        PicLabel = RunName + '_' + date_now +'_' + RollStr 
        savePic(camera,PicLabel)
        return(PicLabel)
    else:
        print 'Camera routine executed'
        return('null photo')
# Main Dice flipping loop from Bernoulli Build
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

###############################################################################
### Main Code
###############################################################################

### Setup

print 'Dice Rolling Machine (Poisson Build)','\n'

## Get run parameters - # flips, die type, etc.

PiFlag = bool(int(GetInput('Pi (1) or PC (0)')))
print 'Raspberry Pi Enabled:',PiFlag,'\n'

DiceType = GetInput('Dice Type')
NumRolls = int(GetInput('Number of Flips'))
DiceNotes = GetInput('Dice Description')
RunName = GetInput('Run Name')
FileSave = GetInput ('Save Image Files?')

## Define other parameters
WaitTime = 2 # Pause before flip

date_now = datetime.now()
date_now = date_now.strftime('%Y%m%d-%H%M%S')

PCDir="S:\Dave\QH\BBP\Dice Rolling Machine\DRM-Poisson\\"
PiDir="/home/pi/Desktop/DRM/Images/"

### Initialize Main Loop
## Raspberry PI only
if PiFlag is True:

# For imaging on Pi we need this module...
    import cv2.cv as cv
# For servo control
#    import sys
    sys.path.append('/home/pi/Adafruit-Raspberry-Pi-Python-Code/Adafruit_PWM_Servo_Driver')
    from Adafruit_PWM_Servo_Driver import PWM
    
# Camera
    from picamera import PiCamera
    
    WKdir = PiDir
# Initialize servos, camera, etc.
    camera=PiCamera()
    
    # Initialise the PCA9685 using the default address (0x40).
    pwm = PWM(0x40)
    # Note if you'd like more debug output you can instead run:
    #pwm = PWM(0x40, debug=True)
    
    pwm.setPWMFreq(60) # Set frequency to 60 Hz
    
    ImDir=PiDir+"Images/"
    ConDir = PiDir + "Config"

else:
    ImDir=PCDir+"\Images\\"
    ConDir = PCDir + "\Config\\"

## Set up reference images and directories
    
#file_names = os.listdir(WKdir) # Get list of photo names    
os.chdir(ImDir)
file_names = os.listdir(ImDir) # Get list of photo names
#test_name = 'RevC_100_B_20171014-121945-079.jpg'
test_name = ImDir+'\\'+'null_20180818-125743_001.jpg'
#file_names = [test_name]
#EmptyFile = "RevC_Cal_Empty_01_20171014-094835.jpg"
#EmptyFile = "Average.jpg"
#DiceFile = "RevC_100_B_20171014-122503_067.jpg"

# Get Ground Truth for run

ConfigFile = 'Ground_Truth.csv'
Run_df=readcsv(ConDir + ConfigFile) # Read in Config File

Run_df.set_index('Parameter',inplace = True)
GT_df=Run_df.drop(Run_df.index[0:6])
GT_df['Value']=pd.to_numeric(GT_df['Value'])

    #GT_df=DHP_df.drop(Run_df.index[0:6])
    #DHP_df['Value']=pd.to_numeric(DHP_df['Value']

# First, initialize the robot
# Robot Configuration
os.chdir(ConDir)
DHP_file = 'DRM_Sainsmart_DHP.csv'
DHP_df=readcsv(DHP_file) # Read in Config File
DHP_df.set_index('Name',inplace = True)

# Zero_Th is the angle in the range of motion of the servo that is defined as zero in the local CS

# Robot Parameters
Rbase_BC = np.array(DHP_df['Rbase'][9:12]).T
Rbase_Ch = int(DHP_df['Rbase']['Channel'])
Rbase_SMin = DHP_df['Rbase'][2]
Rbase_SMax = DHP_df['Rbase'][3]

L_arm_Ch = int(DHP_df['L_arm']['Channel'])
L_arm_SMin = DHP_df['L_arm'][2]
L_arm_SMax = DHP_df['L_arm'][3]
L_arm_X = DHP_df['L_arm'][7]
L_arm_Z = DHP_df['L_arm'][8]

U_arm_Ch = int(DHP_df['U_arm']['Channel'])
U_arm_SMin = DHP_df['U_arm'][2]
U_arm_SMax = DHP_df['U_arm'][3]
U_arm_X = DHP_df['U_arm'][7]
U_arm_Z = DHP_df['U_arm'][8]

E_arm_Ch = int(DHP_df['E_arm']['Channel'])
E_arm_SMin = DHP_df['E_arm'][2]
E_arm_SMax = DHP_df['E_arm'][3]
E_arm_X = DHP_df['E_arm'][7]
E_arm_Z = DHP_df['E_arm'][8]

Pinc_Ch = int(DHP_df['Pinc']['Channel'])
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

Rbase_RB = Rbase_rot- Rbase_rot
Camera_RB = Camera_rot - Rbase_rot
Tower_RB = Tower_rot - Rbase_rot

print "Rbase, Camera, Tower locations in Rbase coords:"
print Rbase_RB,'\n'
print Camera_RB, '\n'
print Tower_RB, '\n'

# Get angle to servo command matrix
# Intercepts are zero servo points (home)
# Need to put zero-angles in for each CF

Rbase_SC = np.array(DHP_df['Rbase'][0:5]).T
print 'Rbase Servo Limits: ', Rbase_SC

Rbase_SC_m,Rbase_SC_b = Angle_to_SC(Rbase_SC)
print 'Rbase slope, intercept: ', Rbase_SC_m,Rbase_SC_b,'\n'

L_arm_SC = np.array(DHP_df['L_arm'][0:5]).T
L_arm_SC_m,L_arm_SC_b = Angle_to_SC(L_arm_SC)

U_arm_SC = np.array(DHP_df['U_arm'][0:5]).T
print 'Uarm Servo Limits: ', U_arm_SC
U_arm_SC_m,U_arm_SC_b = Angle_to_SC(U_arm_SC)
print 'Uarm slope, intercept: ', U_arm_SC_m,U_arm_SC_b,'\n'

E_arm_SC = np.array(DHP_df['E_arm'][0:5]).T
E_arm_SC_m,E_arm_SC_b = Angle_to_SC(E_arm_SC)

Pinc_SC = np.array(DHP_df['Pinc'][0:5]).T
Pinc_SC_m,Pinc_SC_b = Angle_to_SC(Pinc_SC)

# Intercept_Servo is the zero angle for all servos
Intercept_Servo = [Rbase_SC_b,L_arm_SC_b,U_arm_SC_b,E_arm_SC_b,Pinc_SC_b]

# Pinc is not a location - used only to grip/release
print 'Intercept Servo Location: ',Intercept_Servo

# Home the Robot
RobotHome()

## TBD - Put die away

# Initialize position variables
Rbase_CurPos = Rbase_SC_b
L_arm_CurPos = L_arm_SC_b
U_arm_CurPos = U_arm_SC_b
E_arm_CurPos = E_arm_SC_b
Pinc_CurPos = Pinc_SC_b

# Convert to angles and get FK
Th_rb = SC_to_Angle(Rbase_CurPos, Rbase_SC_b, Rbase_SC_m)
Th_la = SC_to_Angle(L_arm_CurPos, L_arm_SC_b, L_arm_SC_m)
Th_ua = SC_to_Angle(U_arm_CurPos, U_arm_SC_b, U_arm_SC_m)
Th_ea = SC_to_Angle(E_arm_CurPos, E_arm_SC_b, E_arm_SC_m)
Th_pc = SC_to_Angle(Pinc_CurPos, Pinc_SC_b, Pinc_SC_m)

print 'CurPos Angles: ',math.degrees(Th_rb),math.degrees(Th_la),math.degrees(Th_ua),'\n'

FK = For_Kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th_rb,Th_la,Th_ua)

print 'Please Zero positions of Rbase, L_arm, and U_arm','\n'
# Force user to set position before proceeding
print 'Current Servo Commands:', Rbase_CurPos, L_arm_CurPos, U_arm_CurPos, '\n'

## Inverse Kinematics - Input X,Y,Z in base frame
# input desired EE location in RB coordinates (get from camera)

#Initial Robot Position
X_command = 00.0
Y_command = 17.0
Z_command = 0.0
R_command = 0
P_command = 30 

Th1,Th2,Th3 = IK_Calc(X_command,Y_command,Z_command)

print ('Initial IK angles (degrees):',Th1,Th2,Th3)

FK = For_Kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th1,Th2,Th3)

## The robot should be out of frame at this point, so take a picture
EmptyFrame = TakePicture(0) # return filename

# Do some imaging on this picture to determine center of frame
# Then transform into robot coordinates to use below
# Also maybe get die coordinates

## Next, move the robot to the center of the camera field and take a picture
#Robot_Move()# Insert frame center coordinates here
CRFrame = TakePicture(0) # return filename

# eventually do robot parameter correction here, but hopefully it's close enough for now....

### IK Loop - functionalize this

cont = 'y'

#while cont == 'y':    
    # Get new position by asking the user
 #   X_command,Y_command,Z_command,R_command,P_command = RobotMove_IK()

  #  Robot_Move(X_command,Y_command,Z_command,R_command,P_command)

## Next, go pick up die from holding area and hover over dice tower

# Move 1: Robot's current position to die holding area
# Move 2: Pick die up (may need an image)
# Move 3: Move to tower

### Main Rolling Loop
RollCount = 0

print 'Starting Rolls'
RollCount = 1
while RollCount <= NumRolls:
# 1. Take null picture and do validity checks

    NullPhoto = TakePicture(RollCount)
# 2. Register points
    
# 3. Drop die
    if PiFlag is True:
        servoCmd = Pinc_SMax
        servoMove(pwm,Pinc_Ch,0,servoCmd)

# 4. Take a picture of it
    DiePhoto = TakePicture(RollCount)
    
# 5. Go image it and get pip count (or maybe do this later in batch)
    Pips = Get_Pips(DiePhoto,'null')
# 6. Log result
# 7. Calculate die orientation
# 8. Plan motion path
# 9. Execute motion to pick up die and return to tower

# Repeat unless error flag is thrown or run is over

    print 'Roll ',RollCount,' of ',NumRolls, 'successfully completed'
    # Elapsed time per roll?
    RollCount = RollCount+1    
#RobotEx()

# When all done with runs, go to data reduction
cont = GetInput('Continue to data reduction (y/n)?')
if cont <>'y':
    exit('Program Aborted')

# Update stats/distribution
print ('Run complete, proceeding to data reduction')

# Write log file

### Statistical Analysis Code

pc = {}
for filename in file_names:
    
    if filename[0]=='t': #.isdigit():
        DiceFile = ImDir+'\\'+filename
    else:
        print ('irrelevant file')
        continue
    # Do image preprocessing
    img = Image.open(DiceFile)
    print('File:',DiceFile)
    diff_images(test_name,DiceFile)
    
    DiceFile=ImDir+'diff21.jpg'
    img = ImageOps.grayscale(img)
    img_gamma = exposure.adjust_gamma(img_as_float(img),1.8)  
    img_cont = exposure.adjust_sigmoid(img_as_float(img),cutoff=0.5, gain=50)
    cv_image = img_as_ubyte(img_gamma)
    
    # Send to Line for location
    Die_Loc = Get_Loc(DiceFile)
    print Die_Loc

    print filename +': '+ str(Die_Loc[3])
    pc[filename]=Die_Loc[3]
    
# Sort by name
pc=collections.OrderedDict(sorted(pc.items()))
GT_df['Run_Results']=pc.values()

Bad_Runs = len(GT_df[GT_df.Run_Results==-100])
print 'Bad runs:',Bad_Runs

GT_df['Difference']=GT_df['Value']-GT_df['Run_Results']

# Get rid of bad runs
GT_df = GT_df.loc[GT_df['Run_Results']>0]

#Plot histogram

n, bins, patches = plt.hist([GT_df['Value'],GT_df['Run_Results']], \
    bins=[0,1,2,3,4,5,6,7], normed=1, color=['g','b'], alpha=0.75, \
    rwidth=0.5, align='left',cumulative=False)

plt.xlabel('Roll')
plt.ylabel('Probability')
plt.title('Histogram of Imaged Results')
#plt.text(2, .75, r'$\mu=100,\ \sigma=15$')
plt.axis([0, 7, 0, .25])
plt.grid(True)
plt.show()

n, bins, patches = plt.hist(GT_df['Difference'], \
    bins=[-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6], normed=0, color=['r'], alpha=0.75, \
     rwidth=0.8,align='left',cumulative=False)

plt.xlabel('Difference')
plt.ylabel('count')
plt.title('Histogram of Difference')
#plt.text(2, .75, r'$\mu=100,\ \sigma=15$')
plt.axis([-6, 6, 0, NumRolls])
plt.grid(True)
plt.show()

