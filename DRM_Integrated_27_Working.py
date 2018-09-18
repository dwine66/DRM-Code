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

from PIL import Image, ImageChops, ImageOps,ImageEnhance
from skimage import data, color, exposure
from skimage.util import img_as_ubyte, img_as_float

### Functions

def angle_to_sc(SC_Vec):
    SC_Vec_m = (float(SC_Vec[2]) - float(SC_Vec[3])) / (float(SC_Vec[0]) - float(SC_Vec[1]) ) 
    SC_Vec_b = float(SC_Vec[2])-SC_Vec_m*(float(SC_Vec[0])-float(SC_Vec[4]))
    return SC_Vec_m,SC_Vec_b

def get_contours(File_Name):

    File_Name = ImDir+File_Name
    default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\diff21.jpg'
    print 'Filename used:',File_Name
    src = cv.imread(File_Name, cv.IMREAD_GRAYSCALE)
    src_col = cv.imread(File_Name)
    
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        src = default_file
    
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
                x,y,w,h = cv.boundingRect(cnt)
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
            cv.waitKey()

        else:
            print 'contour length tiny - ignored (in orange)'
            cv.drawContours(src_col,[cnt],0,(0,128,255),8)
            
    cv.waitKey()
    cv.destroyAllWindows()
    
    if S_count != 1:
        cx=0
        cy=0
        Die_Angle=0
        C_count=-1
            
    return (cx,cy,Die_Angle,C_count,S_count)

def get_diff_image(Null_File,Roll_File):
    img1=Image.open(ImDir + Null_File + '.jpg')
    img2=Image.open(ImDir + Roll_File + '.jpg')
    
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

def get_user_input(Caption):
    # Dumb user input grab
    Return_String = raw_input(Caption + ': ')   
    return Return_String

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

def read_csv_to_df(filename):
    # reads a CSV file into a dataframe
    df_name = pd.DataFrame(pd.read_csv(filename,na_values='n/a'))
    return df_name
    
def robot_ex():
    # Simple manual single-channel robot command function
    R_cmd = int(GetInput('Earm(0), Larm(1), Pinc(2), Rbase(3), Uarm(4),quit(5)'))

    if R_cmd == 5:
          print('abort')
          return
    else:
          print(R_cmd)
          R_pos = int(GetInput('Servo PWM value (150 - 600)'))
          servo_move(pwm,R_cmd,0,R_pos)
          return
 
def robot_home():
    if PiFlag == True:
    # Initialize Robot (parallel to long edge, fully extended)
        servoCmd = U_arm_SC_b
        servo_move(pwm,U_arm_Ch,0,servoCmd)
        
        servoCmd = L_arm_SC_b
        servo_move(pwm,L_arm_Ch,0,servoCmd)
        
        servoCmd = Rbase_SC_b
        servo_move(pwm,Rbase_Ch,0,servoCmd)
        
        servoCmd = E_arm_SC_b # centered
        servo_move(pwm,E_arm_Ch,0,servoCmd)

        servoCmd = Pinc_SC_b # closed
        servo_move(pwm,Pinc_Ch,0,servoCmd)
        
        # TBD - Check with Camera    
        print'Robot Initialized','\n'

def robot_move_IK():
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


def robot_move(XC,YC,ZC,RC,PC):
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
        T_step = 10 # number of subdivisions
        T = 10 # Total time for move in seconds
        T_int = T/T_step
        
        # Make sure Gripper is open
        
        # Move Robot!
        # Calculate step increments (do this all first to speed up movement)
        PolyStep = []
        for i in range(0,T_step+1):
            PolyStep.append(Poly345(float(i)/float(T_step))) # Get intermediate steps per Angeles p.236
        print 'Motion Steps Defined'
        
        # Then move the robot with it    
        for i in range(0,T_step+1):   
            
            s = PolyStep[i]
            
            Theta_1_j = int(Rbase_CurPos+(Rbase_NewPos-Rbase_CurPos)*s)
            Theta_2_j = int(L_arm_CurPos+(L_arm_NewPos-L_arm_CurPos)*s)
            Theta_3_j = int(U_arm_CurPos+(U_arm_NewPos-U_arm_CurPos)*s) 
            Theta_4_j = int(E_arm_CurPos+(E_arm_NewPos-E_arm_CurPos)*s) 
            
            print 'Motion Step #:',i, Theta_1_j,Theta_2_j,Theta_3_j,Theta_4_j,s #,'\n'
            # Command Servos    
            if PiFlag == True:
                
                servo_move(pwm,Rbase_Ch,0,Theta_1_j)
                servo_move(pwm,L_arm_Ch,0,Theta_2_j)
                servo_move(pwm,U_arm_Ch,0,Theta_3_j)
                servo_move(pwm,E_arm_Ch,0,Theta_4_j)
    
            # Then open or close pincer appropriately
                servo_move(pwm,Pinc_Ch,0,P_command)        
    
        # Update position
        Rbase_CurPos = Rbase_NewPos
        L_arm_CurPos = L_arm_NewPos
        U_arm_CurPos = U_arm_NewPos
        E_arm_CurPos = E_arm_NewPos
        Pinc_CurPos = Pinc_NewPos
        print 'Pincer state updated to', Pinc_NewPos,'from', Pinc_CurPos
    
    else:
        print ('Invalid move parameters')

def servo_move(Servo_Name,Channel, SetOn, SetOff):
    # Move Servo - from Adafruit
    Servo_Name.set_pwm(Channel, int(SetOn), int(SetOff))   # Change speed of continuous servo on channel O
    
def setServoPulse(Channel, pulse):
    # Servo setup - from Adafruit
      pulseLength = 1000000                   # 1,000,000 us per second
      pulseLength /= 60                       # 60 Hz
      print ("%d us per period" % pulseLength)
      pulseLength /= 4096                     # 12 bits of resolution
      print ("%d us per bit" % pulseLength)
      pulse *= 1000
      pulse /= pulseLength
      pwm.set_pwm(Channel, 0, pulse)

def sc_to_angle(CurPos,B,M):
    # Convert servo command to angle (degrees)
    Angle = (CurPos - B)/M
    return Angle
    
# Convert 2D coordinates
def three_d_conv(P_i,Th):
    Th = math.radians(Th)
    RotMax = [[np.cos(Th),-np.sin(Th),0],[np.sin(Th),np.cos(Th),0],[0,0,1]]
    P_0 = np.dot(P_i,RotMax)
    return P_0

# Take Picture
def save_picture(Cam_Name,Text):
    # Save picture to Pi desktop
    Cam_Name.start_preview()
    Cam_Name.annotate_text = Text
    Cam_Name.capture(ImDir + Text + '.jpg')
    Cam_Name.stop_preview()
# Image Processing

# Take a picture
def take_picture(Count,Label):
    # Take a picture with the Pi camera and datestamp it
    if PiFlag is True:
        date_now = datetime.now()
        date_now = date_now.strftime('%Y%m%d-%H%M%S')
        Roll_St = str(Count)
        Roll_Str = Roll_St.zfill(int(math.log10(Num_Rolls)+1)) +'_'+Label # add leading zeros to Count
        Pic_Label = File_Header + '_' + Roll_Str 
        save_picture(camera,Pic_Label)
        return(Pic_Label)
    else:
        print 'Camera routine executed'
        return('null photo')
# Main Dice flipping loop from Bernoulli Build
def Dice_Loop():
    sleep(WaitTime)
    FlipCount=1  
    plateCmd = plateservoMin
    while FlipCount <= NumFlips:
    ## Take and save picture
        date_now = datetime.now()
        date_now = date_now.strftime('%Y%m%d-%H%M%S')
        FlipSt = str(FlipCount)
        FlipStr = FlipSt.zfill(int(math.log10(NumFlips)+1)) # add leading zeros to FlipCount
        PicLabel = Run_Name + '_' + date_now +'_' + FlipStr 
        savePic(camera,PicLabel)
        # Image processing
        sleep(WaitTime)  
        ## Flip Box
        if plateCmd == plateservoMin:
            plateCmd = plateservoMax
        else:
            plateCmd = plateservoMin
        #Flip Box now    
        servo_move(pwm,plate_channel,0,plateCmd)
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

print '\n','Dice Rolling Machine (Poisson Build)','\n'

## Get run parameters - # flips, die type, etc.

PiFlag = bool(int(get_user_input('Pi (1) or PC (0)')))
print 'Raspberry Pi Enabled:',PiFlag,'\n'

date_now = datetime.now()
date_now = date_now.strftime('%Y%m%d-%H%M%S')


PCDir="S:\Dave\QH\BBP\Dice Rolling Machine\DRM-Poisson\\"
PiDir="/home/pi/Desktop/DRM/"

## Raspberry PI only
if PiFlag is True:

# For servo control
    sys.path.append('/home/pi/Adafruit-Raspberry-Pi-Python-Code/Adafruit_PWM_Servo_Driver')
    sys.path.append('/home/pi/Adafruit_PCA9685/PCA9685')
    #from Adafruit_PWM_Servo_Driver import PWM
    import Adafruit_PCA9685
    
# Define Working Directories
    WKdir = PiDir    
    ImDir = PiDir + "Images/"
    ConDir = PiDir + "Config/"
#    import cv2.cv as cv # For imaging on Pi we need this module    
    import cv2 as cv
    import cv2.cv as cv1
    from picamera import PiCamera 

# Initialize servos, camera, etc.
    camera = PiCamera()
    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685()
    # Note if you'd like more debug output you can instead run:
    #pwm = PWM(0x40, debug=True)
    pwm.set_pwm_freq(60) # Set frequency to 60 Hz

else:
    import cv2 as cv
    ImDir = PCDir+"\Images\\"
    ConDir = PCDir + "\Config\\"

# Get Config File
Config_File = 'POIS_3_Save.csv'
print 'Using Configuration File: ',Config_File
Config = read_csv_to_df(ConDir + Config_File) # Read in Config File   
Config.set_index('Parameter',inplace = True)  

Dice_Type = Config.loc['Dice Type']
Num_Rolls = int(Config.loc['Rolls'])
Wait_Time = float(Config.loc['Wait Time'])
Config_Notes = Config.loc['Notes']
Image_Save =  Config.loc['Image Save']

File_Header = Config_File[0:-4] + '_' + date_now

print 'File Header for this run:',File_Header

### Initialize the robot

# Load Robot Configuration File -  DHP and location parameters
os.chdir(ConDir)
DHP_File = 'DRM_Sainsmart_DHP.csv'
DHP = read_csv_to_df(DHP_File) # Read in Config File
DHP.set_index('Name',inplace = True)

# Zero_Th is the angle in the range of motion of the servo that is defined as zero in the local CS

# Robot Parameters
Rbase_BC = np.array(DHP['Rbase'][9:12]).T
Rbase_Ch = int(DHP['Rbase']['Channel'])
Rbase_SMin = DHP['Rbase'][2]
Rbase_SMax = DHP['Rbase'][3]

L_arm_Ch = int(DHP['L_arm']['Channel'])
L_arm_SMin = DHP['L_arm'][2]
L_arm_SMax = DHP['L_arm'][3]
L_arm_X = DHP['L_arm'][7]
L_arm_Z = DHP['L_arm'][8]

U_arm_Ch = int(DHP['U_arm']['Channel'])
U_arm_SMin = DHP['U_arm'][2]
U_arm_SMax = DHP['U_arm'][3]
U_arm_X = DHP['U_arm'][7]
U_arm_Z = DHP['U_arm'][8]

E_arm_Ch = int(DHP['E_arm']['Channel'])
E_arm_SMin = DHP['E_arm'][2]
E_arm_SMax = DHP['E_arm'][3]
E_arm_X = DHP['E_arm'][7]
E_arm_Z = DHP['E_arm'][8]

Pinc_Ch = int(DHP['Pinc']['Channel'])
Pinc_SMin = DHP['Pinc'][2]
Pinc_SMax = DHP['Pinc'][3]
Pinc_X = DHP['Pinc'][7]
Pinc_Z = DHP['Pinc'][8]

# Tower Location Data
Tower_BC = np.array(DHP['Tower'][9:12]).T

# CCS Location Data (Camera Coordinate System)
Camera_BC = np.array(DHP['Camera'][9:12]).T
CCS_Th = DHP['Camera'][4]

## Convert Base coordinates to Robot Base coordinates
# Rotation, then translation
Rbase_rot = three_d_conv(Rbase_BC,CCS_Th)
Camera_rot = three_d_conv(Camera_BC,CCS_Th)
Tower_rot = three_d_conv(Tower_BC,CCS_Th)

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

Rbase_SC = np.array(DHP['Rbase'][0:5]).T
print 'Rbase Servo Limits: ', Rbase_SC

Rbase_SC_m,Rbase_SC_b = angle_to_sc(Rbase_SC)
print 'Rbase slope, intercept: ', Rbase_SC_m,Rbase_SC_b,'\n'

L_arm_SC = np.array(DHP['L_arm'][0:5]).T
L_arm_SC_m,L_arm_SC_b = angle_to_sc(L_arm_SC)

U_arm_SC = np.array(DHP['U_arm'][0:5]).T
print 'Uarm Servo Limits: ', U_arm_SC

U_arm_SC_m,U_arm_SC_b = angle_to_sc(U_arm_SC)
print 'Uarm slope, intercept: ', U_arm_SC_m,U_arm_SC_b,'\n'

E_arm_SC = np.array(DHP['E_arm'][0:5]).T
E_arm_SC_m,E_arm_SC_b = angle_to_sc(E_arm_SC)

Pinc_SC = np.array(DHP['Pinc'][0:5]).T
Pinc_SC_m,Pinc_SC_b = angle_to_sc(Pinc_SC)

# Intercept_Servo is the zero angle for all servos
Intercept_Servo = [Rbase_SC_b,L_arm_SC_b,U_arm_SC_b,E_arm_SC_b,Pinc_SC_b]

# Pinc is not a location - used only to grip/release
print 'Intercept Servo Location: ',Intercept_Servo

# Home the Robot
robot_home()

## TBD - Put die away

# Initialize position variables
Rbase_CurPos = Rbase_SC_b
L_arm_CurPos = L_arm_SC_b
U_arm_CurPos = U_arm_SC_b
E_arm_CurPos = E_arm_SC_b
Pinc_CurPos = Pinc_SC_b

# Convert to angles and get FK
Th_rb = sc_to_angle(Rbase_CurPos, Rbase_SC_b, Rbase_SC_m)
Th_la = sc_to_angle(L_arm_CurPos, L_arm_SC_b, L_arm_SC_m)
Th_ua = sc_to_angle(U_arm_CurPos, U_arm_SC_b, U_arm_SC_m)
Th_ea = sc_to_angle(E_arm_CurPos, E_arm_SC_b, E_arm_SC_m)
Th_pc = sc_to_angle(Pinc_CurPos, Pinc_SC_b, Pinc_SC_m)

print 'CurPos Angles: ',math.degrees(Th_rb),math.degrees(Th_la),math.degrees(Th_ua),'\n'

FK = For_Kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th_rb,Th_la,Th_ua)

print 'Please Zero positions of Rbase, L_arm, and U_arm','\n'
# Force user to set position before proceeding
print 'Current Servo Commands:', Rbase_CurPos, L_arm_CurPos, U_arm_CurPos, '\n'

## Inverse Kinematics - Input X,Y,Z in base frame
# input desired EE location in RB coordinates (get from camera)

#Initial Robot Position
X_command = 0.0
Y_command = 17.0
Z_command = 0.0
R_command = 0
P_command = 30 

Th1,Th2,Th3 = IK_Calc(X_command,Y_command,Z_command)

print 'Initial IK angles (degrees):',Th1,Th2,Th3

FK = For_Kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th1,Th2,Th3)

## The robot should be out of frame at this point, so take a picture
EmptyFrame = take_picture(0,'Empty') # return filename

# Do some imaging on this picture to determine center of frame
# Then transform into robot coordinates to use below
# Also maybe get die coordinates

## Next, move the robot to the center of the camera field and take a picture
#Robot_Move()# Insert frame center coordinates here
CRFrame = take_picture(0,'Robot') # return filename

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

print 'Starting Rolls'
Roll_Count = 1

while Roll_Count <= Num_Rolls:
    # 1. Take null picture and do validity checks
    Null_Photo = take_picture(Roll_Count,'Null')
    
    # 2. Register points - TBD
    # 3. Drop die
    if PiFlag is True:
        servoCmd = Pinc_SMax
        servo_move(pwm,Pinc_Ch,0,servoCmd)

    # 4. Take a picture of it
        Null_Input= get_user_input('Place die and continue')
        Roll_Photo = take_picture(Roll_Count,'Die')
        
    # 5. Go image it and get pip count (or maybe do this later in batch)
        Diff_Photo = get_diff_image(Null_Photo,Roll_Photo) 
    
    Contour_Data = get_contours('diff21.jpg')
    print 'Estimated Pip Count:', Contour_Data[3]
    # 6. Log result
    # 7. Calculate die orientation
    # 8. Plan motion path
    # 9. Execute motion to pick up die and return to tower

    # Repeat unless error flag is thrown or run is over

    print 'Roll',Roll_Count,'of',Num_Rolls, 'successfully completed'
    # Elapsed time per roll?
    Roll_Count = Roll_Count + 1
    
# robot_ex()

# When all done with runs, go to data reduction
cont = get_user_input('Continue to data reduction (y/n)?')
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
plt.axis([-6, 6, 0, Num_Rolls])
plt.grid(True)
plt.show()

