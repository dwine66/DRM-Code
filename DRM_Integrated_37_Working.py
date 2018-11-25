# -*- coding: utf-8 -*-
"""
#################
DRM_Integrated_37
#################

This module will control the plate and robot servos, take pictures, do image processing and
log data and statistics for a given set of run parameters.

Notes:
    1. All dimensions in cm.

Revision History:
    Rev A - 7/12/2018: Combined code from DRM_Imaging_27 and DRM_Control_Tower_Robot_27

@author: Dave Wine
"""
#######################################################
print('\n','Dice Rolling Machine (Poisson Build)','\n')
print('Dave Wine - BBP 2018','\n')

### Libraries
import os
import re
import math
import sys
import csv
import collections
import pprint as pp
import time
from datetime import datetime
from time import sleep

print('Basic libraries loaded')

import scipy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
#import pandas_datareader as pdr
#import matplotlib as plt

print('Math libraries loaded')

from PIL import Image, ImageChops, ImageOps,ImageEnhance
from skimage import data, color, exposure
from skimage.util import img_as_ubyte, img_as_float

print('Image libraries loaded')

### Functions

def angle_to_sc(SC_Vec):
    # Converts arm angles to servo counts
    SC_Vec_m = (float(SC_Vec[2]) - float(SC_Vec[3])) / (float(SC_Vec[0]) - float(SC_Vec[1]) ) 
    SC_Vec_b = float(SC_Vec[2]) - SC_Vec_m*(float(SC_Vec[0]) + float(SC_Vec[4]))
    return SC_Vec_m,SC_Vec_b # Output is in counts/deg

def for_kin(LAX,LAZ,UAX,EAX,EAZ,PZ,th1,th2,th3):
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
    print('Forward Kinematics End Point: ',EndPt[0:3],'\n')
    return EndPt[0:3]

def get_spots(File):
    print('Getting Spots')
    img1 = File
    hsv = cv.cvtColor(img1, cv.COLOR_BGR2HSV)
    # Color in BGR    
    lower_red = np.array([139,35,42])
    upper_red = np.array([255,255,255])
    
    mask = cv.inRange(hsv, lower_red, upper_red)
    #print mask
    res = cv.bitwise_and(img1,img1, mask = mask)

##    cv.imshow('frame',img1)
##    cv.imshow('mask',mask)
##    cv.imshow('res',res)  
##    cv.waitKey()
    cv.destroyAllWindows()
#   mask.save(ImDir+"mask.jpg")
    return(mask)

def get_refpoints(File_Name):
    # Gets crop boundaries for dice rolling zone
    print('Getting reference points..')
    #def main(argv):
    default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\POIS_3_Save_20181021-203930_0_Align.jpg'
    filename = ImDir + File_Name
    #filename = argv[0] if len(argv) > 0 else default_file
    # Loads an image
    src = cv.imread(filename, cv.IMREAD_GRAYSCALE)
    src_col = cv.imread(filename)
    
    # Check if image is loaded OK
    if src is None:
        print ('Error opening image - using default file')
        filename = default_file
        src_col = cv.imread(filename)

    Cal_Spots = get_spots(src_col)
    print('Spots returned')
    # Contours
    retval, threshold = cv.threshold(Cal_Spots, 60, 255, cv.THRESH_BINARY)  
    #print 'retval:',retval
    #print 'threshold:',threshold
    if CV_Ver == '2.4.9.1':
        contours, hierarchy = cv.findContours(threshold,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    else:
       srcc,contours, hierarchy = cv.findContours(threshold,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    print('Contours found:',len(contours))
    
    Num_Circles = 0
    Circles = pd.DataFrame(columns=['cnt','cx','cy','Area'])
    
    # from https://www.quora.com/How-I-detect-rectangle-using-OpenCV
    for index, cnt in enumerate(contours):
        #print 'Contour',index, 'has length',len(cnt)
        if len(cnt) > 10:
            approx = cv.approxPolyDP(cnt,0.03*cv.arcLength(cnt,True),True)
            #print 'Approximation Length for index',index,':', len(approx)
            if len(approx) == 3:
                #print "triangle"
                cv.drawContours(src_col,[cnt],0,(0,255,0),2)
            elif len(approx) == 4:
                #print "square"
                cv.drawContours(src_col,[cnt],0,(255,0,0),2)
            elif len(approx) > 4:
                #print "circle"
                Num_Circles += 1
                cv.drawContours(src_col,[cnt],0,(0,255,255),2)
                # Find centroid
                M = cv.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                C_Area = cv.contourArea(cnt)
                
                if C_Area > 600 and C_Area < 1500:
                    print('Centroid:',cx,'x, ',cy,'y,','Area:', C_Area, ' pixels')
                    rect = cv.minAreaRect(cnt)
                    if CV_Ver == '2.4.9.1':
                        box = cv.cv.BoxPoints(rect)
                    else:
                        box = cv.boxPoints(rect)
                    box = np.int0(box)
                    #print box
                    X_box = int((box[0,0] + box[2,0])/2)
                    Y_box = int((box[0,1] + box[2,1])/2)
                    print('Box Center:',X_box,Y_box)
                    
                    L_box_1 = math.hypot(box[0,0]-box[1,0],box[0,1]-box[1,1])
                    L_box_2 = math.hypot(box[1,0]-box[2,0],box[1,1]-box[2,1])
                    AR = L_box_1/L_box_2
                    print('Aspect Ratio', AR)
                    
                    X_abs = float(Y_box)/ImScale - Im_bias_Y
                    Y_abs = float(X_box)/ImScale - Im_bias_X
                    #print 'location in robot CS: ',X_abs,Y_abs
                    
                    cv.circle(src_col,(X_box,Y_box),4,(255,0,255),2,1)
                    cv.drawContours(src_col,[box],0,(255,0,255),3)
                    if abs(AR-1) < 0.05:
                        print('\n','Found Reference Point','\n')                     
                        Circles.loc[Num_Circles] = [index,X_box,Y_box,C_Area]
                        
                    #cv.imshow("Contours", src_col)
                    #cv.waitKey()                    
                #print 'contour',index,' is complex - in red'
                cv.drawContours(src_col,[cnt],0,(0,0,255),1)
        else:
            #print 'contour length tiny - ignored (in blue)'
            cv.drawContours(src_col,[cnt],0,(255,0,0),8)
    
    cv.imshow("Contours", src_col)

    # Define crop boundaries
    Y_crop_min = 0
    Y_crop_max = int(max(Circles['cy'])) - 20
    X_crop_min = int(min(Circles['cx'])) + 20
    X_crop_max = int(max(Circles['cx'])) - 20
    
    Crop_list = [X_crop_min,Y_crop_min, X_crop_max,Y_crop_max]
    print('Crop Boundaries:', Crop_list) 
    cv.rectangle(src_col,(X_crop_min,Y_crop_min),(X_crop_max,Y_crop_max),(255,255,255),2,4)
    cv.imshow("Contours", src_col)
    
    cv.waitKey()
    cv.destroyAllWindows()
    print(Num_Circles,'Circles Found')
    print(Circles)

    return(Circles,Crop_list)  
    
def get_contours(File_Name):
    # Get contours and geometry from image
    File_Name = ImDir + File_Name
    default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\diff21.jpg'
    print('Filename used:',File_Name)
    src = cv.imread(File_Name, cv.IMREAD_GRAYSCALE)
    src_col = cv.imread(File_Name)
    
    # Check if image loaded OK
    if src is None:
        print(('Error opening image! Opening ' + default_file + '] \n'))
    
    # Contours
    retval, threshold = cv.threshold(src,60, 255, cv.THRESH_BINARY)  
    
    if CV_Ver == '2.4.9.1':
       contours, hierarchy = cv.findContours(threshold,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    else:
       srcc,contours, hierarchy = cv.findContours(threshold,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    
    print(len(contours),'Contours found')       
    # from https://www.quora.com/How-I-detect-rectangle-using-OpenCV
    C_count = 0 # Circle count
    S_count = 0 # Square count
    
    for index, cnt in enumerate(contours):

        #print 'Contour',index, 'has length',len(cnt)
        if len(cnt) > 10:
            approx = cv.approxPolyDP(cnt,0.03*cv.arcLength(cnt,True),True)
            #print 'Approximation Length', len(approx)
            # Find a square
            if len(approx) == 4:
                print("Square")
                cv.drawContours(src_col,[cnt],0,(255,0,0),2)
                S_count = S_count+1

                # Find centroid
                M = cv.moments(cnt)
                # Note that these are reversed to match CCS
                Centroid_X = int(M['m10']/M['m00'])
                Centroid_Y = int(M['m01']/M['m00'])
                print('Centroid:',Centroid_X, Centroid_Y)
                cv.circle(src_col,(Centroid_X,Centroid_Y),4,(255,128,128),2,1)
                
                #Find angle
                rect = cv.minAreaRect(cnt)
                if CV_Ver == '2.4.9.1':
                    box = cv.cv.BoxPoints(rect)
                else:
                    box = cv.boxPoints(rect)
                box = np.int0(box)
                print('Bounding Box Coordinates:',box)
                cv.drawContours(src_col,[box],0,(255,128,128),1)

                try:
                    slope = float(float(box[1][1]-box[0][1])/float(box[1][0]-box[0][0]))
                    print('Slope:',slope)
                except:
                    slope = 0.0
                    print('Infinite slope - reset to 0')
                    
                Die_Angle = math.degrees(math.atan(slope))
                print('Angle:' , Die_Angle)
                
            elif len(approx) > 4:
                # Assume it's a circle
                #area=cv.contourArea(cnt)
                x,y,w,h = cv.boundingRect(cnt)
                cv.rectangle(src_col,(x,y),(x+w,y+h),(0,255,0),2)
                aspect_ratio = float(w) / float(h)

                if abs(aspect_ratio - 1) < 0.2: # check to see if it's really a circle
                    C_count= C_count + 1
                    print('Found circle #',C_count, ' w/aspect ratio: ',aspect_ratio)
                    # draw it yellow
                    cv.drawContours(src_col,[cnt],0,(0,255,255),2)
                    #cv.imshow("Contours", src_col)        
                    #cv.waitKey()                   
                else:
                    # draw it red
                    print('not a circle')
                    cv.drawContours(src_col,[cnt],0,(0,0,255),8)    
                    
            else:
                #print 'Contour',index,' is complex - in red'
                cv.drawContours(src_col,[cnt],0,(0,0,255),1)

            #cv.imshow("Contours", src_col)
            #cv.waitKey()
            #sleep(1)

        else:
            #print 'Contour length tiny - ignored (in orange)'
            cv.drawContours(src_col,[cnt],0,(0,128,255),8)
            
    #cv.imshow("Contours", src_col)        
    #cv.waitKey()
    print('Completed image analysis')
    #sleep(1)
    cv.destroyAllWindows()

    if S_count != 1:
        Centroid_X = 200
        Centroid_Y = 200
        Die_Angle = 0
        C_count = -1
    print('cx, cy, angle:',Centroid_X,Centroid_Y,Die_Angle)
    return (Centroid_X,Centroid_Y,Die_Angle,C_count,S_count)

def get_diff_image(Null_File,Roll_File,Crop):
    img1=Image.open(ImDir + Null_File + '.jpg')
    img2=Image.open(ImDir + Roll_File + '.jpg')


    img1=img1.crop((Crop[0],Crop[1],Crop[2],Crop[3]))
    img2=img2.crop((Crop[0],Crop[1],Crop[2],Crop[3]))
        
    diff12=ImageChops.subtract(img1,img2)
    diff21=ImageChops.subtract(img2,img1)
    
    diff12.save(ImDir+"diff12.jpg")
    diff21.save(ImDir+"diff21.jpg")
    print ('Differential image created')
    
def get_user_input(Caption):
    # Dumb user input grab
    Return_String = input(Caption + ': ')   
    return Return_String

def ik_calc(XB,YB,ZB):
    # Returns joint angles from cartesian coordinates
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
    Theta3_IK = Theta2_IK - ((math.pi) - phi3_IK)
    
    Theta1_IK_d = math.degrees(Theta1_IK)
    Theta2_IK_d = math.degrees(Theta2_IK)
    Theta3_IK_d = math.degrees(Theta3_IK)
    
    return Theta1_IK_d,Theta2_IK_d,Theta3_IK_d

def read_csv_to_df(filename):
    # reads a CSV file into a dataframe
    df_name = pd.DataFrame(pd.read_csv(filename,na_values='n/a'))
    return df_name
    
def robot_move_IK():
    IK_run = get_user_input('Ready to move robot (y/n)?')

    if IK_run != 'y':
          print('abort')
          return
    else:
        user_input = input("Input X, Y, Z (cm) in RB frame, and Pincer rotation (deg) and state (deg): ")
        input_list = user_input.split(',')
        numbers = [float(x.strip()) for x in input_list]
        IK_x = numbers[0]
        IK_y = numbers[1]
        IK_z = numbers[2]
        IK_rot = numbers[3]
        IK_pinc = numbers[4]
        return (IK_x,IK_y,IK_z,IK_rot,IK_pinc)

def robot_init():
    Servo.close()
    print('maestro off')

def robot_move(XC,YC,ZC,RC,PC):
    global Rbase_CurPos,L_arm_CurPos,U_arm_CurPos,E_arm_CurPos,Pinc_CurPos,R_Accel,R_Speed
    
    MoveRobot = True
    
    print('Command To: ',XC,YC,ZC)
    
    # Get joint angles
    Th1,Th2,Th3 = ik_calc(XC,YC,ZC)    
    
    print('New IK theta angles (degrees):', Th1,Th2,Th3)
    
    # Translate joint angles into servo commands
    Rbase_NewPos = int(Th1 * Rbase_SC_m + Rbase_SC_b)
    L_arm_NewPos = int(Th2 * L_arm_SC_m + L_arm_SC_b)
    U_arm_NewPos = int(Th3 * U_arm_SC_m + U_arm_SC_b)
    
    # Do end of arm stuff
    E_arm_NewPos = int(RC * E_arm_SC_m + E_arm_SC_b) #-Rbase_NewPos
    Pinc_NewPos = int(PC * Pinc_SC_m + Pinc_SC_b)
    
    print('New Position Servo Commands: ',Rbase_NewPos,L_arm_NewPos, U_arm_NewPos, E_arm_NewPos, Pinc_NewPos)
    
    #FK = for_kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th1,Th2,Th3)
    
    # Check to make sure the robot isn't going to crash...
    if Rbase_NewPos < Rbase_SMin or Rbase_NewPos > Rbase_SMax:
        print('RBase IK out of bounds: ',Rbase_NewPos)
        MoveRobot = False   
    if L_arm_NewPos < L_arm_SMin or L_arm_NewPos > L_arm_SMax:
        print('L_arm IK out of bounds: ',L_arm_NewPos)        
        MoveRobot = False
    if U_arm_NewPos < U_arm_SMin or U_arm_NewPos > U_arm_SMax:
        print('U_arm IK out of bounds: ', U_arm_NewPos)   
        MoveRobot = False
    if E_arm_NewPos < E_arm_SMin or E_arm_NewPos > E_arm_SMax:
        print('E_arm IK out of bounds: ', E_arm_NewPos)   
        MoveRobot = False
    if Pinc_NewPos < Pinc_SMin or Pinc_NewPos > Pinc_SMax:
        print('Pinc IK out of bounds: ', Pinc_NewPos)   
        MoveRobot = False
                
    if MoveRobot == True: # Only move if possible

        # Compare to current servo commands
        Rbase_Diff = Rbase_CurPos-Rbase_NewPos
        L_arm_Diff = L_arm_CurPos-L_arm_NewPos
        U_arm_Diff = U_arm_CurPos-U_arm_NewPos
        E_arm_Diff = E_arm_CurPos-E_arm_NewPos
        Pinc_Diff = Pinc_CurPos - Pinc_NewPos
        # Figure out dynamics
        
##        # Run movement loop
##        T_step = 10 # number of subdivisions
##        T = 10 # Total time for move in seconds
##        T_int = T/T_step
        
        # Move Robot!
        Theta_1_j = int(Rbase_CurPos+(Rbase_NewPos-Rbase_CurPos))
        Theta_2_j = int(L_arm_CurPos+(L_arm_NewPos-L_arm_CurPos))
        Theta_3_j = int(U_arm_CurPos+(U_arm_NewPos-U_arm_CurPos)) 
        Theta_4_j = int(E_arm_CurPos+(E_arm_NewPos-E_arm_CurPos)) 
            
        print('Moving to:', Theta_1_j,Theta_2_j,Theta_3_j,Theta_4_j) #,'\n'
        # Command Servos    
        servo_move(Rbase_Ch,R_Accel,R_Speed, Theta_1_j)
        servo_move(L_arm_Ch,R_Accel,R_Speed, Theta_2_j)
        servo_move(U_arm_Ch,R_Accel,R_Speed, Theta_3_j)
        servo_move(E_arm_Ch,R_Accel,R_Speed, Theta_4_j)

    # Then open or close pincer appropriately
        servo_move(Pinc_Ch,R_Accel*2,R_Speed*2,Pinc_NewPos)        
        print('Pincer state updated to', Pinc_NewPos,'from', Pinc_CurPos)
                
        # Update position
        Rbase_CurPos = Rbase_NewPos
        L_arm_CurPos = L_arm_NewPos
        U_arm_CurPos = U_arm_NewPos
        E_arm_CurPos = E_arm_NewPos
        Pinc_CurPos = Pinc_NewPos
        
        print('CurPos Positions: ',Rbase_CurPos,L_arm_CurPos,U_arm_CurPos,E_arm_CurPos,Pinc_CurPos)
    
    else:
        print ('Invalid move parameters')

def robot_tweak():
    # Simple manual single-channel robot servo command function
    global Rbase_CurPos,L_arm_CurPos,U_arm_CurPos,E_arm_CurPos,Pinc_CurPos,R_Accel,R_Speed
    while get_user_input('Hit y to tweak') == 'y':
        
        R_cmd = int(get_user_input('Uarm(1), Larm(2), Pinc(5), Rbase(4), Earm(3), reset(6)'))

        if R_cmd == 6:
            print('reset robot')
            robot_init()
        else:
            print(R_cmd)
            R_pos = int(get_user_input('Input servo value'))
            servo_move(R_cmd,R_Accel,R_Speed,R_pos)
            if R_cmd == 3:
                E_arm_CurPos = R_pos
            elif R_cmd == 2:
                L_arm_CurPos = R_pos
            elif R_cmd == 5:
                Pinc_CurPos = R_pos          
            elif R_cmd == 4:
                Rbase_CurPos = R_pos
            elif R_cmd == 1:
                U_arm_CurPos = R_pos
        print(E_arm_CurPos,L_arm_CurPos,Pinc_CurPos,Rbase_CurPos,U_arm_CurPos)
    return

def save_picture(Cam_Name,Text):
    # Save picture to Pi desktop
    Cam_Name.start_preview()
    Cam_Name.annotate_text = ""
    Cam_Name.capture(ImDir + Text + '.jpg')
    Cam_Name.stop_preview()
    
def servo_move(n,a,s,p):
    Servo.setAccel(n,a)      #set servo n acceleration
    #print 'set speed'
    Servo.setSpeed(n,s)     #set speed of servo n
    #print 'moving servo', n
    Servo.setTarget(n,p)  #set servo n to move to position
    
def sc_to_angle(CurPos,B,M):
    # Convert servo command to angle (degrees)
    Angle = float((float(CurPos) - float(B))/float(M))
    return Angle

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
        print('Camera routine executed')
        return('null photo')

def three_d_conv(P_i,Th):
    # Convert 2D coordinates.  P_i is a three-vector
    Th = math.radians(Th)
    RotMax = [[np.cos(Th),-np.sin(Th),0],[np.sin(Th),np.cos(Th),0],[0,0,1]]
    P_0 = np.dot(P_i,RotMax)
    return P_0

###############################################################################
### Main Code
###############################################################################

### Setup

## Get run parameters - # flips, die type, etc.

PiFlag = bool(int(get_user_input('Pi (1) or PC (0)')))
print('Raspberry Pi Enabled:',PiFlag,'\n')

date_now = datetime.now()
date_now = date_now.strftime('%Y%m%d-%H%M%S')

print(date_now,'\n')

R_Accel = 4
R_Speed = 8

PCDir="S:\Dave\QH\BBP\Dice Rolling Machine\DRM-Poisson\\"
PiDir="/home/pi/Desktop/DRM/"

## Raspberry PI only
if PiFlag is True:

# For servo control
    sys.path.append('/home/pi/Pololu')
    import maestro
    print('Maestro loaded')

# Define Working Directories
    WKdir = PiDir    
    ImDir = PiDir+"Images/"
    ConDir = PiDir + "Config/"
 #   import cv2.cv as cv # For imaging on Pi we need this module    
    import cv2 as cv
   #   import cv2.cv as cv1
    from picamera import PiCamera 
    print('Opencv and PiCamera loaded')
# Initialize servo board and camera
    camera = PiCamera()
    Servo = maestro.Controller()

else:
    import maestro
    Servo = maestro.Controller('COM22')
    R_Accel = 4
    R_Speed = 4
    import cv2 as cv
    ImDir = PCDir+"\Images\\"
    ConDir = PCDir + "\Config\\"
    
CV_Ver = cv.__version__

# Get Config File
Config_File = 'POIS_3_Save.csv'
print('Using Configuration File: ',Config_File)
Config = read_csv_to_df(ConDir + Config_File) # Read in Config File   
Config.set_index('Parameter',inplace = True)  

Dice_Type = Config.loc['Dice Type']
Num_Rolls = int(Config.loc['Rolls'])
Wait_Time = float(Config.loc['Wait Time'])
Config_Notes = Config.loc['Notes']
Image_Save =  Config.loc['Image Save']

File_Header = Config_File[0:-4] + '_' + date_now

print('File Header for this run:',File_Header)

### Robot Kinematics Setup

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

ImScale = 39.04

#Reference tack in base coords
Im_bias_X = DHP['LR_Tack'][9]
Im_bias_Y = DHP['LR_Tack'][10]

print(' Image offsets:',Im_bias_X, Im_bias_Y)

## Convert Base coordinates to Robot Base coordinates
# Rotation, then translation
Rbase_rot = three_d_conv(Rbase_BC,CCS_Th)
Camera_rot = three_d_conv(Camera_BC,CCS_Th)
Tower_rot = three_d_conv(Tower_BC,CCS_Th)
Tack_rot = three_d_conv((Im_bias_X,Im_bias_Y,0),CCS_Th)

Rbase_RB = Rbase_rot - Rbase_rot
Camera_RB = Camera_rot - Rbase_rot
Tower_RB = Tower_rot - Rbase_rot
Tack_RB = Tack_rot - Rbase_rot

print("Rbase, Camera, Tower, Tack locations in Rbase coords:")
print(Rbase_RB,'\n')
print(Camera_RB, '\n')
print(Tower_RB, '\n')
print(Tack_RB, '\n')
# Get angle to servo command matrix
# Intercepts are zero servo points (home)
# Need to put zero-angles in for each CF

Rbase_SC = np.array(DHP['Rbase'][0:5]).T
print('Rbase Servo Limits: ', Rbase_SC)

Rbase_SC_m,Rbase_SC_b = angle_to_sc(Rbase_SC)
print('Rbase slope, intercept: ', Rbase_SC_m,Rbase_SC_b,'\n')

L_arm_SC = np.array(DHP['L_arm'][0:5]).T
print('L_arm Servo Limits: ', L_arm_SC)
L_arm_SC_m,L_arm_SC_b = angle_to_sc(L_arm_SC)
print('L_arm slope, intercept: ', L_arm_SC_m,L_arm_SC_b,'\n')

U_arm_SC = np.array(DHP['U_arm'][0:5]).T
print('Uarm Servo Limits: ', U_arm_SC)

U_arm_SC_m,U_arm_SC_b = angle_to_sc(U_arm_SC)
print('Uarm slope, intercept: ', U_arm_SC_m,U_arm_SC_b,'\n')

E_arm_SC = np.array(DHP['E_arm'][0:5]).T
E_arm_SC_m,E_arm_SC_b = angle_to_sc(E_arm_SC)
print('E_arm Servo Limits: ', E_arm_SC)
print('E_arm slope, intercept: ', E_arm_SC_m,E_arm_SC_b,'\n')

Pinc_SC = np.array(DHP['Pinc'][0:5]).T
Pinc_SC_m,Pinc_SC_b = angle_to_sc(Pinc_SC)
print('Pinc Servo Limits: ', Pinc_SC)
print('Pinc slope, intercept: ', Pinc_SC_m,Pinc_SC_b,'\n')

# Pincer setup
Pinc_Open = DHP['Pinc'][1]/2
Pinc_Close = DHP['Pinc'][0]
Pinc_Grab_Die = Pinc_Open/5
print('Pincer Range:',Pinc_Open,Pinc_Close,Pinc_Grab_Die)

# Intercept_Servo is the zero angle for all servos
Intercept_Servo = [Rbase_SC_b,L_arm_SC_b,U_arm_SC_b,E_arm_SC_b,Pinc_SC_b]

# Pinc is not a location - used only to grip/release
print('Intercept Servo Locations: ',Intercept_Servo)

## TBD - Put die away

# Initialize Servo Commands
servo_move(Rbase_Ch,R_Accel,R_Speed,6500)
servo_move(L_arm_Ch,R_Accel,R_Speed,6000)
servo_move(U_arm_Ch,R_Accel,R_Speed,7000)
servo_move(E_arm_Ch,R_Accel,R_Speed,6112)
servo_move(Pinc_Ch,R_Accel,R_Speed,2000)

print('Robot moved home using maestro directly')

Rbase_CurPos = 6500
L_arm_CurPos = 6000
U_arm_CurPos = 7000
E_arm_CurPos = 6112
Pinc_CurPos = 2000
    
print('CurPos Positions: ',Rbase_CurPos,L_arm_CurPos,U_arm_CurPos,E_arm_CurPos,Pinc_CurPos)

# Convert to angles and get FK
Th_rb = sc_to_angle(Rbase_CurPos, Rbase_SC_b, Rbase_SC_m)
Th_la = sc_to_angle(L_arm_CurPos, L_arm_SC_b, L_arm_SC_m)
Th_ua = sc_to_angle(U_arm_CurPos, U_arm_SC_b, U_arm_SC_m)
Th_ea = sc_to_angle(E_arm_CurPos, E_arm_SC_b, E_arm_SC_m)
Th_pc = sc_to_angle(Pinc_CurPos, Pinc_SC_b, Pinc_SC_m)

print('CurPos Angles (deg): ',Th_rb,Th_la,Th_ua,'\n')
#print 'CurPos Angles: ',math.degrees(Th_rb),math.degrees(Th_la),math.degrees(Th_ua),'\n'

#FK = for_kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th_rb,Th_la,Th_ua)

# Force user to set position before proceeding
print('Current Servo Commands:', Rbase_CurPos, L_arm_CurPos, U_arm_CurPos, '\n')

print('Please tweak robot servo parameters now')
robot_tweak()

#get_user_input('hit any key to proceed')

## Inverse Kinematics - Input X,Y,Z in base frame
# input desired EE location in RB coordinates (get from camera)

#Initial Robot Position (should be over reference tack)
X_command = Tack_RB[0]
Y_command = Tack_RB[1]
Z_command = 12
R_command = 0
P_command = Pinc_Open

R_Die = 0

Th1,Th2,Th3 = ik_calc(X_command,Y_command,Z_command)

print('Initial IK angles (degrees):',Th1,Th2,Th3)

FK = for_kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th1,Th2,Th3)

robot_move(X_command, Y_command,Z_command,R_command,P_command)
sleep(5)
# take a picture of the robot for later alignment
print('Take picture of robot')
CRFrame = take_picture(0,'Robot') # return filename

## Move robot out of frame!!
X_command = 15
Y_command = 8
Z_command = 15
R_command = 0
P_command = Pinc_Grab_Die
robot_move(X_command, Y_command,Z_command,R_command,P_command)

#TBD
## Get reference photo and define rollout area
EmptyFrame = take_picture(0,'Align') # return filename
EmptyFrame = EmptyFrame+'.jpg'
print('Using',EmptyFrame,' for reference')

Ref_pts,Crop = get_refpoints(EmptyFrame)

# Then transform into robot coordinates to use below
X_maxpix = Ref_pts.loc[Ref_pts['cx'].idxmax()][1]
Y_maxpix = Ref_pts.loc[Ref_pts['cx'].idxmax()][2]

X_minpix = Ref_pts.loc[Ref_pts['cx'].idxmin()][1]
Y_minpix = Ref_pts.loc[Ref_pts['cx'].idxmin()][2]

print('X,Y maxpix',X_maxpix, Y_maxpix)
print('X,Y minpix',X_minpix, Y_minpix)
# Coordinates of full image frame in Rbase CCS - coordinates are flipped and one is reversed!
X_refpix = Tack_RB[0] - Y_maxpix/ImScale 
Y_refpix = X_maxpix/ImScale - Tack_RB[1]

print('ULH corner of image in RCS', X_refpix,Y_refpix)


# eventually do robot parameter correction here, but hopefully it's close enough for now....

### IK Loop - functionalize this

cont = 'y'

### Main Rolling Loop

print('Starting Rolls')
Roll_Count = 1

while Roll_Count <= Num_Rolls:
    # Move Robot around if desired
    while get_user_input('Do you want to adjust robot position?') == 'y':

        #robot_tweak()
        Th_rb = sc_to_angle(Rbase_CurPos, Rbase_SC_b, Rbase_SC_m)
        Th_la = sc_to_angle(L_arm_CurPos, L_arm_SC_b, L_arm_SC_m)
        Th_ua = sc_to_angle(U_arm_CurPos, U_arm_SC_b, U_arm_SC_m)
        Th_ea = sc_to_angle(E_arm_CurPos, E_arm_SC_b, E_arm_SC_m)
        Th_pc = sc_to_angle(Pinc_CurPos, Pinc_SC_b, Pinc_SC_m)

        FK = for_kin(L_arm_X,L_arm_Z,U_arm_X,E_arm_X,E_arm_Z,Pinc_Z,Th_rb,Th_la,Th_ua)
    
        # Move robot in RCS terms
        X_command = float(get_user_input('input x:'))
        Y_command = float(get_user_input('input y:'))
        Z_command = float(get_user_input('input z:'))
        R_command = float(get_user_input('input angle:'))
        P_command = float(get_user_input('input p (0 to 100):'))
        robot_move(X_command, Y_command,Z_command,R_command,P_command)   

    # Move up
    Z_command = 20
    robot_move(X_command, Y_command,Z_command,R_command,P_command)

    # Retract
    X_command = 15
    Y_command = 8
    robot_move(X_command, Y_command,Z_command,R_command,P_command)

    sleep(2)
    # 1. Take null picture and do validity checks
    Null_Photo = take_picture(Roll_Count,'Null')
    print('Null picture taken')
    if Null_Photo == 'null photo':
        Null_Photo = 'POIS_3_Save_20181026-132854_1_Null'
        
    # 2. Register points
    #Null_Input= get_user_input('Place die and continue')
    
    # Traverse
    X_command = -10
    Y_command = 22
    Z_command = 27
    robot_move(X_command, Y_command,Z_command,R_command,P_command)
    sleep(5)
    # 3. Drop die
    P_command = Pinc_Open
    robot_move(X_command, Y_command,Z_command,R_command,P_command)
    print('Die has been dropped - waiting for roll...')
        
    # 4. Take a picture of it
    sleep(5)
    Roll_Photo = take_picture(Roll_Count,'Die')
    if Roll_Photo == 'null photo':
        Roll_Photo = 'POIS_3_Save_20181026-132854_1_Die'
        
    # 5. Go image it and get pip count and die location
    Diff_Photo = get_diff_image(Null_Photo,Roll_Photo,Crop)   
    Contour_Data = get_contours('diff21.jpg')
    
    # 6. Log result
    print('Pips for Roll#',Roll_Count,':',Contour_Data[3],'\n')
    # 7. Calculate die orientation
    R_Die = float(Contour_Data[2])
    Pix_x = float(Contour_Data[0])
    Pix_y = float(Contour_Data[1])
    print('Pixel X,Y of die center:',Pix_x,Pix_y)
    print('Die Angle:',R_Die)
    
    Im_diff = cv.imread(ImDir + 'diff21.jpg')
    cv.circle(Im_diff,(int(Pix_x),int(Pix_y)),4,(255,0,255),2,1)

    # Move into RCS
    R_y = ((Pix_x + X_minpix)/ImScale) + Y_refpix
    R_x = (Pix_y / ImScale) + X_refpix
    print('Rx,Ry:',R_x,R_y)
    
    cv.imshow('Die Location', Im_diff)
    cv.waitKey()
    cv.destroyAllWindows()
    
    # 8. Plan motion path
    
    # 9. Execute motion to pick up die and return to tower
    # Stay high and hover
    X_command = R_x
    Y_command = R_y
    R_command = R_Die
    P_command = Pinc_Open
    robot_move(X_command, Y_command,Z_command,R_command,P_command)

    #Then drop down
    Z_command = -1.5
    robot_move(X_command, Y_command,Z_command,R_command,P_command)

    #Then grab it
    print('grabbing die...')
    P_command = Pinc_Grab_Die
    robot_move(X_command, Y_command,Z_command,R_command,P_command)    

    # Repeat unless error flag is thrown or run is over

    print('Roll ',Roll_Count,' of ',Num_Rolls, 'successfully completed')
    # Elapsed time per roll?
    Roll_Count = Roll_Count + 1

# When all done with runs, go to data reduction
cont = get_user_input('Continue to data reduction (y/n)?')
if cont !='y':
    robot_init()
    cv.destroyAllWindows()
    #sys.exit('Program Aborted')
    quit()
# Update stats/distribution
print ('Run complete, proceeding to data reduction')
robot_init()
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
    print(('File:',DiceFile))
    diff_images(test_name,DiceFile)
    
    DiceFile=ImDir+'diff21.jpg'
    img = ImageOps.grayscale(img)
    img_gamma = exposure.adjust_gamma(img_as_float(img),1.8)  
    img_cont = exposure.adjust_sigmoid(img_as_float(img),cutoff=0.5, gain=50)
    cv_image = img_as_ubyte(img_gamma)
    
    # Send to Line for location
    Die_Loc = Get_Loc(DiceFile)
    print(Die_Loc)

    print(filename +': '+ str(Die_Loc[3]))
    pc[filename] = Die_Loc[3]
    
# Sort by name
pc=collections.OrderedDict(sorted(pc.items()))
GT_df['Run_Results']=list(pc.values())

Bad_Runs = len(GT_df[GT_df.Run_Results==-100])
print('Bad runs:',Bad_Runs)

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

