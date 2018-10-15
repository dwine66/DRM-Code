# -*- coding: utf-8 -*-
"""
Created on Thu Aug 23 05:22:04 2018

@author: Dave
"""

"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
https://docs.opencv.org/master/d9/db0/tutorial_hough_lines.html
"""
import sys
import math
import pprint as pp
import cv2 as cv
import numpy as np
import pandas as pd

def get_spots(File):
    
    img1 = File
    hsv = cv.cvtColor(img1, cv.COLOR_BGR2HSV)
    # Color in BGR    
    lower_green = np.array([139,35,42])
    upper_green = np.array([255,255,255])
    
    mask = cv.inRange(hsv, lower_green, upper_green)
    res = cv.bitwise_and(img1,img1, mask = mask)

    cv.imshow('frame',img1)
    cv.imshow('mask',mask)
    cv.imshow('res',res)
    
    cv.waitKey()

    cv.destroyAllWindows()
    
    return(mask)

    #img1=img1.crop((685,50,1006,756))    
    
### Main Code  

#def main(argv):
default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\red_tacks_20180930-132252_001.jpg'
filename = default_file
#filename = argv[0] if len(argv) > 0 else default_file
# Loads an image
src = cv.imread(filename, cv.IMREAD_GRAYSCALE)
src_col = cv.imread(filename)

# Check if image is loaded fine
if src is None:
    print ('Error opening image!')
    print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
#    return -1
Cal_Spots = get_spots(src_col)

# Contours
retval, threshold = cv.threshold(Cal_Spots,60, 255, cv.THRESH_BINARY)  
srcc, contours, hierarchy = cv.findContours(threshold,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
#
#cv.imshow("Source", src)
#cv.waitKey()
#cv.imshow("Threshold", threshold)
#cv.waitKey()

#cv.imshow("Contours", srcc)
#cv.waitKey()
#for i in range(0,len(contours)):
#    cv.drawContours(src_col, contours, i, (0,255,0), 2)
#    cv.imshow("Contours", src_col)
#    print 'Contour: ',i
#    cv.waitKey()
#
#cv.destroyAllWindows()
#src_col = cv.imread(filename)
#cv.imshow("Shape ID", src_col)
print len(contours)
Num_Circles = 0
Circles=pd.DataFrame(columns=['cnt','cx','cy','Area'])

# from https://www.quora.com/How-I-detect-rectangle-using-OpenCV
for index, cnt in enumerate(contours):
    #print 'Contour',index, 'has length',len(cnt)
    if len(cnt) > 10:
        approx = cv.approxPolyDP(cnt,0.03*cv.arcLength(cnt,True),True)
        print 'Approximation Length', len(approx)
        if len(approx)==3:
            print "triangle"
            cv.drawContours(src_col,[cnt],0,(0,255,0),2)
        elif len(approx)==4:
            print "square"
            cv.drawContours(src_col,[cnt],0,(255,0,0),2)
        elif len(approx) > 4:
            print "circle"
            Num_Circles += 1
            cv.drawContours(src_col,[cnt],0,(0,255,255),2)
                    # Find centroid
            M = cv.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            C_Area = cv.contourArea(cnt)
            print 'Centroid:',cx,'x, ',cy,'y'
            print 'Area:', C_Area, ' pixels'
            if C_Area > 200 and C_Area < 800 and cy < 500 and cx < 700:
                
                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                X_box = int((box[0,0] + box[2,0])/2)
                Y_box = int((box[0,1] + box[2,1])/2)
                print X_box,Y_box
                cv.circle(src_col,(X_box,Y_box),4,(128,64,239),2,1)
                cv.drawContours(src_col,[box],0,(0,128,255),1)
                Circles.loc[Num_Circles] = [index,X_box,Y_box,C_Area]

            #print 'contour',index,' is complex - in red'
            cv.drawContours(src_col,[cnt],0,(0,0,255),1)
    else:
        #print 'contour length tiny - ignored (in orange)'
        cv.drawContours(src_col,[cnt],0,(0,128,255),8)

    cv.imshow("Contours", src_col)
    #cv.waitKey()
        
cv.waitKey()
cv.destroyAllWindows()
print Num_Circles,'Circles Found'

Y_crop_min = 0
Y_crop_max = max(Circles['cy'])
X_crop_min = min(Circles['cx'])
X_crop_max = max(Circles['cx'])

Crop_list = [X_crop_min,Y_crop_min, X_crop_max,Y_crop_max]
print 'Crop Boundaries:', Crop_list
                