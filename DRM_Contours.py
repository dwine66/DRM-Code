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

### Main Code  

#def main(argv):
default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\diff21.jpg'
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

# Contours
retval, threshold = cv.threshold(src,60, 255, cv.THRESH_BINARY)  
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
    
# from https://www.quora.com/How-I-detect-rectangle-using-OpenCV
for index, cnt in enumerate(contours):
    print 'Contour',index, 'has length',len(cnt)
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
            cv.drawContours(src_col,[cnt],0,(0,255,255),2)
        else:
            print 'contour',index,' is complex - in red'
            cv.drawContours(src_col,[cnt],0,(0,0,255),1)

        # Find centroid
        M=cv.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print 'Centroid:',cx,cy
        
        #Find angle
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        print box
        try:
            slope = (box[1][1]-box[0][1])/(box[1][0]-box[0][0])
        except:
            slope = 0.0
            print 'slope corrected'
        Die_Angle = math.degrees(math.atan(slope))
        print 'Angle:' , Die_Angle
        print 'slope:', slope
        cv.drawContours(src_col,[box],0,(0,128,255),1)

    else:
        print 'contour length tiny - ignored (in orange)'
        cv.drawContours(src_col,[cnt],0,(0,128,255),8)

    cv.imshow("Contours", src_col)
    cv.waitKey()
        
cv.waitKey()
cv.destroyAllWindows()

