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
import cv2 as cv
import numpy as np

def main(argv):
    
    default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\diff21.jpg'
    filename = argv[0] if len(argv) > 0 else default_file
    # Loads an image
    src = cv.imread(filename, cv.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    retval, threshold = cv.threshold(src,15, 255, cv.THRESH_BINARY)   
    blur = cv.medianBlur(threshold, 9)
   # gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    
    dst = cv.Canny(blur, 50, 200, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    lines = cv.HoughLines(dst, 1, np.pi / 180, 52, None, 0, 0)
    
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
    print(lines)
    
    linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 30, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
    print(linesP)
    cv.imshow("Source", blur)
    cv.imshow("Source - Canny", dst)
    cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    cv.waitKey()
    cv.destroyAllWindows()
    return 0
    
if __name__ == "__main__":
    main(sys.argv[1:])
