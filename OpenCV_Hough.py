# -*- coding: utf-8 -*-
"""
Created on Sun Oct 22 14:38:12 2017
See
https://docs.opencv.org/master/d4/d70/tutorial_hough_circle.html

## This is an experimental file...
@author: Dave
"""
import os
import sys
import cv2
import numpy as np
from PIL import Image, ImageChops, ImageOps,ImageEnhance
from skimage import data, color, exposure,io
from skimage.util import img_as_ubyte, img_as_float

WKdir="S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\Python Code\\DRM_Images"

os.chdir(WKdir)
DiceFile = 'RevC_100_B_20171014-121945-035.jpg'
DiceFile = 'diff21.jpg'

argv=WKdir+'\\'+ DiceFile

img = Image.open(DiceFile)
img = ImageOps.grayscale(img)
img_gamma = exposure.adjust_gamma(img_as_float(img),2)
img_cont = exposure.adjust_sigmoid(img_as_float(img_gamma),cutoff=0.5, gain=50)
io.imshow(img_gamma)

#cv_image = img_as_ubyte(img_gamma)
    

filename = argv
# Loads an image
src = cv2.imread(filename, cv2.IMREAD_COLOR)
# Check if image is loaded fine
if src is None:
    print ('Error opening image!')
    print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')



retval, threshold = cv2.threshold(src,10,255,cv2.THRESH_BINARY)   

gray = cv2.medianBlur(threshold, 7)

gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)


rows = gray.shape[0]
print (rows)
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 40,
                           param1=50, param2=20,
                           minRadius=10, maxRadius=30)


if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        center = (i[0], i[1])
        # circle center
        cv2.circle(src, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = i[2]
        cv2.circle(src, center, radius, (255, 0, 255), 3)
 
cv2.imshow("Thresholded", gray)
cv2.imshow("detected circles", src)
cv2.waitKey(0)
cv2.destroyAllWindows()



