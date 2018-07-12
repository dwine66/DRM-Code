# -*- coding: utf-8 -*-
"""
Created on Sat Jun 09 18:27:15 2018

@author: Dave
"""
import csv
import numpy as np
import pandas as pd
import os
import re
import cv2
import math
import collections

from PIL import Image, ImageChops, ImageOps,ImageEnhance

import matplotlib.pyplot as plt

import scipy
import cv2

from skimage import data, color, exposure
from skimage.util import img_as_ubyte, img_as_float

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
    
    diff12.save("diff12.jpg")
    diff21.save("diff21.jpg")

def OpenCV_Hough(argv,av2):
# https://docs.opencv.org/master/d4/d70/tutorial_hough_circle.html
    filename = argv
    default_file = 'default_file'
    # Loads an image
    src = cv2.imread(filename, cv2.IMREAD_COLOR)
    #src = av2  
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        return -1
    

    retval, threshold = cv2.threshold(src,15, 255, cv2.THRESH_BINARY)   
    blur = cv2.medianBlur(threshold, 9)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    rows = gray.shape[0]
    print (rows)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 200,
                               param1=100, param2=11 ,
                               minRadius=3, maxRadius=9)
    # Default is 100,30,15,30
    Pips=0
   # print (len(circles))
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(src, center, radius, (255, 0, 255), 3)
            Pips=len(circles[0,:,:])
    else:
        Pips==0
    
    cv2.imshow('Threshold', gray )        
    cv2.imshow("detected circles", src)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return Pips

def readcsv(fname):
    vname = pd.DataFrame(pd.read_csv(fname,na_values='n/a'))
    return vname

### Main
WKdir="S:\\Dave\QH\\BBP\\Dice Rolling Machine\\DRM Rev C Images"

os.chdir(WKdir)
file_names = os.listdir(WKdir) # Get list of photo names
#test_name = 'RevC_100_B_20171014-121945-079.jpg'
test_name = 'd_20180601-193139_0.jpg'
#file_names = [test_name]
#EmptyFile = "RevC_Cal_Empty_01_20171014-094835.jpg"
#EmptyFile = "Average.jpg"
#DiceFile = "RevC_100_B_20171014-122503_067.jpg"

# Get Ground Truth for run
ConfigFile = 'C:\\Users\\Dave\\Desktop\\RunC_100_B_20171014-1212945_Config.csv'
Run_df=readcsv(ConfigFile) # Read in Config File
Run_df.set_index('Parameter',inplace = True)
GT_df=Run_df.drop(Run_df.index[0:6])
GT_df['Value']=pd.to_numeric(GT_df['Value'])

## Loop
pc = {}
for filename in file_names:
    
    if filename[3].isdigit():
        DiceFile = filename
    else:
        print ('non-image file')
        continue
    # Do image preprocessing
    img = Image.open(DiceFile)
    
    diff_images(test_name,DiceFile)
    
    DiceFile='diff21.jpg'
    img = ImageOps.grayscale(img)
    img_gamma = exposure.adjust_gamma(img_as_float(img),1.8)  
    img_cont = exposure.adjust_sigmoid(img_as_float(img),cutoff=0.5, gain=50)
    cv_image = img_as_ubyte(img_gamma)
    
    # Send to Hough for pip ID    
    Pips = OpenCV_Hough(DiceFile,cv_image)
    print (filename +': '+ str(Pips))
    pc[filename]=Pips
    
# Sort by name
pc=collections.OrderedDict(sorted(pc.items()))
GT_df['Run Results']=pc.values()
GT_df['Difference']=GT_df['Value']-GT_df['Run Results']

#Plot histogram

n, bins, patches = plt.hist([GT_df['Value'],GT_df['Run Results']], \
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
plt.axis([-6, 6, 0, 100])
plt.grid(True)
plt.show()