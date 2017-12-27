# -*- coding: utf-8 -*-
"""
Created on Thu Oct 12 20:02:36 2017

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

import matplotlib.pyplot as plt
from scipy import ndimage as ndi

from skimage import feature, measure
import os
from PIL import Image, ImageChops, ImageOps,ImageEnhance

# For Hough
from skimage import data, color, exposure
from skimage.transform import hough_circle, hough_circle_peaks
from skimage.feature import blob_dog, blob_log, blob_doh
from skimage.color import rgb2gray
from skimage.feature import canny
from skimage.draw import circle_perimeter
from skimage.util import img_as_ubyte, img_as_float
from skimage.feature import match_template

### Functions
def diff_images(fileempty,file):
    img1=Image.open(fileempty)
    img2=Image.open(file)
    
    img1=img1.crop((95,30,1548,1126))
    img2=img2.crop((95,30,1548,1126))
        
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

def canny2(im,sg,th_L,th_H,uq):
    #from http://scikit-image.org/docs/dev/auto_examples/edges/plot_canny.html#sphx-glr-auto-examples-edges-plot-canny-py

    edges1 = feature.canny(im)
    edges2 = feature.canny(im, sigma=sg,low_threshold=th_L,high_threshold=th_H,use_quantiles=uq)
    # display results
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3, figsize=(10, 3),
                                        sharex=True, sharey=True)
    
    ax1.imshow(im, cmap=plt.cm.gray)
    ax1.axis('off')
    ax1.set_title('noisy image', fontsize=10)
    
    ax2.imshow(edges1, cmap=plt.cm.gray)
    ax2.axis('off')
    ax2.set_title('Canny filter, $\sigma=1$', fontsize=10)
    
    ax3.imshow(edges2, cmap=plt.cm.gray)
    ax3.axis('off')
    ax3.set_title('Canny filter, $\sigma=sg$', fontsize=10)
    
    fig.tight_layout()
    
    plt.show()
    
    return edges2

def Contours(r,s,a):
    # from http://scikit-image.org/docs/dev/auto_examples/edges/plot_contours.html#sphx-glr-auto-examples-edges-plot-contours-py
    contours = measure.find_contours(r, s,'low')
    pip_list=[]
    # Display the image and plot all contours found
    fig, ax = plt.subplots()
    ax.imshow(r, interpolation='nearest', cmap=plt.cm.gray)
    
    for n, contour in enumerate(contours):
        x_len=max(contour[:,0])-min(contour[:,0])
        y_len=max(contour[:,1])-min(contour[:,1])
        Area=x_len*y_len
       # print(Area)
      #  AR=x_len/y_len
        if Area>a:
            ax.plot(contour[:, 1], contour[:, 0], linewidth=1)
            pip_list.append(contour)
    ax.axis('image')
    ax.set_xticks([])
    ax.set_yticks([])
    plt.show()
    return contours,pip_list
    
def Shapes(gray):
    
    ret,thresh = cv2.threshold(gray,127,255,1)
    
    contours,h = cv2.findContours(thresh,1,2)
    
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        print ( len(approx))
        if len(approx)==5:
            print ("pentagon")
            cv2.drawContours(img,[cnt],0,255,-1)
        elif len(approx)==3:
            print ("triangle")
            cv2.drawContours(img,[cnt],0,(0,255,0),-1)
        elif len(approx)==4:
            print ("square")
            cv2.drawContours(img,[cnt],0,(0,0,255),-1)
        elif len(approx) == 9:
            print ("half-circle")
            cv2.drawContours(img,[cnt],0,(255,255,0),-1)
        elif len(approx) > 15:
            print ("circle")
            cv2.drawContours(img,[cnt],0,(0,255,255),-1)
    
    cv2.imshow('img',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()   

def Hough(image,th_L,th_H):
    #http://scikit-image.org/docs/dev/auto_examples/edges/plot_circular_elliptical_hough_transform.html#sphx-glr-auto-examples-edges-plot-circular-elliptical-hough-transform-py
    edges = canny(image, sigma=2, low_threshold=th_L, high_threshold=th_H)
    
    # Detect two radii
    hough_radii = np.arange(5, 15, 1)
    hough_res = hough_circle(edges, hough_radii)
    
    # Select the most prominent 5 circles
    accums, cx, cy, radii = hough_circle_peaks(hough_res, hough_radii,
                                               total_num_peaks=3)
    
    # Draw them
    fig, ax = plt.subplots(ncols=1, nrows=1, figsize=(10, 4))
    image = color.gray2rgb(image)
    for center_y, center_x, radius in zip(cy, cx, radii):
        circy, circx = circle_perimeter(center_y, center_x, radius)
        image[circy, circx] = (220, 20, 20)
    
    ax.imshow(image, cmap=plt.cm.gray)
    plt.show()

def Blobs(image,th):
    #image = data.hubble_deep_field()[0:500, 0:500]
    
    image_gray = rgb2gray(image)
    
    blobs_log = blob_log(image_gray, min_sigma=15,max_sigma=19, num_sigma=10, threshold=th, overlap=.1)
    
    # Compute radii in the 3rd column.
    blobs_log[:, 2] = blobs_log[:, 2] * math.sqrt(2)
    
    blobs_dog = blob_dog(image_gray, max_sigma=30, threshold=th)
    blobs_dog[:, 2] = blobs_dog[:, 2] * math.sqrt(2)
    
    blobs_doh = blob_doh(image_gray, min_sigma=15, max_sigma=30, threshold=th)
    
    blobs_list = [blobs_log, blobs_dog, blobs_doh]
    colors = ['yellow', 'lime', 'red']
    titles = ['Laplacian of Gaussian', 'Difference of Gaussian',
              'Determinant of Hessian']
    sequence = zip(blobs_list, colors, titles)
    
    fig, axes = plt.subplots(1, 3, figsize=(9, 3), sharex=True, sharey=True,
                             subplot_kw={'adjustable': 'box-forced'})
    ax = axes.ravel()
    
    for idx, (blobs, color1, title) in enumerate(sequence):
        ax[idx].set_title(title)
        ax[idx].imshow(image, interpolation='nearest')
        for blob in blobs:
            y, x, r = blob
            c = plt.Circle((x, y), r, color=color1, linewidth=1, fill=False)
            ax[idx].add_patch(c)
        ax[idx].set_axis_off()
    
    plt.tight_layout()
    plt.show()
    return blobs_log

def TempMatch(img1,refimage):

    image = img_as_ubyte(img1)
    coin = img_as_ubyte(refimage)
   # coin = coin[0:200,200:200]
    
    result = match_template(image, coin)
    ij = np.unravel_index(np.argmax(result), result.shape)
    x, y = ij[::-1]
    
    fig = plt.figure(figsize=(8, 3))
    ax1 = plt.subplot(1, 3, 1)
    ax2 = plt.subplot(1, 3, 2, adjustable='box-forced')
    ax3 = plt.subplot(1, 3, 3, sharex=ax2, sharey=ax2, adjustable='box-forced')
    
    ax1.imshow(coin, cmap=plt.cm.gray)
    ax1.set_axis_off()
    ax1.set_title('template')
    
    ax2.imshow(image, cmap=plt.cm.gray)
    ax2.set_axis_off()
    ax2.set_title('image')
    # highlight matched region
    hcoin, wcoin = coin.shape
    rect = plt.Rectangle((x, y), wcoin, hcoin, edgecolor='r', facecolor='none')
    ax2.add_patch(rect)
    
    ax3.imshow(result)
    ax3.set_axis_off()
    ax3.set_title('`match_template`\nresult')
    # highlight matched region
    ax3.autoscale(False)
    ax3.plot(x, y, 'o', markeredgecolor='r', markerfacecolor='none', markersize=10)
    
    plt.show()

def ImPlot(im):
    fig, ax = plt.subplots()
    ax.imshow(im, interpolation='nearest', cmap=plt.cm.gray)
    
    
def OpenCV_Hough(argv,av2):
# https://docs.opencv.org/master/d4/d70/tutorial_hough_circle.html
    filename = argv
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
    #print (rows)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 40,
                               param1=50, param2=18,
                               minRadius=10, maxRadius=30)
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
WKdir="U:\\BBP\\Dice Rolling Machine\\Python Code\\DRM_Images"

os.chdir(WKdir)
file_names = os.listdir(WKdir) # Get list of photo names
test_name = 'RevC_100_B_20171014-121945-079.jpg'

file_names = [test_name]
#EmptyFile = "RevC_Cal_Empty_01_20171014-094835.jpg"
#EmptyFile = "Average.jpg"
#DiceFile = "RevC_100_B_20171014-122503_067.jpg"

ConfigFile = 'C:\\Users\\Dave\\Desktop\\RunC_100_B_20171014-1212945_Config.csv'
Run_df=readcsv(ConfigFile) # Read in Config File
Run_df.set_index('Parameter',inplace = True)
GT_df=Run_df.drop(Run_df.index[0:6])
GT_df['Value']=pd.to_numeric(GT_df['Value'])

#diff_images(EmptyFile,DiceFile)
#
##img_Diff = img_as_ubyte(Image.open('diff12.jpg'))
##LoG_Array = Blobs(img_Diff)
#
#img = Image.open('diff12.jpg')
#img = ImageOps.grayscale(img)
#img_gamma = exposure.adjust_gamma(img_as_float(img),.2)
#
#ImPlot(img_gamma)
#img_Diff = img_as_ubyte(img_gamma)
#LoG_Array = Blobs(img_Diff,.4)
#
#
#
#img_Dice_G = ImageOps.grayscale(Image.open(DiceFile))
#img.load()
#img_np=np.asarray(img,dtype='int32')
#
#cont = Contours(img_np,30,1100)
#Cont_list = cont[0]
#pips = cont[1]
#
##for n, pip in enumerate(pips):
#xy = measure.CircleModel().predict_xy(np.array(pips[0]), params=(2, 3, 4))
#model = measure.CircleModel()
#model.estimate(xy)
# 
#canny2(img_np,2,2,10)       
# look for contours with same beginning and ending
#img_g=cv2.imread(DiceFile,0)
#Shapes(img_g)
#
## Hough Transform
#img_h = img_as_ubyte(img)
#Hough(img_h,20,50)

## Loop
pc = {}
for filename in file_names:
    
    if filename[-7].isdigit():
        DiceFile = filename
    else:
        continue
    # Do image preprocessing
    img = Image.open(DiceFile)
    
    diff_images('Average.jpg',DiceFile)
    
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

"""
#PIL dif
    diff_images(EmptyFile,DiceFile)
    
    #http://scikit-image.org/docs/dev/api/skimage.feature.html#skimage.feature.blob_doh
#    img_Diff = img_as_ubyte(Image.open('diff12.jpg'))
#    LoG_Array = Blobs(img_Diff)
    
    img = Image.open('diff21.jpg')
    img = ImageOps.grayscale(img)
    img_gamma = exposure.adjust_gamma(img_as_float(img),.2)
    
    ImPlot(img_gamma)
    img_Diff = img_as_ubyte(img_gamma)
    LoG_Array = Blobs(img_Diff,.25)



    for LoG in LoG_Array:
        if LoG[0] != 0:
            if LoG[1] != 0:
                PipCount+=1
    print (filename +': '+ str(PipCount))
    pc[filename]=PipCount
    
## Plot Results


#
"""

