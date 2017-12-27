# -*- coding: utf-8 -*-
"""
Created on Sat Nov  4 05:21:13 2017

@author: Dave
"""
import os
import sys
import cv2
import numpy as np
from PIL import Image, ImageChops, ImageOps,ImageEnhance
import matplotlib.pyplot as plt

from skimage.feature import hog
from skimage import data, color, exposure
from skimage.util import img_as_ubyte, img_as_float

WKdir="U:\\BBP\\Dice Rolling Machine\\Python Code\\DRM_Images"

os.chdir(WKdir)
DiceFile = 'RevC_100_B_20171014-121945-094.jpg'
argv=WKdir+'\\'+ DiceFile

img = Image.open(DiceFile)
img = ImageOps.grayscale(img)


#image = color.rgb2gray(img)
image = img_as_float(img)

img2=cv2.imread(DiceFile)

img2=cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
retval, threshold = cv2.threshold(img2,100,200,cv2.THRESH_BINARY)

cv2.imshow('original',img2)
cv2.imshow('threshold',threshold)
cv2.waitKey(0)
cv2.destroyAllWindows()

## HOG processing
fd, hog_image = hog(image, orientations=8, pixels_per_cell=(8, 8), cells_per_block=(1, 1),visualise=True, block_norm='L2-Hys')

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4), sharex=True, sharey=True)

ax1.axis('off')
ax1.imshow(image, cmap=plt.cm.gray)
ax1.set_title('Input image')
ax1.set_adjustable('box-forced')

# Rescale histogram for better display
hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 0.02))

ax2.axis('off')
ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
ax2.set_title('Histogram of Oriented Gradients')
ax1.set_adjustable('box-forced')
plt.show()