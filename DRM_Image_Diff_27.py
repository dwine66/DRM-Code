# -*- coding: utf-8 -*-
"""
Created on Sun Oct 28 06:08:19 2018

@author: Dave
"""
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
from PIL import Image, ImageChops, ImageOps,ImageEnhance
from skimage import data, color, exposure
from skimage.util import img_as_ubyte, img_as_float

PCDir="C:\\Users\\Dave\\Desktop\\20181028 KM Download\\"

Null_File = 'PICT0017'
Roll_File = 'PICT0018'

    
img1=Image.open(PCDir + Null_File + '.jpg')
img2=Image.open(PCDir + Roll_File + '.jpg')
    
diff12=ImageChops.add(img1,img2)
diff21=ImageChops.add(img2,img1)

diff12.save(PCDir+'add'+Null_File+'-'+Roll_File+'.jpg')
print ('Differential image created')