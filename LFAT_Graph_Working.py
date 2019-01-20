# -*- coding: utf-8 -*-
"""
Created on Sun Oct 22 04:34:14 2017

Initiated 10/22/2017

LFAT_Graph - plots data from LFAT data

Rev 0.1 10/22/2017: Initital functionality

@author: Dave
"""
### Libraries
import csv
import numpy as np
import pandas as pd
import os
import re
import math
import matplotlib.pyplot as plt
plt.style.use('ggplot')
from datetime import datetime, timedelta
import dateutil

### Functions
def readcsv(fname):
    vname = pd.DataFrame(pd.read_csv(fname,na_values='n/a',header=0))
    return vname

### Constants

### Main Code
WKdir='S:\\Dave\\QH\\BBP\\LFAT Pump\\Data\\'
WKdir='C:\\Users\\Dave\\Desktop\\'
os.chdir(WKdir)

# Read in LFAT pump data
filename='trend_nh.txt'

LFAT_df = readcsv(filename)
print (filename,' read OK')
##LFAT_df.drop('entry_id',axis=1,inplace = True)
##Timestamp = pd.to_datetime(LFAT_df['created_at'])
#LFAT_df.set_index('created_at',inplace = True)
#LFAT_df.index.name='Timestamp'
#LFAT_df.columns = ['Alert Level','Milone Level','Float Sensor','Temp. (C)','Board V','Milone Raw','AD590 Raw', 'Unix Timestamp']
time = pd.to_numeric(LFAT_df['time(sec)'])
Disp = pd.to_numeric(LFAT_df['disp(mm)'])
#print(Timestamp[1])
#
#pd.to_datetime(Milone.index)

# Use pandas plot function
Toffset=300
plt.plot(time)
plt.plot(Disp)
LFAT_df.plot('time(sec)','disp(mm)')

roll_test = Disp.rolling(100).std()
plt.plot(roll_test)


            
            #,title='LFAT Pump Water Depth (cm)')#, xlim=(len(Milone)-Toffset,len(Milone)))
print (Toffset)
# Pull out Milone data and timestamp

## Plot
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
#ax1.set_xscale('log')
#ax1.set_yscale('log')
ax1.grid(True,which="both",ls="-", color='0.65')
ax1.plot(Milone)
plt.xlabel('Date')
plt.ylabel('Water Level (cm)')
plt.title('Water Level in LFAT vs. time')

"""
# Annotate specific data points
for i in range(0,len(ss_df.index)):
    plt.annotate(ss_df.index[i],xy=(np.array(ss_Density)[i],ss_Temp[i]),xycoords='data')
"""