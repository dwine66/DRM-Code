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

def Image_Rot(filename,angle):
    # Rotate it 45
    rows,cols = filename.shape
    M = cv.getRotationMatrix2D((cols/2,rows/2),angle,1)
    dst2 = cv.warpAffine(filename,M,(cols,rows))
    return (dst2)

def Image_Prep(filename):
    # Adjust image to pull out die edges
    retval, threshold = cv.threshold(filename,15, 255, cv.THRESH_BINARY)   
    blur = cv.medianBlur(threshold, 9)
       # gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    
    dst = cv.Canny(blur, 50, 200, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstPr = np.copy(cdst)
    return(dst, cdstPr)
    
def HLP(filename):
    # Run the Hough Line transform to get candidate lines
    dst=filename
    # image, lines, distance resolution in pixels, angle resolution in radians, threhold, min line length, max line gap
    HLP_lines = cv.HoughLinesP(dst, 1, np.pi / 180, 35, 30, 5)
    return (HLP_lines)

def Slope_Finder(line_data,image_data,VFlag):
    l_D=np.zeros((len(line_data),3),dtype=float) # Create array for line info

    for i in range(0, len(line_data)): # Draw line
        l = line_data[i][0]
        l2 = [float(j) for j in l]
        cv.line(image_data, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv.LINE_AA)
        
        # clean up lines
        try: # trap vertical line singularity
            slope = (l2[3]-l2[1])/(l2[2]-l2[0]) 
        except: # if it's vertical, flag it
            # slope = (l2[3]-l2[1])/(l2[2]-(l2[0]+0.001))
            VFlag = VFlag +1 # Don't calculate, let it be NaN
            # Why not just do a quick transformation and back in this loop?
            # Because if you think you've found a vertical line, then there is
            # probably another.
           
        l_D[i][0] = slope # calculated slope
        l_D[i][1] = l2[3]-slope*l2[2] # intercept
        l_D[i][2] = math.hypot((l2[3]-l2[1]), (l2[2]-l2[0])) # length
    print'Line slopes, intercepts, lengths:'
    pp.pprint(l_D)
    print'Number of lines:',len(l_D)
    print 'VFlag:',VFlag
    return (l_D)

### Main Code  

#def main(argv):
default_file =  'S:\\Dave\\QH\\BBP\\Dice Rolling Machine\\DRM-Poisson\\Images\\diff21.jpg'
filename = default_file
#filename = argv[0] if len(argv) > 0 else default_file
# Loads an image
src = cv.imread(filename, cv.IMREAD_GRAYSCALE)
src_col = cv.imread(filename)
src_2 = src_col

# Check if image is loaded fine
if src is None:
    print ('Error opening image!')
    print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
#    return -1


## Prep image
src_prep = Image_Prep(src)
src_p = src_prep[0] # Canny data
cdstP = src_prep[1] # Edge overlay
linesP = HLP(src_p) # Get line info (points)

# Contours
retval, threshold = cv.threshold(src,60, 255, cv.THRESH_BINARY)  
srcc, contours, hierarchy = cv.findContours(threshold,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

cv.imshow("Source", src)
cv.waitKey()
cv.imshow("Threshold", threshold)
cv.waitKey()

#cv.imshow("Contours", srcc)
#cv.waitKey()
for i in range(0,len(contours)):
    cv.drawContours(src_col, contours, i, (0,255,0), 2)
    cv.imshow("Contours", src_col)
    print 'Contour: ',i
    cv.waitKey()

cv.destroyAllWindows()
src_col = cv.imread(filename)
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
        print cx,cy
        
        #Find angle
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        print box
        cv.drawContours(src_col,[box],0,(0,128,255),1)

    else:
        print 'contour length tiny - ignored (in orange)'
        cv.drawContours(src_col,[cnt],0,(0,128,255),8)


    cv.imshow("Contours", src_col)
    cv.waitKey()
        
cv.waitKey()
cv.destroyAllWindows()


## Check Slopes
if linesP is not None:
    VFlag = 0 # Set vertical line flag off
    linesD = Slope_Finder(linesP,cdstP,VFlag) # Find slopes and intercepts

    if VFlag != 0: # If slopes are near vertical, rotate and do it again
        src = Image_Rot(src,45)   
        src_prep = Image_Prep(src)
        src_p = src_prep[0]
        cdstP = src_prep[1]
        linesP = HLP(src_p)
        linesD = Slope_Finder(linesP,cdstP,VFlag)

    # redo line parameter finding (put it in a function)

    # check for duplicates - only add if not a duplicate
    SThres = .1
    IThres = .1
    Flag = 0
    Dup=[]
    for i in range (0,len(linesP)):
        for j in range (0,len(linesP)):
            if j>i:
                if abs(linesD[i][0]-linesD[j][0]) > SThres: # if slopes don't match
                    Flag = 0
                else: # check intercepts
                    Flag = 1
                    print (i,' and ',j ,'match slopes')
                    if abs(linesD[i][1] - linesD[j][1])/linesD[i][1] < IThres: # and if intercepts match
                        Flag = 2
                        print (i,' and ',j ,'match intercepts')
                        if linesD[i][2]>linesD[j][2]: # pick the longest line
                            print(i, 'is longest')
                            Dup.append(j)
                        else:
                            print(j, 'is longest')
                            Dup.append(i)
                    else:
                        continue
            else:
                continue    
    print(Dup,' ',len(Dup))
    linesB = np.delete(linesD,np.array(Dup),axis=0)
    print(linesB)
    
    if len(linesB) > 4: # delete the shortest line
        linesB = np.delete(linesB,np.argmin(linesB[:,2]),axis=0)
        print ('deleted a line')
        print (linesB)
        print ('Number of Lines: ',len(linesB))
        
    for i in range(0,len(linesB)):
        x1=0
        x2=300
        y1=int(linesB[i][0]*x1+linesB[i][1])
        y2=int(linesB[i][0]*x2+linesB[i][1])       
        cv.line(cdstP, (x1,y1), (x2,y2), (255-30*i,127,0), 2, cv.LINE_AA)
        
    # get angles for DH
    Die_Angle = math.degrees(math.atan((linesB[1][0])))
    print('Die Angle',Die_Angle)
    
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

    cv.waitKey()
    cv.destroyAllWindows()
    
    linesB = np.insert(linesB,4,linesB[0],axis=0)
    # Now create four new points that are the bounding box of the die
    BoxP = np.zeros((5,2),dtype=int)
    for i in range (1,len(linesB)):
        if abs(linesB[i][0]-linesB[i-1][0]) > SThres: # if lines are not parallel
            x = (linesB[i][1]-linesB[i-1][1])/(linesB[i-1][0]-linesB[i][0])
            y = linesB[i-1][0]*x + linesB[i-1][1]
            BoxP[i][0]=int(x)
            BoxP[i][1]=int(y)
            
    BoxP = np.delete(BoxP,(0),axis=0)

    # Finally, create a bounding box in screen coordinates (use for cropping)
    BoxB=np.zeros((4,2),dtype=int)
    
    # Upper Left
    BoxB[0][0]=np.amin(BoxP[:,0])
    BoxB[0][1]=np.amin(BoxP[:,1])
    cv.circle(cdstP, (BoxB[0][0], BoxB[0][1]), 6, (255,0,0),1, 8)
    
    # Upper Right
    BoxB[1][0]=np.amax(BoxP[:,0])
    BoxB[1][1]=np.amin(BoxP[:,1])
    
    # Lower Left
    BoxB[2][0]=np.amin(BoxP[:,0])
    BoxB[2][1]=np.amax(BoxP[:,1])
    
    #Lower Right
    BoxB[3][0]=np.amax(BoxP[:,0])
    BoxB[3][1]=np.amax(BoxP[:,1])    
    
    print(BoxB)
    cv.line(cdstP, (BoxB[0][0], BoxB[0][1]), (BoxB[3][0], BoxB[3][1]), (0,255,0), 3, cv.LINE_AA)
    cv.line(cdstP, (BoxB[1][0], BoxB[1][1]), (BoxB[2][0], BoxB[2][1]), (0,255,0), 3, cv.LINE_AA)   

# The center of the die is just the average of them
    Die_X=(BoxB[0][0]+BoxB[1][0])/2
    Die_Y=(BoxB[0][1]+BoxB[2][1])/2
    
    print ('Die Center (X,Y):' ,Die_X,Die_Y)
    cv.line(cdstP, (Die_X,Die_Y), (0,0), (255,0,255), 2, cv.LINE_AA)
    
cv.imshow("Source", blur)
cv.imshow("Source - Canny", dst)
cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

cv.waitKey()
cv.destroyAllWindows()
#    return 0
    
#if __name__ == "__main__":
#    main(sys.argv[1:])
