##########################################################################
## FollowBot - Vision													##
##																		##
## This module is responsible for reading images from a Pi Camera		##
## and process those images with OpenCV.  The module also  identifies	##
## a pattern and provides the distance and angle error between the		##
## pattern and the camera.												##
##########################################################################

import numpy
import math
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import pistream
import sys

#Values for HSV filtering with OpenCV.
LOWER_HSV = numpy.array([113, 76, 124])
UPPER_HSV = numpy.array([171, 255, 254])

#Parameters for robot turning speed
TURN_SCALE = 0.005       #Multiplier to determine turn rate, given error from center in pixels that will determine the robot turn rate

#Conversion constants
DISTANCE_A = 168.51
DISTANCE_B = 0.987561
TURN_SCALE = 0

#Allowable width/height error, measured in boxes
WH_ERROR = 0.35
#Allowable area error from W*H, measured in percentage/100.
AREA_ERROR = 0.35
#Spacing between boxes for both x and y directions, measured in boxes.
BOX_SPACING = 0.35
#Allowable spacing error for both x and y directions, measured in boxes
SPACING_ERROR = 0.35
#The x offset of center, in pixels, created by the position of the camera on the robot.
CENTER_OFFSET_X = 0
#The dimensions of the image.
IMAGE_DIMENSIONS = [240, 180]

kernel = numpy.ones((3, 3), numpy.uint8)  #used to do dialate
video = pistream.PiVideoStream((IMAGE_DIMENSIONS[0],IMAGE_DIMENSIONS[1])).start()
time.sleep(2)

def findPattern(contours, standard):
    #The standard contour properties, defined by the current baseline contour (assumed the top-middle square in the pattern)
    stX, stY, stH, stW = cv2.boundingRect(contours[standard])    #Whether or not contours have been verified for the bottom left and right positions of the pattern.
    signHeight = None
    leftVerified = False
    rightVerified = False
    turnError = 0
    distance = 0
    global pattern
        
    #Cycle through all contours and check against position and congruency relative to standard contour.
    counter = 0
    while counter < len(contours):
        #Properties of current contour.
        x,y,w,h = cv2.boundingRect(contours[counter])
        area = cv2.contourArea(contours[counter])
	   
        #If x is the index of the standard contour, do not check contour.
        if x == standard:
            counter += 1
            continue
        #Check if contour width/height is within allowable error of defined standard.
        else:
	    #If width/height/area is inside allowable error, continue checks.
            if abs(w - stW) <= WH_ERROR*stW and abs(h - stH) <= WH_ERROR*stH and (area - w*h) / (w*h) <= AREA_ERROR:                #Check if contour is in the left or right position. If so, verify contour as part of the pattern.
                if abs(x - (stX - stW*(1+BOX_SPACING))) <= stW*SPACING_ERROR and abs(y - (stY + stH*(1+BOX_SPACING))) <= stH*SPACING_ERROR:
                    signHeight = (y + w - stY)
                    leftVerified = True
                if abs(x - (stX + stW*(1+BOX_SPACING))) <= stW*SPACING_ERROR and abs(y - (stY + stH*(1+BOX_SPACING))) <= stH*SPACING_ERROR:
                    rightVerified = True
        counter += 1
    if leftVerified and rightVerified:
        #Center of the pattern as [x, y]
        center = [stX + stW/2, stY + (1+BOX_SPACING/2)*stH]
                
        #The error in turn relative to the target.
        #print center[0], ";", (IMAGE_DIMENSIONS[0]/2 + CENTER_OFFSET_X)
        turnError = center[0] - (IMAGE_DIMENSIONS[0]/2 + CENTER_OFFSET_X)

        #The distance from target based on the height of the pattern in the image.
        distance = DISTANCE_A * (DISTANCE_B**signHeight)
        #print turnError, ":", calcTurnRate(turnError)
        pattern = True
	angleError = turnError * TURN_SCALE
    return distance, angleError, pattern
#Process images continuously, outputting a command to the robot each cycle
def process():
    #Grab frame from PiCamera
    img_bgr = video.read()
	
    #Resize to a smaller image for better performance
    img_bgr = cv2.resize(img_bgr, (IMAGE_DIMENSIONS[0],IMAGE_DIMENSIONS[1]), interpolation = cv2.INTER_LINEAR)
    #cv2.imwrite('/home/pi/FollowBot/capture.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
    #cv2.imshow('img_bgr',img_bgr)
	
    #Create an HSV variant of the image
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
    #Create a binary threshold based on the HSV range
    img_thresh = cv2.inRange(img_hsv, LOWER_HSV, UPPER_HSV)
    #cv2.imshow("img_thresh", img_thresh)
                
    #Dilate them image to fill holes
    img_dialate = cv2.dilate(img_thresh, kernel, iterations=1)
    #cv2.imshow("img_dialate", img_dialate)
                
    #Find all of the contours in binary image
    _, contours, _ = cv2.findContours(img_dialate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
    #Cycle through contours and test for pattern, using each contour as the "top" box.
    counter = 0
    pattern = False
    if len(contours) > 0:
        while counter < len(contours)-1:
			distance, angleError, pattern = findPattern(contours, counter)
			if pattern:
				return distance, angleError, pattern
            counter += 1
