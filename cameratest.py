import pistream
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy

LOWER_HSV = numpy.array([111, 117, 142])
UPPER_HSV = numpy.array([180, 255, 255])

video = pistream.PiVideoStream((240, 180)).start()
time.sleep(0.5)
while True:
    image = video.read()
    cv2.imshow('Image', image)
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    imageThresh = cv2.inRange(imageHSV, LOWER_HSV, UPPER_HSV)
    cv2.imshow('Threshhold', imageThresh)
    
    if cv2.waitKey(1) &0xFF == ord('q'):
        cv2.imwrite('/home/pi/FollowBot/capture.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        break
