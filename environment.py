######################################################################
## FollowBot - Environment											##
##																	##
## This module is responsible for storing all sensor readings and	##
## calculating drive forward and turn rates from those readings.	##
######################################################################

import FollowBot
import protocol
import vision
import time

#Distance constants
MIN_DISTANCE = 70		#Minimum distance allowed through camera detection
ULTRA_MIN_DISTANCE = 42	#Minimum distance allowed through ultrasonic detection

#Forward movement constants.
FORWARD_SCALE = 0.005	#Distance scalar to determine forward speed
FORWARD_MAX_RATE = 0.6	#Fastest forward rate allowed
FORWARD_MIN_RATE = 0.08	#Slowest forward rate allowed

#Turn constants
TURN_SCALE = 0.10		#Angle error scalar to determine turn speed
TURN_MAX_RATE = 0.25    #Fastest turn rate allowed
TURN_MIN_RATE = 0.025   #Slowest turn rate allowed
TURN_DEAD_ZONE = 5		#Dead zone to disable turn, degrees from center

distanceLeft = 0		#Current left distance from object, based on ultrasonic readings
distanceRight = 0		#Current right distance from object, based on ultrasonic readings
targetDistance = 0		#Current target distance, based on last camera reading
distance = 0			#The current error between target distance and current position
heading = 0				#Current absolute heading, based on gyro readings
patternHeading = 0		#Current target angle offset from current heading 0
pattern = False			#Whether or not a pattern was detected last cycle
lastUpdate = time.time()#The previous forward

#Updates all readings of the robot's environment.
def update():
	updateUltrasonic()
	updateVision()
	global pattern
	if pattern:
		resetGyro()
		resetDistance()
	else:
		updateGyro()
		updateDistance()
	
#Updates the left and right ultrasonic sensor readings over serial.	
def updateUltrasonic():
	protocol.usb.flushInput()
	protocol.sendCommand('M')
	lrDistance = protocol.read(2)
	if len(lrDistance) == 2:
		global distanceLeft
		global distanceRight
		distanceLeft = ord(lrDistance[0])
		distanceRight = ord(lrDistance[1])

#Updates the current heading of the robot over serial.
def updateGyro():
	protocol.usb.flushInput()
	protocol.sendCommand('G')
	angle = protocol.read(1)
	if len(angle > 0):
		global heading
		heading = ord(angle[0]) - 128

#Updates the location and state of the pattern using vision processing.
def updateVision():
	global targetDistance
	global patternHeading
	global pattern
	distance, angleError, pattern = vision.process()
	if pattern:
		targetDistance = distance
		patternHeading = angleError
	
#Updates the distance the robot has moved since starting or the last reset.  Based on calibrated velocity at various motor powers.
def updateDistance():
	global distance
	distance = distance + calcDistanceMoved(FollowBot.driveTrain.getForward())

#Resets the current heading to 0 on both the Pi and Arduino.
def resetHeading():
	protocol.usb.flushOutput()
	protocol.usb.sendCommand('Z')
	updateGyro()

#Labels current position as distance 0.
def resetDistance():
	global distance
	distance = 0

#Returns the forward and turn values that should be applied to the motors based on sensor input.
def getMovement():
	global targetDistance
	global distance
	return calcForwardRate(targetDistance - distance), calcTurnRate(patternHeading)
	
#Based on the error from target center, in pixels, this function will
#calculate the appropriate rate at which the robot should turn.
def calcTurnRate(error):
    if not pattern:
        return -0.0
    if abs(error) <= TURN_DEAD_ZONE:
        print 'DEAD'
        return 0
	
    #Determine magnitude of turn using the scale factor
    absRate = abs(error) * TURN_SCALE
    #Constrain turn rate within acceptable Min and Max bounds
    if absRate < TURN_MIN_RATE:
        absRate = TURN_MIN_RATE
    else:
        if absRate > TURN_MAX_RATE:
            absRate = TURN_MAX_RATE
    #Apply direction to turn based on the sign of the error
	rate = None
    if error >= 0:
        rate = -absRate
        if distanceRight < ULTRA_MIN_DISTANCE:
            print 'MIN: ', distanceRight
            #rate = 0
    else:
        rate = absRate
        if distanceLeft < ULTRA_MIN_DISTANCE:
            print 'MIN: ', distanceLeft
            #rate = 0
    return rate
            
#Based on the distance from target, this function will
#calculate the appropriate speed at which the robot should move forward.
def calcForwardRate(error):
    if not pattern or error < MIN_DISTANCE or distanceLeft < ULTRA_MIN_DISTANCE or distanceRight < ULTRA_MIN_DISTANCE:
        return 0
    #Determine magnitude of speed using the scale factor
    rate = (error - MIN_DISTANCE) * FORWARD_SCALE
    #Constrain forward rate within acceptable Min and Max bounds
    if rate < FORWARD_MIN_RATE:
        rate = FORWARD_MIN_RATE
    elif rate > FORWARD_MAX_RATE:
        rate = FORWARD_MAX_RATE
    return -rate

#Calculates the distance moved since the last cycle. Based on elapsed time and calibrated velocities at different motor powers.
def calcDistanceMoved(speed):
	return #some calculation from speed