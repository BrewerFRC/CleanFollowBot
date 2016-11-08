##############################################################
## FollowBot - Drive										##
##															##
## This module is responsible for converting drive values	##
## and sending them to the Arduino-based motor controller.	##
##############################################################

import protocol

# For motor controllers, servo speed setting dampens acceleration (acts like inertia).
# Higher values will reduce inertia (try values around 50 to 100) 
#

class DriveTrain:
    # Init drive train, passing maestro controller obj, and channel
    # numbers for the motor servos Left and Right
    def __init__(self, chLeft, chRight):
        self.chRight = chRight
        self.chLeft = chLeft
        self.servoLeft = protocol.Servo(chLeft)
        self.servoRight = protocol.Servo(chRight, 94)
		self.forward = 0
		self.turn = 0
        
    # Mix joystick inputs into motor L/R mixes
    def arcadeMix(self, joyX, joyY):
        r = -1 * joyX
        l = joyY
        v = (1 - abs(r)) * l + l
        w = (1 - abs(l)) * r + r
        motorR = -(v + w) / 2  
        motorL = (v - w) / 2
        return (motorR, motorL)

    # Scale motor speeds (-1 to 1) to servo target values
    def patternHeading(self, motorR, motorL):
        if (motorR >= 0) :
            r = int(self.servoRight.center + (self.servoRight.max - self.servoRight.center) * motorR)
        else:
            r = int(self.servoRight.center + (self.servoRight.center - self.servoRight.min) * motorR)
        if (motorL >= 0) :
            l = int(self.servoLeft.center + (self.servoLeft.max - self.servoLeft.center) * motorL)
        else:
            l = int(self.servoLeft.center + (self.servoLeft.center - self.servoLeft.min) * motorL)
        return (r, l)

    # Blend X and Y joystick inputs for arcade drive and set servo
    # output to drive motor controllers
    def drive(self, joyX, joyY):
        (motorR, motorL) = self.arcadeMix(joyX, joyY)
        (servoR, servoL) = self.patternHeading(motorR, motorL)
        #print servoR, ":", servoL
        self.servoLeft.setSpeed(servoL)
        self.servoRight.setSpeed(servoR)
		self.forward = joyX
		self.turn = joyY
		
	def getForward(self):
		return self.forward

    # Set both motors to stopped (center) position
    def stop(self):
        self.servoLeft.setSpeed(self.servoLeft.center)
        self.servoRight.setSpeed(self.servoRight.center)

    # Close should be used when shutting down Drive object
    def close(self):
        self.stop()