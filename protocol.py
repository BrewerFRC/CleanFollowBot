##################################################################
## FollowBot - Protocol											##
##																##
## This module is responsible for sending and receiving data	##
## over a serial connection.  Intended for communication with	##
## the Arduino controller on FollowBot.							##
##################################################################

import serial
import vision
import time

#usb = serial.Serial('/dev/ttyUSB0',baudrate=115200)
usb = serial.Serial('/dev/ttyUSB0',baudrate=115200, timeout=0.01)

class Servo:
    def __init__(self, channel, center = 90):
		self.min = 0
		self.max = 180
		self.center = center
		self.cmd = "P" + chr(channel)
        
    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()
	
    def setSpeed(self, speed):
	self.speed = speed
	self.send()
	
    def send(self):
	#Write P<channel><speed>
        usb.write(self.cmd + chr(self.speed))
	#print self.cmd + chr(self.speed)

def sendCommand(command):
	usb.write(command.encode())
	
def read(bytes):
    timeout = time.time()+0.01  #Max time to wait for the response
    response = ""
    while usb.inWaiting() < bytes and time.time() < timeout:
        time.sleep(0.001)
    if usb.inWaiting() == bytes:
        response = usb.read(bytes)
    else:
        print "Too many bytes! Expected: ", bytes, " Received: ", usb.inWaiting()
        usb.flushInput()
    if len(response) == bytes:
        return response
    else:
        return ""