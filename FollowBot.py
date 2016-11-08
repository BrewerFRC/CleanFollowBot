import drive
import environment
import cv2
import time

driveTrain = drive.DriveTrain(0, 1)

def run():
    while True:
		envrionment.update()
		fb, turn = envrionment.getMovement()
		driveTrain.drive(fb, turn)
        if cv2.waitKey(1) &0xFF == ord('q'):
            break
	
try:
    run()
finally:
    driveTrain.close()
    vision.video.stop()
    cv2.destroyAllWindows()
