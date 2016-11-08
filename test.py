import drive
import time

dt = drive.DriveTrain(0, 1)

dt.drive(0.1, 0)
time.sleep(2)
dt.stop()