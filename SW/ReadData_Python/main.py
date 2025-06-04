from lib.MAVLink import *
from lib.MSG import *
import os

print(os.listdir("./lib/MSG"))
mav = RAW_IMU('COM3', 0, 1000, 4)

while True: 
    mav.getData()
    
