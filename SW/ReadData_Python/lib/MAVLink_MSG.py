import serial

from .MSG.SCALED_IMU import *
from .MSG.RAW_IMU import *
from .MSG.RC_CHANNELS import *
from .MSG.SERVO_OUTPUT_RAW import *

class MSG_NUM:
    SCALED_IMU = 26
    RAW_IMU = 27
    SERVO_OUTPUT_RAW = 36
    RC_CHANNELS = 65

def MAVLink(msgNum:MSG_NUM):
    msg = None


    match(msgNum):
        case MSG_NUM.SCALED_IMU:
            msg = SCALED_IMU()
        case MSG_NUM.RAW_IMU: 
            msg = RAW_IMU()
        case MSG_NUM.SERVO_OUTPUT_RAW:
            msg = SERVO_OUTPUT_RAW()
        case MSG_NUM.RC_CHANNELS:
            msg = RC_CHANNELS()

     
    return msg