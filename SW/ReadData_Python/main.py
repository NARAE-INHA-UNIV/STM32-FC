import serial
from lib.packet import *
from lib.MAVLink_MSG import *

# COM 포트 설정 (실제 사용 포트로 변경)
ser = serial.Serial('COM23', 57600)

rx = packet()
cnt = 0
while True: 
    data = ser.read()  # 1바이트씩 읽기

    if data == b'\xFD':  # 0xA6이 나오면
        rx.length = cnt
        cnt = 0;

        if(rx.length>0 and rx.checkCRC()):
            #msg = RC_CHANNELS(rx)
            #msg = SERVO_OUTPUT_RAW(rx)
            msg = RAW_IMU(rx)
            msg.print();

    rx.data[cnt] = byte2int(data.hex())
    cnt = cnt +1
    
