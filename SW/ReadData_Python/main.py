import serial
from lib.packet import *
from lib.MAVLink_MSG import *

# COM 포트 설정 (실제 사용 포트로 변경)
ser = serial.Serial('COM23', 57600)

rx = packet()
cnt = 0
imu = MAVLink(MSG_NUM.SCALED_IMU)
ser.write(b'\xFD\x01\x75\xBC')
#ser.write(b'\xFD\x00\x65\x9D')

while True: 
    data = ser.read()  # 1바이트씩 읽기

    if data == b'\xFD':  # 0xFD이 나오면
        rx.length = cnt
        cnt = 0;

        if(rx.length>0 and rx.checkCRC()):
            imu.update(rx)
            imu.display();

    rx.data[cnt] = byte2int(data.hex())
    cnt = cnt +1
    
