import serial
from lib.packet import *

# COM 포트 설정 (실제 사용 포트로 변경)
ser = serial.Serial('COM14', 57600)

rc_data = [0 for i in range(0,20)]


def show_RC_CHANNELS(data, length):
    print("[time : %d | count : %d] "%(
        (data[1]|data[2]<<8|data[3]<<16|data[4]<<24)/1000,
        data[5]
        ), end=" ")

    for i in range(0, 18):
        print(data[6+i*2]|data[7+i*2]<<8, end=" ")
    print("rssi : %d"%data[42], end=" ")


def show_ServoOutputRaw(data):
    print("[time : %d | mask : %x] "%(
        (data[1]|data[2]<<8|data[3]<<16|data[4]<<24)/1000,
        data[5]
        ), end=" ")

    for i in range(0, 16):
        print(data[6+i*2]|data[7+i*2]<<8, end=" ")


cnt = 0

while True: 
    data = ser.read()  # 1바이트씩 읽기

    if data == b'\xFD':  # 0xA6이 나오면
        if(packet.data[cnt-2]|packet.data[cnt-1]<<8 == calulate_crc(packet.data, cnt)):
            show_RC_CHANNELS(packet.data, cnt)
            #show_ServoOutputRaw(packet.data)
            print()
        cnt = 0;

    packet.data[cnt] = byte2int(data.hex())
    cnt = cnt +1
    
