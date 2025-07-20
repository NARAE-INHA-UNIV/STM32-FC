import sys
import serial
from .packet import *
from lib.MSG import *

handlerDict = {
    26 : SCALED_IMU,
    27 : RAW_IMU,
    # 29 : SCALED_IMU,
    # 36 : SERVO_OUTPUT_RAW,
    # 65 : RC_CHANNELS,
    # 116 : SCALED_IMU2
}

class MAVLink:
    rx:packet = None
    ser:serial = None
    __cnt:int = 0
    __MSG_ID = None

    def __init__(self, port, baudrate=115200, MSG_ID=None):
        self.rx = packet()
        self.connect(port, baudrate)
        self.setLog(MSG_ID)

    # 한 패킷을 받아서 출력
    def getData(self):
        if(self.getByte() != 0): return -1

        self.rx.seq = self.rx.data[2]
        self.rx.msgId = self.rx.data[3] | self.rx.data[4] << 8

        handler = handlerDict.get(self.__MSG_ID)
        if(self.__MSG_ID != self.rx.msgId) : return 0
        if(handler == None) : return -2

        handler.display(handler, self.rx)
        return 

    def connect(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate)
            self.__cnt = 0
            print("[%s %d] Connected!"%(port, baudrate))
            return 0

        except Exception as err:
            print(err)
            exit()

    def setLog(self, id):
        self.__MSG_ID = id
        return

    # byte 단위로 데이터 받아옴
    def getByte(self):
        try:
            rx_byte = self.ser.read()  # 1바이트씩 읽기

            self.rx.data[self.__cnt] = byte2int(rx_byte.hex())
            match(self.__cnt):
                case 0 : 
                    if rx_byte != b'\xFD':
                        self.__cnt = 0
                        return -1;
                case 1 : 
                    self.rx.length = byte2int(rx_byte.hex())
                case _:
                    if(self.__cnt == self.rx.length -1):
                        self.__cnt = 0
                        if(self.rx.length>0 and self.rx.checkCRC()):
                            return 0
                        return 1;
                    elif (self.__cnt > self.rx.length -1) :
                        return -1;

            self.__cnt = self.__cnt + 1


        except Exception as e:
            print(e)
            sys.exit(0)

        return 1


    # MSG 선택 (Not used)
    def __select(self, num):
        print("NOT USED : select");
        tx = [0xFD]
        tx.append(num)
        crc = calulate_crc(tx, 4)
        tx.append(crc >> 8)
        tx.append(crc & 0xff)

        self.ser.write(bytes(tx))
        return 0