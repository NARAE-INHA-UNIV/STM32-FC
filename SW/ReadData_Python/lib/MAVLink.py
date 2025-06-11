import serial
from .packet import *

class MSG_NUM:
    SCALED_IMU = 26
    RAW_IMU = 27
    SCALED_PRESSURE = 29
    SERVO_OUTPUT_RAW = 36
    RC_CHANNELS = 65

class MAVLink:
    rx:packet = None
    ser:serial = None
    __cnt:int = 0

    def __init__(self, port, baudrate=115200):
        self.rx = packet()
        self.connect(port, baudrate)

    def connect(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate)
        self.__cnt = 0
        print("[%s %d] Connected!"%(port, baudrate))
        return 0

    # MSG 선택
    def select(self, num:MSG_NUM):
        tx = [0xFD]
        tx.append(num)
        crc = calulate_crc(tx, 4)
        tx.append(crc >> 8)
        tx.append(crc & 0xff)

        self.ser.write(bytes(tx))
        return 0

    # 모든 바이트를 받았을 때
    def update(self):
        return 0

    # 데이터 출력
    def display(self):
        return 0

    # byte 단위로 데이터 받아옴
    def getByte(self):
        rx_byte = self.ser.read()  # 1바이트씩 읽기

        if rx_byte == b'\xFD':  # 0xFD이 나오면
            self.rx.length = self.__cnt
            self.__cnt = 0;

            if(self.rx.length>0 and self.rx.checkCRC()):
                return 0

        self.rx.data[self.__cnt] = byte2int(rx_byte.hex())
        self.__cnt = self.__cnt +1

        return 1

    # 한 패킷을 받아서 출력
    def getData(self):
        if(self.getByte() == 0):
            self.update()
            self.display()
        return 
