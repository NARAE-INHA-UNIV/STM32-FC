from ..MAVLink import *
import struct

class SCALED_IMU2():
    def __init__(self):
        self.xdeg:float = 0;
        self.ydeg:float = 0;
        self.zdeg:float = 0;
        self.time_previous:float = 0;


    def update(self):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'I' = 4바이트 (uint32_t), 
        # 'h' = 2바이트 signed short (int16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<Ihhhhhhhhhh'  # 작은 엔디안 순서로 24바이트 데이터를 처리
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(self.rx.data[1:self.rx.length-2]))

        # unpack된 데이터를 멤버 변수에 할당
        self.time_boot_ms = unpacked_data[0]  # uint32_t (Timestamp)
        self.xacc = unpacked_data[1]       # int16_t (X acceleration)
        self.yacc = unpacked_data[2]       # int16_t (Y acceleration)
        self.zacc = unpacked_data[3]       # int16_t (Z acceleration)
        self.xgyro = unpacked_data[4]      # int16_t (X gyro)
        self.ygyro = unpacked_data[5]      # int16_t (Y gyro)
        self.zgyro = unpacked_data[6]      # int16_t (Z gyro)
        self.xmag = unpacked_data[7]       # int16_t (X magnetic field)
        self.ymag = unpacked_data[8]       # int16_t (Y magnetic field)
        self.zmag = unpacked_data[9]       # int16_t (Z magnetic field)
        self.temperature = unpacked_data[10] # int16_t (Temperature in 0.01C)


    def dps2deg(self):
        # ms -> s
        self.time:float = self.time_boot_ms / 1.0e3

        self.time_diff:float = self.time - self.time_previous
        self.time_previous = self.time

        self.xdeg += self.xgyro * self.time_diff
        self.ydeg += self.ygyro * self.time_diff
        self.zdeg += self.zgyro * self.time_diff


    def display(self):
        # self.dps2deg()

        print("%0.2f : (%3.2f %3.2f %3.2f) (%1.2f %1.2f %1.2f)"%(
            self.time_boot_ms,
            # self.time, self.time_diff,
            self.xgyro, self.ygyro, self.zgyro,
            #self.xdeg, self.ydeg, self.zdeg,
            self.xacc/1000, self.yacc/1000, self.zacc/1000
            )
            )