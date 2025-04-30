from ..packet import *
import struct

class RAW_IMU:
    def __init__(self):
        self.xdeg = 0;
        self.ydeg = 0;
        self.zdeg = 0;
        self.time_previous = 0;

        # 자이로스코프 감도 (LSB/dps)
        # GYRO_CONFIG0 = 0x06 (±2000 dps) 설정 기준
        self.GYRO_SENSITIVITY_2000DPS = float(2000/32768.0)
        pass

    def update(self, rx:packet):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'Q' = 8바이트 unsigned long long (uint64_t), 
        # 'h' = 2바이트 signed short (int16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<QhhhhhhhhhBh'  # 작은 엔디안 순서로 24바이트 데이터를 처리
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(rx.data[1:rx.length-2]))

        # unpack된 데이터를 멤버 변수에 할당
        self.time_usec = unpacked_data[0]  # uint64_t (Timestamp)
        self.xacc = unpacked_data[1]       # int16_t (X acceleration)
        self.yacc = unpacked_data[2]       # int16_t (Y acceleration)
        self.zacc = unpacked_data[3]       # int16_t (Z acceleration)
        self.xgyro = unpacked_data[4]      # int16_t (X gyro)
        self.ygyro = unpacked_data[5]      # int16_t (Y gyro)
        self.zgyro = unpacked_data[6]      # int16_t (Z gyro)
        self.xmag = unpacked_data[7]       # int16_t (X magnetic field)
        self.ymag = unpacked_data[8]       # int16_t (Y magnetic field)
        self.zmag = unpacked_data[9]       # int16_t (Z magnetic field)
        self.id = unpacked_data[10]        # uint8_t (Id)
        self.temperature = unpacked_data[11] # int16_t (Temperature in 0.01C)


    def reg2dps(self):
        self.xdps = self.xgyro * self.GYRO_SENSITIVITY_2000DPS
        self.ydps = self.ygyro * self.GYRO_SENSITIVITY_2000DPS
        self.zdps = self.zgyro * self.GYRO_SENSITIVITY_2000DPS

    def dps2deg(self):
        # time us -> s
        self.time = self.time_usec / 1.0e6        

        self.time_diff = self.time - self.time_previous
        self.time_previous = self.time

        self.xdeg += self.xdps * self.time_diff
        self.ydeg += self.ydps * self.time_diff
        self.zdeg += self.zdps * self.time_diff


    def display(self):
        self.reg2dps()
        self.dps2deg()

        print("%0.2f %0.2f: %d %d %d\n"%(
            self.time, self.time_diff,
            self.xdeg, self.ydeg, self.zdeg
            )
            )

        # print("[time : %d] %d %d %d | %d %d %d | %d\n"%(
        #     self.time_usec, 
        #     self.xacc, self.yacc, self.zacc,
        #     self.xgyro, self.ygyro, self.zgyro,
        #     self.temperature
        #     ), end="")
