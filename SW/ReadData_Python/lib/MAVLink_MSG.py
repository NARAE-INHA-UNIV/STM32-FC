from .packet import *

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

        # time us -> s
        self.time = self.time_usec / 1.0e6        

    def reg2dps(self):
        self.xdps = self.xgyro * self.GYRO_SENSITIVITY_2000DPS
        self.ydps = self.ygyro * self.GYRO_SENSITIVITY_2000DPS
        self.zdps = self.zgyro * self.GYRO_SENSITIVITY_2000DPS

    def dps2deg(self):
        self.time_diff = self.time - self.time_previous
        self.time_previous = self.time

        self.xdeg += self.xdps * self.time_diff
        self.ydeg += self.ydps * self.time_diff
        self.zdeg += self.zdps * self.time_diff


    def print(self):
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

    
class SERVO_OUTPUT_RAW:
    def __init__(self, rx:packet):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'I' = 4바이트 (uint32_t), 
        # 'H' = 2바이트 (uint16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<IBHHHHHHHHHHHHHHHH'
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(rx.data[1:rx.length-2]))

        # unpack된 데이터를 멤버 변수에 할당
        self.time_usec = unpacked_data[0]
        self.port = unpacked_data[1]
        self.value = [0 for i in range (0,16)]
        for i in range(0, 16):
            self.value[i] = unpacked_data[2+i]

        pass

    def print(self):
        print("[time : %d] %04d %04d %04d %04d | %04d %04d %04d %04d | %04d %04d %04d %04d | %04d %04d %04d %04d\n"%(
            self.time_usec, 
            self.value[0], self.value[1], self.value[2], self.value[3], 
            self.value[4], self.value[5], self.value[6], self.value[7], 
            self.value[8], self.value[9], self.value[10], self.value[11],
            self.value[12], self.value[13], self.value[14], self.value[15],
            ), end="")

def show_ServoOutputRaw(data):
    print("[time : %d | mask : %x] "%(
        (data[1]|data[2]<<8|data[3]<<16|data[4]<<24)/1000,
        data[5]
        ), end=" ")

    for i in range(0, 16):
        print(data[6+i*2]|data[7+i*2]<<8, end=" ")

class RC_CHANNELS:
    def __init__(self, rx:packet):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'I' = 4바이트 (uint32_t), 
        # 'H' = 2바이트 (uint16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<IBHHHHHHHHHHHHHHHHHHB'
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(rx.data[1:rx.length-2]))

        # unpack된 데이터를 멤버 변수에 할당
        self.time_boot_ms = unpacked_data[0]
        self.chancout = unpacked_data[1]
        self.value = [0 for i in range (0,18)]
        for i in range(0, 18):
            self.value[i] = unpacked_data[2+i]
        self.rssi = unpacked_data[18]

        pass

    def print(self):
        print("[time : %d | count : %d] %04d %04d %04d %04d %04d %04d | %04d %04d %04d %04d %04d %04d | %04d %04d %04d %04d %04d %04d | %d\n"%(
            self.time_boot_ms, 
            self.chancout, 
            self.value[0], self.value[1], self.value[2], self.value[3], self.value[4], self.value[5],
            self.value[6], self.value[7], self.value[8], self.value[9], self.value[10], self.value[11],
            self.value[12], self.value[13], self.value[14], self.value[15], self.value[16], self.value[17],
            self.rssi
            ), end="")
