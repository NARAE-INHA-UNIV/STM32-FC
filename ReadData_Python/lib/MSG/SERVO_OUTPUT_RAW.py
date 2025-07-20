from ..MAVLink import *
import struct

class SERVO_OUTPUT_RAW():
    def update(self):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'I' = 4바이트 (uint32_t), 
        # 'H' = 2바이트 (uint16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<IBHHHHHHHHHHHHHHHH'
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(self.rx.data[1:self.rx.length-2]))

        # unpack된 데이터를 멤버 변수에 할당
        self.time_usec = unpacked_data[0]
        self.port = unpacked_data[1]
        self.value = [0 for i in range (0,16)]
        for i in range(0, 16):
            self.value[i] = unpacked_data[2+i]

        pass

    def display(self):
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
