import struct

class SCALED_PRESSURE():
    def update(self):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'I' = 4바이트 unsinged int (uint32_t), 
        # 'f' = 4바이트 float
        # 'h' = 2바이트 signed short (int16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<Iffhh'
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(self.rx.data[1:self.rx.length-2]))

        # unpack된 데이터를 멤버 변수에 할당
        self.time_boot_ms = unpacked_data[0]
        self.press_abs = unpacked_data[1]
        self.press_diff = unpacked_data[2]
        self.temperature = unpacked_data[3]
        self.temperature_press_diff = unpacked_data[4]


    def display(self):
        print("%0.2f : (%4.2f %4.2f) (%2.2f %2.2f)"%(
            self.time_boot_ms,
            self.press_abs, self.press_diff,
            self.temperature, self.temperature_press_diff
            )
            )