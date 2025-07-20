import struct

class RC_CHANNELS():
    def update(self):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'I' = 4바이트 (uint32_t), 
        # 'H' = 2바이트 (uint16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<IBHHHHHHHHHHHHHHHHHHB'
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(self.rx.data[1:self.rx.length-2]))

        # unpack된 데이터를 멤버 변수에 할당
        self.time_boot_ms = unpacked_data[0]
        self.chancout = unpacked_data[1]
        self.value = [0 for i in range (0,18)]
        for i in range(0, 18):
            self.value[i] = unpacked_data[2+i]
        self.rssi = unpacked_data[18]


    def display(self):
        print("[time : %d | count : %d | rssi : %d ]"%(
            self.time_boot_ms, self.chancout, self.rssi
            ), end=" ")

        for i in range(0,18):
            print("%04d"%(self.value[i]), end=" ")
            if(i+1)%4==0:
                print("|", end=" ")
        print()