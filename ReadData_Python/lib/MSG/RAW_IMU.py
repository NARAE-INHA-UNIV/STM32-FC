import struct

class RAW_IMU():
    xdeg = 0
    ydeg = 0
    zdeg = 0

    time_previous = 0

    GYRO_SENSITIVITY_DPS = 0
    ACC_SENSITIVITY_DPS = 0

    xgyro_offset = 0
    ygyro_offset = 0
    zgyro_offset = 0
    xacc_offset = 0
    yacc_offset = 0
    zacc_offset = 0

    def __init__(self, GYRO_SENS=2000, ACC_SENS=16):
        self.GYRO_SENSITIVITY_DPS = float(GYRO_SENS/32768.0)
        self.ACC_SENSITIVITY_DPS = float(ACC_SENS/32768.0)
        self.calibration()

    def calibration(self):
        LEN = 30
        for i in range(0, LEN):
            while (super().getByte()) == 1 :
                continue
            self.update()

            self.xgyro_offset += self.xgyro
            self.ygyro_offset += self.ygyro
            self.zgyro_offset += self.zgyro
            self.xacc_offset += self.xacc
            self.yacc_offset += self.yacc
            self.zacc_offset += self.zacc
        
        self.xgyro_offset /= LEN
        self.ygyro_offset /= LEN
        self.zgyro_offset /= LEN
        self.xacc_offset /= LEN
        self.yacc_offset /= LEN
        self.zacc_offset /= LEN

        self.time_previous = self.time_usec / 1.0e6
            

    def update(self, rx):
        # 데이터 포맷: (각 항목의 바이트 크기에 맞게 포맷 지정)
        # 'Q' = 8바이트 unsigned long long (uint64_t), 
        # 'h' = 2바이트 signed short (int16_t), 
        # 'B' = 1바이트 unsigned char (uint8_t)
        fmt = '<QhhhhhhhhhBh'  # 작은 엔디안 순서로 24바이트 데이터를 처리
        
        # struct.unpack을 사용해 데이터를 한 번에 풀어냄
        unpacked_data = struct.unpack(fmt, bytes(rx.data[5:rx.length-2]))

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


    def raw2dps_cal(self):
        self.xdps = (self.xgyro-self.xgyro_offset) * self.GYRO_SENSITIVITY_DPS
        self.ydps = (self.ygyro-self.ygyro_offset) * self.GYRO_SENSITIVITY_DPS
        self.zdps = (self.zgyro-self.zgyro_offset) * self.GYRO_SENSITIVITY_DPS

    def raw2dps(self):
        self.xdps = self.xgyro * self.GYRO_SENSITIVITY_DPS
        self.ydps = self.ygyro * self.GYRO_SENSITIVITY_DPS
        self.zdps = self.zgyro * self.GYRO_SENSITIVITY_DPS

    def dps2deg(self):
        # time us -> s
        self.time = self.time_usec / 1.0e6        

        self.time_diff = self.time - self.time_previous
        self.time_previous = self.time

        self.xdeg += (self.xdps * self.time_diff)
        self.ydeg += (self.ydps * self.time_diff)
        self.zdeg += (self.zdps * self.time_diff)


    def raw2g_cal(self):
        self.xacc = (self.xacc-self.xacc_offset) * self.ACC_SENSITIVITY_DPS
        self.yacc = (self.yacc-self.yacc_offset) * self.ACC_SENSITIVITY_DPS
        self.zacc = (self.zacc-self.zacc_offset) * self.ACC_SENSITIVITY_DPS

    def raw2g(self):
        self.xacc = self.xacc * self.ACC_SENSITIVITY_DPS
        self.yacc = self.yacc * self.ACC_SENSITIVITY_DPS
        self.zacc = self.zacc * self.ACC_SENSITIVITY_DPS


    def display(self, rx):
        self.update(self,rx)
        # self.raw2dps_cal(self)
        #self.raw2dps()
        # self.dps2deg(self)

        # self.raw2g_cal(self)
        #self.raw2g()

        print("%0.2f %0.2f: (%0.2f %0.2f %0.2f) (%0.2f %0.2f %0.2f)"%(
            # self.time, self.time_diff,
            self.time_usec, 0.0,
            # self.xdeg, self.ydeg, self.zdeg,
            self.xacc, self.yacc, self.zacc,
            # self.xdps, self.ydps, self.zdps,
            self.xgyro, self.ygyro, self.zgyro,
            )
            )