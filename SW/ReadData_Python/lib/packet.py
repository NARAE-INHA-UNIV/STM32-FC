class packet:
    data = [0 for i in range(0, 255)]
    length = 0

    def checkCRC(self):
        rxCRC = self.data[self.length-2]|self.data[self.length-1]<<8
        retval = calulate_crc(self.data, self.length)
        return retval == rxCRC

def calulate_crc(data, length):
    crc = 0x0000
    for i in range(0, length -2):
        crc ^= (data[i] << 8)
        for j in range(0, 8):
            if (crc & 0x8000):
                crc = (crc << 1) ^ 0x1021
            else:
                crc = (crc << 1)

    return crc&0xffff

def byte2int(x):
    return int("0x"+str(x),16)
