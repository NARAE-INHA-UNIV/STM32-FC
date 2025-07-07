# pip isntall pyserial questionary
import os
import questionary
import serial.tools.list_ports

from lib.MSG import *
from lib.MAVLink import *


def choose_serial_port():
    answer = questionary.select(
        "Choose Serial Port:",
        choices=[f"{port.device} - {port.description}" for port in serial.tools.list_ports.comports()]
    ).ask()
    return answer.split(" - ")[0]


print(os.listdir("./lib/MSG"))
# mav = RAW_IMU('COM3', 0, 1000, 4)
# mav = RC_CHANNELS('COM3')
# mav = SCALED_IMU2('COM4')

port = choose_serial_port()
mav = SCALED_IMU2(port)

while True:
    mav.getData()
