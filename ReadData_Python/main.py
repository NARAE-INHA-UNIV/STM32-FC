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

port = choose_serial_port()
mav = SCALED_IMU(port)

while True:
    mav.getData()
