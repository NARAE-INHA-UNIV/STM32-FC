# pip isntall pyserial questionary
import os
import serial.tools.list_ports
import questionary
import keyboard

from lib.MAVLink import *


def choose_serial_port():
    answer = questionary.select(
        "Choose Serial Port :",
        choices = [f"{port.device} - {port.description}" for port in serial.tools.list_ports.comports()]
    ).ask()
    return answer.split(" - ")[0]

def choose_buadrate():
    answer = questionary.select(
        "Choose baudrate :",
        default='57600',
        choices = ['9600', '57600', '115200', "(input)"]
    ).ask()

    if(answer!="INPUT"): return int(answer)
    try:
        answer = int(input("INPUT baudrate :"))
    except Exception as err:
        print(err)

    return choose_buadrate()

def choose_log():

    answer = questionary.select(
        "Choose MSG :",
        choices=[f"{handler[0]} : {handler[1].__name__}" for handler in list(handlerDict.items())]
    ).ask()
    return int(answer.split(" : ")[0])
    
try:
    port = choose_serial_port()
    baudrate = choose_buadrate()
    logType = choose_log()
except Exception as err:
    print(err)

mav = MAVLink(port, baudrate, logType)

while True:
    mav.getData()
    if keyboard.is_pressed('q'): break
    if keyboard.is_pressed('s'): 
        mav.setLog(choose_log())
