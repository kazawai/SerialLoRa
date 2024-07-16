from serial.tools import list_ports
from colors import bcolors
from serials_list import SERIAL_TO_ID

DEVICE_LIST = list(list_ports.grep("ACM"))

SERIAL_PORT_LIST = [port.device for port in DEVICE_LIST]

print(f"{bcolors.OKCYAN}Available devices: {bcolors.ENDC}")
for device in DEVICE_LIST:
    print(
        f"{bcolors.OKCYAN}{'{' + device.product + ',' + device.serial_number + ',' + SERIAL_TO_ID[device.serial_number] + '}'}{bcolors.ENDC}"
    )
