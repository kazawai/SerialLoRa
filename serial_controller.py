import serial
from serial import SerialException, SerialTimeoutException
from serial.tools import list_ports
import sys
import signal
import time
import socket
from colors import bcolors

# Import the dictionary that maps the serial number to the device ID
# Replace with your own dictionary
from serials_list import SERIAL_TO_ID

D_TIME_BETWEEN_PORTS = 1


# SERIAL_PORT_LIST = [f"/dev/ttyACM{i}" for i in range(36)]
DEVICE_LIST = list(list_ports.grep("ACM"))
SERIAL_PORT_LIST = [(port.device, port.serial_number) for port in DEVICE_LIST]

SERVER_PORT = 12345
SERVER_IP = "localhost"

data_connection = None

if "-r" in sys.argv:
    try:
        data_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        data_connection.connect((SERVER_IP, SERVER_PORT))
    except ConnectionRefusedError:
        print(
            f"{bcolors.FAIL}Connection to server {SERVER_IP}:{SERVER_PORT} refused, impossible to communicate with receiver. Exiting{bcolors.ENDC}",
            file=sys.stderr,
        )
        sys.exit(1)


class SerialController:
    def __init__(self, port, baudrate, id):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.flush()

        beginFlagHex = 0x7E
        self.beginFlag = beginFlagHex.to_bytes(
            (beginFlagHex.bit_length() + 7) // 8, "big"
        )
        endFlagHex = 0x7F
        self.endFlag = endFlagHex.to_bytes((endFlagHex.bit_length() + 7) // 8, "big")
        responseFlagHex = 0x7D
        self.responseFlag = responseFlagHex.to_bytes(
            (responseFlagHex.bit_length() + 7) // 8, "big"
        )
        interruptFlagHex = 0x7C
        self.interruptFlag = interruptFlagHex.to_bytes(
            (interruptFlagHex.bit_length() + 7) // 8, "big"
        )
        self.timeout = 5
        self.device_id = id
        self.response_timeout_limit = 5
        self.skip = False
        self.ser.write_timeout = self.timeout
        self.ser.timeout = self.timeout

    def write(self, data):
        return self.ser.write(data)

    def read(self):
        try:
            if self.ser.in_waiting == 0:
                return b""
            if self.skip:
                return b""
            return self.ser.read()
        except SerialException:
            print(
                f"{bcolors.FAIL}Error reading from port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}",
                file=sys.stderr,
            )
            self.skip = True
            return b""
        except OSError:
            print(
                f"{bcolors.WARNING}I/O error on port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
            )
            self.skip = True

    def readline(self):
        return self.ser.readline()

    def _resend_begin_flag(self, signum=None, frame=None):
        if self.response_timeout_limit == 0:
            print(
                f"{bcolors.FAIL}Timeout limit reached, skipping port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
            )
            self.skip = True
        print(f"{bcolors.WARNING}Timeout, resending Start Flag{bcolors.ENDC}")
        try:
            self.write(self.beginFlag)
            print(f"{bcolors.OKGREEN}Start flag sent{bcolors.ENDC}")
        except SerialException:
            print(
                f"{bcolors.FAIL}Error writing to port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}",
                file=sys.stderr,
            )
            self.skip = True
            return
        self.response_timeout_limit -= 1
        signal.alarm(self.timeout)

    def wait_for_response_flag(self):
        signal.signal(signal.SIGALRM, self._resend_begin_flag)
        signal.alarm(self.timeout)
        if self.skip:
            return
        buffer_ = b""
        try:
            while (r := self.read()) != self.responseFlag:
                if self.skip:
                    signal.alarm(0)
                    return
                buffer_ += r
                if r == b"\n":
                    print(buffer_)
                    # If the buffer contains "Sending packet", send the interruptFlag
                    if b"Sending packet" in buffer_:
                        print(
                            f"{bcolors.WARNING}Sending packet detected, sending Interrupt Flag{bcolors.ENDC}"
                        )
                        self.interrupt()
                    buffer_ = b""
        except SerialTimeoutException:
            signal.alarm(0)
            self._resend_begin_flag()
            self.wait_for_response_flag()
            return
        print(f"{bcolors.OKGREEN}Response flag received{bcolors.ENDC}")
        signal.alarm(0)

    def interrupt(self):
        self.write(self.interruptFlag)
        self.wait_for_flag(self.endFlag)
        print(f"{bcolors.OKGREEN}End flag received{bcolors.ENDC}")

    def wait_for_flag(self, flag):
        while self.read() != flag:
            continue

    def _read_timeout(self, signum, frame):
        print(
            f"{bcolors.WARNING}Timeout while reading data on port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
        )
        print(f"{bcolors.WARNING}Skipping...{bcolors.ENDC}")
        self.skip = True

    def readlines_until(self, until):
        signal.signal(signal.SIGALRM, self._read_timeout)
        signal.alarm(self.timeout)
        buffer = b""
        try:
            while (r := self.read()) != until:
                if self.skip:
                    signal.alarm(0)
                    return

                buffer += r
                if r == b"\n":
                    signal.alarm(0)
                    print(buffer)
                    # Reset buffer
                    buffer = b""
                    signal.alarm(self.timeout)
        except SerialTimeoutException:
            signal.alarm(0)
            self._read_timeout(None, None)
            return
        signal.alarm(0)
        print(buffer)

    def _close_timeout(self, signum, frame):
        print(
            f"{bcolors.WARNING}Timeout while closing port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
        )
        self.skip = True

    def close(self):
        signal.signal(signal.SIGALRM, self._close_timeout)
        signal.alarm(self.timeout)
        self.ser.close()
        signal.alarm(0)


if __name__ == "__main__":
    serial_controller_list = []
    print(f"{bcolors.OKCYAN}Available ports: {SERIAL_PORT_LIST}{bcolors.ENDC}")
    for device in DEVICE_LIST:
        print(
            f"{bcolors.OKCYAN}{'{' + device.product + ',' + device.serial_number + '}'}{bcolors.ENDC}"
        )
    try:
        if any(port[1] not in SERIAL_TO_ID for port in SERIAL_PORT_LIST):
            print(
                f"{bcolors.FAIL}Some devices are not in the SERIAL_TO_ID dictionary, exiting{bcolors.ENDC}",
                file=sys.stderr,
            )
            if data_connection:
                data_connection.send(b"close")
                data_connection.close()
            sys.exit(1)

        # Sort the SERIAL_PORT_LIST based on their ID
        SERIAL_PORT_LIST.sort(key=lambda x: int(SERIAL_TO_ID[x[1]]))
        for port_t in SERIAL_PORT_LIST:
            port = port_t[0]
            device_id = SERIAL_TO_ID[port_t[1]]
            try:
                serial_controller = SerialController(port, 115200, device_id)
                serial_controller_list.append(serial_controller)
                print(
                    f"{bcolors.OKGREEN}Port {port} connected : DEVICE {device_id}{bcolors.ENDC}"
                )
            except serial.SerialException:
                print(
                    f"{bcolors.WARNING}Port {port} is not available : DEVICE {device_id}{bcolors.ENDC}",
                    file=sys.stderr,
                )

        if not serial_controller_list:
            print(
                f"{bcolors.FAIL}No available port, exiting{bcolors.ENDC}",
                file=sys.stderr,
            )
            if data_connection:
                data_connection.send(b"close")
                data_connection.close()
            sys.exit(1)

        for serial_controller in serial_controller_list:
            print(
                f"{bcolors.OKCYAN}Writing to port {serial_controller.port} : DEVICE {serial_controller.device_id}{bcolors.ENDC}"
            )
            if data_connection:
                data_connection.send(serial_controller.device_id.encode())
            try:
                serial_controller.write(serial_controller.beginFlag)
                print(f"{bcolors.OKGREEN}Start flag sent{bcolors.ENDC}")
            except SerialException:
                print(
                    f"{bcolors.FAIL}Error writing to port {serial_controller.port} : DEVICE {serial_controller.device_id}{bcolors.ENDC}",
                    file=sys.stderr,
                )
                time.sleep(D_TIME_BETWEEN_PORTS)
                serial_controller.close()
                continue
            try:
                serial_controller.wait_for_response_flag()
                if serial_controller.skip:
                    serial_controller.close()
                    time.sleep(D_TIME_BETWEEN_PORTS)
                    continue
                serial_controller.readlines_until(serial_controller.endFlag)
            except SerialException:
                print(
                    f"{bcolors.FAIL}Error reading from port {serial_controller.port} : DEVICE {serial_controller.device_id}{bcolors.ENDC}",
                    file=sys.stderr,
                )
            serial_controller.close()
            print(
                f"{bcolors.OKGREEN}Port {serial_controller.port} closed : DEVICE {serial_controller.device_id}{bcolors.ENDC}"
            )
            # Wait for 1 second before writing to the next port
            time.sleep(D_TIME_BETWEEN_PORTS)
    except KeyboardInterrupt:
        for serial_controller in serial_controller_list:
            serial_controller.close()
            print(
                f"{bcolors.WARNING}Port {serial_controller.port} closed{bcolors.ENDC}"
            )
        if data_connection:
            data_connection.send(b"close")
            data_connection.close()
        sys.exit(0)

    print(f"{bcolors.OKGREEN}All ports closed{bcolors.ENDC}")
    if data_connection:
        data_connection.send(b"close")
        data_connection.close()
    sys.exit(0)
