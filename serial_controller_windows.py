import serial
from serial import SerialException, SerialTimeoutException
from serial.tools import list_ports
import sys
import threading
import time
import socket
from colors import bcolors

# Import the dictionary that maps the serial number to the device ID
# Replace with your own dictionary
from serials_list import SERIAL_TO_ID

D_TIME_BETWEEN_PORTS = 7.5

# SERIAL_PORT_LIST = [f"/dev/ttyACM{i}" for i in range(36)]
DEVICE_LIST = list(list_ports.grep("COM"))
SERIAL_PORT_LIST = [(port.device, port.serial_number) for port in DEVICE_LIST]

SERVER_PORT = 12345
SERVER_IP = "192.168.145.200"
# SERVER_IP = "localhost"

SF = 7
BW = 125

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
        initFlagHex = 0x7B
        self.initFlag = initFlagHex.to_bytes((initFlagHex.bit_length() + 7) // 8, "big")

        self.timeout = 5
        self.device_id = id
        self.response_timeout_limit = 5
        self.skip = False
        self.ser.write_timeout = self.timeout
        self.ser.timeout = self.timeout

        self.timeout_thread = None
        self.timeout_event = threading.Event()

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

    def _timeout_handler(self, func, *args):
        time.sleep(self.timeout)
        if not self.timeout_event.is_set():
            func(*args)

    def _start_timeout(self, func, *args):
        self.timeout_event.clear()
        self.timeout_thread = threading.Thread(target=self._timeout_handler, args=(func, *args))
        self.timeout_thread.start()

    def _cancel_timeout(self):
        self.timeout_event.set()
        if self.timeout_thread:
            self.timeout_thread.join()

    def _resend_begin_flag(self):
        if self.response_timeout_limit == 0:
            print(
                f"{bcolors.FAIL}Timeout limit reached, skipping port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
            )
            self.skip = True
            return
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
        self._start_timeout(self._resend_begin_flag)

    def wait_for_response_flag(self, timeout_func=None):
        self._start_timeout(timeout_func)
        if self.skip:
            return
        buffer_ = b""
        try:
            while (r := self.read()) != self.responseFlag:
                if self.skip:
                    self._cancel_timeout()
                    return
                buffer_ += r
                if r == b"\n":
                    print(buffer_)
                    if b"Sending packet" in buffer_:
                        print(
                            f"{bcolors.WARNING}Sending packet detected, sending Interrupt Flag{bcolors.ENDC}"
                        )
                        self.interrupt()
                    buffer_ = b""
        except SerialTimeoutException:
            self._cancel_timeout()
            self._resend_begin_flag()
            self.wait_for_response_flag(timeout_func)
            return
        print(f"{bcolors.OKGREEN}Response flag received{bcolors.ENDC}")
        self._cancel_timeout()

    def interrupt(self):
        self.write(self.interruptFlag)
        self.wait_for_flag(self.endFlag)
        print(f"{bcolors.OKGREEN}End flag received{bcolors.ENDC}")

    def init_transmission(self, sf=7, bw=125):
        self.write(self.initFlag)
        self.write(sf.to_bytes(1, "big"))
        self.write(bw.to_bytes(1, "big"))
        self.wait_for_response_flag(self._resend_init_flag)

    def _resend_init_flag(self):
        print(f"{bcolors.WARNING}Timeout, resending Init Flag{bcolors.ENDC}")
        self.init_transmission()
        self.wait_for_response_flag(self._resend_init_flag)

    def wait_for_flag(self, flag):
        while self.read() != flag:
            continue

    def _read_timeout(self):
        print(
            f"{bcolors.WARNING}Timeout while reading data on port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
        )
        print(f"{bcolors.WARNING}Skipping...{bcolors.ENDC}")
        self.skip = True

    def readlines_until(self, until):
        self._start_timeout(self._read_timeout)
        buffer = b""
        lines_printed = 0
        try:
            while (r := self.read()) != until:
                if self.skip:
                    self._cancel_timeout()
                    return

                buffer += r
                if r == b"\n":
                    self._cancel_timeout()
                    print(buffer)
                    lines_printed += 1
                    buffer = b""
                    self._start_timeout(self._read_timeout)
                if lines_printed == 200:
                    if data_connection:
                        data_connection.send(self.device_id.encode())
                    lines_printed = 0
        except SerialTimeoutException:
            self._cancel_timeout()
            self._read_timeout()
            return
        self._cancel_timeout()
        print(buffer)

    def _close_timeout(self):
        print(
            f"{bcolors.WARNING}Timeout while closing port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
        )
        self.skip = True

    def close(self):
        self._start_timeout(self._close_timeout)
        self.ser.close()
        self._cancel_timeout()


curr_device_id = None

if __name__ == "__main__":

    def ack_timeout_handler():
        print(
            f"{bcolors.FAIL}Timeout while waiting for ACK, resending...{bcolors.ENDC}"
        )
        if curr_device_id is None:
            print(
                f"{bcolors.FAIL}Device ID is None, exiting{bcolors.ENDC}",
                file=sys.stderr,
            )
            exit(1)

        send_new_device_id(curr_device_id)

    def send_new_device_id(d_id):
        if not data_connection:
            return

        timeout_event = threading.Event()
        timeout_thread = threading.Thread(target=lambda: (time.sleep(5), timeout_event.set()))
        timeout_thread.start()

        print(f"Device ID: {d_id}")
        r = data_connection.send(d_id.encode())
        if r == 0:
            print(
                f"{bcolors.FAIL}Error sending device ID to receiver, exiting{bcolors.ENDC}",
                file=sys.stderr,
            )
            exit(1)
        print(f"{bcolors.OKCYAN}Device ID sent to receiver{bcolors.ENDC}")

        while not timeout_event.is_set():
            try:
                response = data_connection.recv(1024)
                if response == b"ACK":
                    print(f"{bcolors.OKGREEN}ACK received from receiver{bcolors.ENDC}")
                    break
            except socket.timeout:
                continue
        if timeout_event.is_set():
            ack_timeout_handler()

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

        i = 0
        for serial_controller in serial_controller_list:
            print(
                f"{bcolors.OKCYAN}Writing to port {serial_controller.port} : DEVICE {serial_controller.device_id}{bcolors.ENDC}"
            )
            if data_connection:
                curr_device_id = serial_controller.device_id
                send_new_device_id(serial_controller.device_id)
                time.sleep(D_TIME_BETWEEN_PORTS)
            try:
                serial_controller.init_transmission(SF, BW)
                print(f"{bcolors.OKGREEN}Transmission initialized{bcolors.ENDC}")
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
                serial_controller.wait_for_response_flag(
                    serial_controller._resend_begin_flag
                )
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
            time.sleep(D_TIME_BETWEEN_PORTS)
    except KeyboardInterrupt:
        for serial_controller in serial_controller_list:
            serial_controller.interrupt()
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
