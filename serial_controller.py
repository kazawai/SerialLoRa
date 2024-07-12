import serial
from serial import SerialException, SerialTimeoutException
from serial.tools import list_ports
import sys
import signal
import time
from colors import bcolors
import socket

D_TIME_BETWEEN_PORTS = 1


# SERIAL_PORT_LIST = [f"/dev/ttyACM{i}" for i in range(36)]
DEVICE_LIST = list(list_ports.grep("ACM"))
SERIAL_PORT_LIST = [(port.device, port.serial_number) for port in DEVICE_LIST]

SERIAL_TO_ID = {
    "7DF194D65030574B412E3120FF191C07": "1",
    "360931A35030574B412E3120FF15190F": "2",
    "0ED7D81E5030574B412E3120FF160839": "3",
    "7B2DD5C55030574B412E3120FF160A14": "4",
    "FFC5ED415030574B412E3120FF130729": "5",
    "EF5CE8325030574B412E3120FF13072D": "6",
    "3B66C4E25030574B412E3120FF130616": "7",
    "54E4203B5030574B412E3120FF13071F": "8",
    "1A925BDC5030574B412E3120FF15062A": "9",
    "B12EED6F5030574B412E3120FF16091E": "10",
    "BB7AFFBE5030574B412E3120FF160810": "11",
    "D46A0F5B5030574B412E3120FF15172E": "12",
    "02BC80115030574B412E3120FF160C15": "13",
    "3B9F09355030574B412E3120FF160C27": "14",
    "5210D2305030574B412E3120FF142A3A": "15",
    "53E0564F5030574B412E3120FF13072C": "16",
    "63174B875030574B412E3120FF15290A": "17",
    "CC5630315030574B412E3120FF180A38": "18",
    "3E321F2A5030574B412E3120FF140421": "19",
    "DABE48365030574B412E3120FF130924": "20",
    "0683FD0F5030574B412E3120FF152739": "21",
    "4E1FD0D35030574B412E3120FF13093B": "22",
    "3955ACBF5030574B412E3120FF191D38": "23",
    "A8E7B2285030574B412E3120FF13072B": "24",
    "15EA004F5030574B412E3120FF160C20": "25",
    "48C909845030574B412E3120FF160B34": "26",
    "DFA6BCB15030574B412E3120FF151711": "27",
    "8C7DE18D5030574B412E3120FF180C15": "28",
    "518680F75030574B412E3120FF152710": "29",
    "8146287150593730372E3120FF031A30": "30",
    "47BFC0915030574B412E3120FF14233D": "31",
    "062B95965030574B412E3120FF191D39": "32",
    "F003CE685030574B412E3120FF130A11": "33",
    "71EFE65E5030574B412E3120FF13061A": "34",
    "E019727B5030574B412E3120FF191B09": "35",
    "31CA30465030574B412E3120FF180C0D": "36",
}

SERVER_PORT = 12345
SERVER_IP = "localhost"

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
        if self.ser.in_waiting == 0:
            return b""
        if self.skip:
            return b""
        return self.ser.read()

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
            data_connection.send(b"close")
            data_connection.close()
            sys.exit(1)

        for serial_controller in serial_controller_list:
            print(
                f"{bcolors.OKCYAN}Writing to port {serial_controller.port} : DEVICE {serial_controller.device_id}{bcolors.ENDC}"
            )
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
        data_connection.send(b"close")
        data_connection.close()
        sys.exit(0)

    print(f"{bcolors.OKGREEN}All ports closed{bcolors.ENDC}")
    data_connection.send(b"close")
    data_connection.close()
    sys.exit(0)
