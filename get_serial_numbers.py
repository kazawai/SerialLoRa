from serial.tools import list_ports
from colors import bcolors

DEVICE_LIST = list(list_ports.grep("ACM"))

SERIAL_PORT_LIST = [port.device for port in DEVICE_LIST]

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

print(f"{bcolors.OKCYAN}Available devices: {bcolors.ENDC}")
for device in DEVICE_LIST:
    print(
        f"{bcolors.OKCYAN}{'{' + device.product + ',' + device.serial_number + ',' + SERIAL_TO_ID[device.serial_number] + '}'}{bcolors.ENDC}"
    )
