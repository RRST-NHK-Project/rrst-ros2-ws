import atexit
from dataclasses import dataclass
from enum import IntEnum

import serial


@dataclass
class VersionInfo:
    model: int
    hardware_version: int
    firmware_version_major: int
    firmware_version_minor: int
    serial_number: int


class BaudRateHex(IntEnum):
    BAUD_230400 = 0x00
    BAUD_460800 = 0x01
    BAUD_512000 = 0x02  # not supported by YDLIDAR USB ADAPTER BOARD
    BAUD_921600 = 0x03
    BAUD_1500000 = 0x04  # not supported by YDLIDAR USB ADAPTER BOARD


class BaudRate(IntEnum):
    BAUD_230400 = 230400
    BAUD_460800 = 460800
    BAUD_512000 = 512000  # not supported by YDLIDAR USB ADAPTER BOARD
    BAUD_921600 = 921600
    BAUD_1500000 = 1500000  # not supported by YDLIDAR USB ADAPTER BOARD


class OutputFreqHex(IntEnum):
    Freq_10Hz = 0x00
    Freq_100Hz = 0x01
    Freq_200Hz = 0x02
    Freq_500Hz = 0x03
    Freq_1000Hz = 0x04
    Freq_1800Hz = 0x05


class OutputDataFormatHex(IntEnum):
    Standard = 0x00
    Pixhawk = 0x01


class FilterHex(IntEnum):
    Off = 0x00
    On = 0x01


class CheckSumError(Exception):
    pass


class FailedToReadError(Exception):
    pass


class SelfTestFailedError(Exception):
    pass


class LidarScanningError(Exception):
    pass


PACKET_HED1 = 0xAA
PACKET_HED2 = 0x55
START_SCAN = 0x60
STOP_SCAN = 0x61
GET_DEVICE_INFO = 0x62
SELF_TEST = 0x63
SET_OUTPUT_FREQ = 0x64
SET_FILTER = 0x65
SET_SERIAL_BAUD = 0x66
SET_FORMAT_OUTPUT_DATA = 0x67
RESTORE_FACTORY_SETTINGS = 0x68

NO_DATA = 0x00


class SDM15(object):
    """
    class for SDM15 serial communication
    """

    def __init__(self, port: str, baud_rate: BaudRate = BaudRate.BAUD_460800):
        """setup serial port

        Args:
            port (str): serial port name
            baud_rate (BaudRate, optional): baud rate. Warning: ydlidar usb adapter board does not support baud rate 512000 and 1500000. Defaults to BaudRate.BAUD_460800.

        Raises:
            Exception: serial port is not opened
        """
        self.ser = serial.Serial(port=port, baudrate=baud_rate)

        # check serial port is opened
        if not self.ser.is_open:
            raise Exception("serial port is not opened")

        self.scanning = False
        self.pixhawk = False

        atexit.register(self._at_exit)

    def _at_exit(self):
        """close serial port when program exit"""
        self.stop_scan()
        self.ser.close()
        print("serial port is closed")

    def get_cmd_type(self, cmd: int) -> str:
        """get command type from hex

        Args:
            cmd (int): command hex

        Returns:
            str: command type
        """

        if cmd == START_SCAN:
            return "START_SCAN"
        elif cmd == STOP_SCAN:
            return "STOP_SCAN"
        elif cmd == GET_DEVICE_INFO:
            return "GET_DEVICE_INFO"
        elif cmd == SELF_TEST:
            return "SELF_TEST"
        elif cmd == SET_OUTPUT_FREQ:
            return "SET_OUTPUT_FREQ"
        elif cmd == SET_FILTER:
            return "SET_FILTER"
        elif cmd == SET_SERIAL_BAUD:
            return "SET_SERIAL_BAUD"
        elif cmd == SET_FORMAT_OUTPUT_DATA:
            return "SET_FORMAT_OUTPUT_DATA"
        elif cmd == RESTORE_FACTORY_SETTINGS:
            return "RESTORE_FACTORY_SETTINGS"
        else:
            return "UNKNOWN"

    @staticmethod
    def check(data: list[int]) -> int:
        """calculate check sum

        Args:
            data (list[int]): data to calculate check sum

        Returns:
            int: check sum
        """

        # sum of data
        check_sum = sum(data)

        # get lsb
        check_sum = check_sum & 0xFF

        return check_sum

    def _reset_buffer(self):
        """reset serial buffer"""
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def _write(self, cmd: bytes):
        """write command to serial port"""
        self._reset_buffer()
        self.ser.write(cmd)
        self.ser.flush()

    def _read(self) -> list:
        """receive data from serial port"""

        # wait until data is received
        while self.ser.in_waiting == 0:
            pass

        # read all data
        recv = self.ser.read_all()

        # check data is received
        if recv is None or len(recv) == 0:
            raise FailedToReadError("no data received")

        # convert to hex
        recv_hex = recv.hex(":").split(":")

        # check pixhawk
        if recv_hex[0] != "aa" or recv_hex[1] != "55":
            distance = float(
                recv.decode("utf-8").replace("[Master]: ", "").replace("\r\n", "")
            )
            self.pixhawk = True

            return [distance]

        # convert to int
        recv_hex = [int(x, 16) for x in recv_hex]

        # check sum
        check_sum = recv_hex[-1]
        cal_check_sum = self.check(recv_hex[0:-1])

        # check check_sum
        if cal_check_sum != check_sum:
            print(f"check sum error: {check_sum} != {cal_check_sum}")
            # raise CheckSumError("check sum error")

        return recv_hex

    def check_scanning(self):
        """check lidar is scanning because some commands can only be executed when lidar is not scanning

        Raises:
            LidarScanningError: lidar is scanning
        """

        # check lidar is scanning
        if self.scanning:
            raise LidarScanningError("lidar is scanning")

    def start_scan(self):
        """start scan"""

        # create command
        cmd = bytes([PACKET_HED1, PACKET_HED2, START_SCAN, NO_DATA, 0x5F])

        # write command
        self._write(cmd)

        # drop first data
        self._read()

        # set scanning to True
        self.scanning = True

    def stop_scan(self):
        """stop scan"""

        # create command
        cmd = bytes([PACKET_HED1, PACKET_HED2, STOP_SCAN, NO_DATA, 0x60])

        self._write(cmd)
        self._read()

        # set scanning to False
        self.scanning = False

    def obtain_version_info(self) -> VersionInfo:
        """obtain version info from lidar

        Returns:
            VersionInfo: version info
        """

        # check lidar is scanning
        self.check_scanning()

        # create command
        cmd = bytes([PACKET_HED1, PACKET_HED2, GET_DEVICE_INFO, NO_DATA, 0x61])

        self._write(cmd)
        recv = self._read()

        # get data segment length
        data_len = recv[3]

        # get data segment
        data_segment = recv[4 : 4 + data_len]

        # get serial number
        serial_number = data_segment[4 : 4 + data_len]
        serial_number = int("".join([str(x) for x in serial_number]))

        # create VersionInfo object
        version_info = VersionInfo(
            model=data_segment[0],
            hardware_version=data_segment[1],
            firmware_version_major=data_segment[2],
            firmware_version_minor=data_segment[3],
            serial_number=serial_number,
        )

        return version_info

    def lidar_self_test(self) -> list[int]:
        """lidar self test

        Raises:
            SelfTestFailedError: self test failed

        Returns:
            list[int]: self test data
        """

        # check lidar is scanning
        self.check_scanning()

        cmd = bytes([PACKET_HED1, PACKET_HED2, SELF_TEST, NO_DATA, 0x62])
        self._write(cmd)
        recv = self._read()

        # header = recv[0:2]
        # cmd_type = recv[2]
        # print(self.get_cmd_type(cmd_type))

        data_len = recv[3]
        data_segment = recv[4 : 4 + data_len]

        self_test_result = data_segment[0]
        self_test_error_code = data_segment[1]

        # check self test result
        if self_test_result != 0x01:
            raise SelfTestFailedError(
                f"self test failed error_code: {self_test_error_code}"
            )

        self_test_data = data_segment[2:]

        return self_test_data

    def get_distance(self) -> tuple[int, int, int]:
        """get distance, intensity and disturb

        Returns:
            tuple[int, int, int]: distance, intensity and disturb. If pixhawk is True, intensity and disturb will be -1
        """
        recv = self._read()

        # if pixhawk is True, distance will be returned
        if self.pixhawk:
            distance = recv[0]

            return distance, -1, -1

        data_len = recv[3]
        data_segment = recv[4 : 4 + data_len]

        distance_low = data_segment[0]
        distance_high = data_segment[1]

        # combine distance_low and distance_high
        distance = (distance_high << 8) | distance_low
        intensity = data_segment[2]
        disturb = data_segment[3]

        return distance, intensity, disturb

    def set_output_freq(self, freq: OutputFreqHex = OutputFreqHex.Freq_100Hz):
        """set output frequency

        Args:
            freq (OutputFreqHex, optional): data output frequency. Defaults to OutputFreqHex.Freq_100Hz.

        Raises:
            Exception: set output freq failed
        """

        # check lidar is scanning
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, SET_OUTPUT_FREQ, 0x01, freq]

        # calculate check sum
        check_sum = self.check(cmd)

        cmd.append(check_sum)
        cmd = bytes(cmd)
        self._write(cmd)
        recv = self._read()

        recv_freq = recv[4]

        # check recv_freq
        if recv_freq != freq:
            raise Exception("set output freq failed")

        if recv_freq == OutputFreqHex.Freq_10Hz:
            print("set output freq to 10Hz")
        elif recv_freq == OutputFreqHex.Freq_100Hz:
            print("set output freq to 100Hz")
        elif recv_freq == OutputFreqHex.Freq_200Hz:
            print("set output freq to 200Hz")
        elif recv_freq == OutputFreqHex.Freq_500Hz:
            print("set output freq to 500Hz")
        elif recv_freq == OutputFreqHex.Freq_1000Hz:
            print("set output freq to 1000Hz")
        elif recv_freq == OutputFreqHex.Freq_1800Hz:
            print("set output freq to 1800Hz")

        self._reset_buffer()

    def set_filter(self, filter: FilterHex = FilterHex.On):
        """set filter on or off

        Args:
            filter (FilterHex, optional): filter on or off. Defaults to FilterHex.On.

        Raises:
            Exception: _description_
        """
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, SET_FILTER, 0x01, filter]
        check_sum = self.check(cmd)
        cmd.append(check_sum)
        cmd = bytes(cmd)
        self._write(cmd)
        recv = self._read()

        recv_filter = recv[4]

        if recv_filter != filter:
            raise Exception("set filter failed")

        if recv_filter == FilterHex.Off:
            print("set filter to off")
        elif recv_filter == FilterHex.On:
            print("set filter to on")

        self._reset_buffer()

    def set_baud_rate(self, baud_rate: BaudRateHex = BaudRateHex.BAUD_460800):
        """set baud rate of lidar

        Args:
            baud_rate (BaudRateHex, optional): baud rate. Warning: ydlidar usb adapter board does not support baud rate 512000 and 1500000. Defaults to BaudRateHex.BAUD_460800.

        Raises:
            Exception: set baud rate failed
        """
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, SET_SERIAL_BAUD, 0x01, baud_rate]
        check_sum = self.check(cmd)
        cmd.append(check_sum)
        cmd = bytes(cmd)
        self._write(cmd)
        recv = self._read()

        recv_baud_rate = recv[4]

        if recv_baud_rate != baud_rate:
            raise Exception("set baud rate failed")

        if recv_baud_rate == BaudRateHex.BAUD_230400:
            print("set baud rate to 230400")
        elif recv_baud_rate == BaudRateHex.BAUD_460800:
            print("set baud rate to 460800")
        elif recv_baud_rate == BaudRateHex.BAUD_512000:
            print("set baud rate to 512000")
        elif recv_baud_rate == BaudRateHex.BAUD_921600:
            print("set baud rate to 921600")
        elif recv_baud_rate == BaudRateHex.BAUD_1500000:
            print("set baud rate to 1500000")

        self._reset_buffer()

    def set_output_data_format(
        self, data_format: OutputDataFormatHex = OutputDataFormatHex.Standard
    ):
        """set output data format. Standard or Pixhawk

        Args:
            data_format (OutputDataFormatHex, optional): output data format. Defaults to OutputDataFormatHex.Standard.
        Raises:
            Exception: set output data format failed
        """
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, SET_FORMAT_OUTPUT_DATA, 0x01, data_format]
        check_sum = self.check(cmd)
        cmd.append(check_sum)
        cmd = bytes(cmd)
        self._write(cmd)
        recv = self._read()

        recv_data_format = recv[4]

        if recv_data_format != data_format:
            raise Exception("set output data format failed")

        if recv_data_format == OutputDataFormatHex.Standard:
            print("set output data format to standard")
        elif recv_data_format == OutputDataFormatHex.Pixhawk:
            print("set output data format to pixhawk")

        self._reset_buffer()

    def restore_factory_settings(self):
        """restore factory settings"""
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, RESTORE_FACTORY_SETTINGS, 0x00, 0x67]
        cmd = bytes(cmd)

        self._write(cmd)
        self._read()

        self._reset_buffer()
