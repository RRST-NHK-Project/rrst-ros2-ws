import smbus
import time
import math

class WT901C:
    def __init__(self, bus_number=1, address=0x1d):
        self.bus = smbus.SMBus("/dev/ttyUSB0")
        self.address = address

    def read_data(self):
        # 加速度データの取得
        accel_data = self.bus.read_i2c_block_data(self.address, 0x28, 6)
        accel_x = self.convert_to_signed(accel_data[0], accel_data[1]) / 16384.0
        accel_y = self.convert_to_signed(accel_data[2], accel_data[3]) / 16384.0
        accel_z = self.convert_to_signed(accel_data[4], accel_data[5]) / 16384.0

        # ジャイロデータの取得
        gyro_data = self.bus.read_i2c_block_data(self.address, 0x1c, 6)
        gyro_x = self.convert_to_signed(gyro_data[0], gyro_data[1])
        gyro_y = self.convert_to_signed(gyro_data[2], gyro_data[3])
        gyro_z = self.convert_to_signed(gyro_data[4], gyro_data[5])

        # 磁力計データの取得
        mag_data = self.bus.read_i2c_block_data(self.address, 0x04, 6)
        mag_x = self.convert_to_signed(mag_data[0], mag_data[1])
        mag_y = self.convert_to_signed(mag_data[2], mag_data[3])
        mag_z = self.convert_to_signed(mag_data[4], mag_data[5])

        return (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), (mag_x, mag_y, mag_z)

    @staticmethod
    def convert_to_signed(high, low):
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value

def calculate_yaw(accel, mag):
    accel_x, accel_y, accel_z = accel
    mag_x, mag_y, mag_z = mag

    # 磁力計からヨー角を計算
    yaw = math.atan2(mag_y, mag_x)
    yaw = math.degrees(yaw)  # ラジアンから度に変換
    return yaw

if __name__ == "__main__":
    sensor = WT901C()
    while True:
        accel, gyro, mag = sensor.read_data()
        yaw = calculate_yaw(accel, mag)
        print(f"Yaw: {yaw:.2f} degrees")
        time.sleep(0.5)
