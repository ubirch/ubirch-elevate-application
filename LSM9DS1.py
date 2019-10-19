from machine import *
import utime as time
import struct


LSM9DS1_ADDR = 0x6b
LSM9DS1_CTRL_REG8 = 0x22
LSM9DS1_CTRL_REG8_VRESET = 0x05
LSM9DS1_CTRL_REG1_GYRO = 0x10
LSM9DS1_CTRL_REG5_ACCEL = 0x1f
LSM9DS1_CTRL_REG6_ACCEL = 0x20

LSM9DS1_DATA_REG_ACEL = 0x28
LSM9DS1_DATA_REG_GYRO = 0x18

ACCELRANGE_2G = (0b00 << 3)
ACCELRANGE_16G = (0b01 << 3)
ACCELRANGE_4G = (0b10 << 3)
ACCELRANGE_8G = (0b11 << 3)
_ACCELRANGE_2G = 0.061
_ACCELRANGE_4G = 0.122
_ACCELRANGE_8G = 0.244
_ACCELRANGE_16G = 0.732
GYROSCALE_245DPS = (0b00 << 3)  # +/- 245 degrees/s rotation
GYROSCALE_500DPS = (0b01 << 3)  # +/- 500 degrees/s rotation
GYROSCALE_2000DPS = (0b11 << 3)  # +/- 2000 degrees/s rotation
_GYROSCALE_245DPS = 0.00875
_GYROSCALE_500DPS = 0.01750
_GYROSCALE_2000DPS = 0.07000


# default earth-accel in m/s*s
G = 9.81


class LSM9DS1:
    def __init__(self, pins=('P9', 'P10')):
        # initialise the I2C bus
        print("Initialising I2C bus ...")
        self.i2c = I2C(0, I2C.MASTER, pins=pins)

        # reset the accel and gyro
        print("Resetting accel and gyro ...")
        self.i2c.writeto_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG8, LSM9DS1_CTRL_REG8_VRESET)
        time.sleep(0.1)

        # enable gyro and accel
        print("Enabling accel and gyro ...")
        self.i2c.writeto_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG1_GYRO, 0xc0)
        self.i2c.writeto_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG5_ACCEL, 0x38)
        self.i2c.writeto_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG6_ACCEL, 0xc0)

        # read configuration
        print("Reading config values from accel and gyro ...")
        self.accel_range = self.read_accel_range()
        self.gyro_scale = self.read_gyro_scale()
        print("GyroScale:", self.gyro_scale)
        print("AccelRange:", self.accel_range)

    def read_accel_range(self):
        data = self.i2c.readfrom_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG6_ACCEL, 1)
        val = (data[0] & 0b00011000) & 0xFF
        if val == ACCELRANGE_2G:
            return _ACCELRANGE_2G
        elif val == ACCELRANGE_4G:
            return _ACCELRANGE_4G
        elif val == ACCELRANGE_8G:
            return _ACCELRANGE_8G
        elif val == ACCELRANGE_16G:
            return _ACCELRANGE_16G
        else:
            return 0

    def read_gyro_scale(self):
        data = self.i2c.readfrom_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG1_GYRO, 1)[0]
        val = (data & 0b00011000) & 0xFF

        if val == GYROSCALE_245DPS:
            return _GYROSCALE_245DPS
        elif val == GYROSCALE_500DPS:
            return _GYROSCALE_500DPS
        elif val == GYROSCALE_2000DPS:
            return _GYROSCALE_2000DPS
        else:
            return 0

    def set_accel_range(self, range):
        # check that range is valid
        assert range in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G, ACCELRANGE_16G)

        # read the current state of REG6 and modify it
        data = self.i2c.readfrom_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG6_ACCEL, 1)[0]
        data = (data & ~(0b00011000)) & 0xFF
        data |= range

        # write it back to the device
        self.i2c.writeto_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG6_ACCEL, data)

        # set the correct factor
        if range == ACCELRANGE_2G:
            self.accel_range = _ACCELRANGE_2G
        elif range == ACCELRANGE_4G:
            self.accel_range = _ACCELRANGE_4G
        elif range == ACCELRANGE_8G:
            self.accel_range = _ACCELRANGE_8G
        elif range == ACCELRANGE_16G:
            self.accel_range = _ACCELRANGE_16G

        return

    def set_gyro_scale(self, scale):
        # check that range is valid
        assert range in (GYROSCALE_245DPS, GYROSCALE_500DPS, GYROSCALE_2000DPS)

        # read the current state of REG6 and modify it
        reg = self.i2c.readfrom_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG1_GYRO, 1)[0]
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= range

        # write it back to the device
        self.i2c.writeto_mem(LSM9DS1_ADDR, LSM9DS1_CTRL_REG1_GYRO, reg)

        # set the correct factor
        if range == GYROSCALE_245DPS:
            self.gyro_scale = _GYROSCALE_245DPS
        elif range == GYROSCALE_500DPS:
            self.gyro_scale = _GYROSCALE_500DPS
        elif range == GYROSCALE_2000DPS:
            self.gyro_scale = _GYROSCALE_2000DPS

    def read_accel(self):
        data = self.i2c.readfrom_mem(LSM9DS1_ADDR, 0x80 | LSM9DS1_DATA_REG_ACEL, 6)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', data)

        return (raw_x * self.accel_range / 1000 * G,
                raw_y * self.accel_range / 1000 * G,
                raw_z * self.accel_range / 1000 * G)

    def read_gyro(self):
        data = self.i2c.readfrom_mem(LSM9DS1_ADDR, 0x80 | LSM9DS1_DATA_REG_GYRO, 6)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', data)

        return (raw_x * self.gyro_scale,
                raw_y * self.gyro_scale,
                raw_z * self.gyro_scale)
