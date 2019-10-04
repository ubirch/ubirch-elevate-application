from machine import *
import utime as time
import struct
import math
# 0x6b = 107 --> GYRO/ACCEL address
i2c = I2C(0, I2C.MASTER)

print("Found I2C devices:", i2c.scan())
print("Resetting accel and gyro ...")
i2c.writeto_mem(0x6b, 0x22, 0x05)
time.sleep(0.1)
print("Enabling gyro ...")
i2c.writeto_mem(0x6b, 0x10, 0xc0)
print("Enabling accel ...")
i2c.writeto_mem(0x6b, 0x1f, 0x38)
i2c.writeto_mem(0x6b, 0x20, 0xc0)
print("Entering gyro/accel read-loop ...")
while True:
    time.sleep(0.2)
    print("Reading gyro ... ", end='')
    i2c.writeto(0x6b, 0x98 & 0xff)
    data = i2c.readfrom_mem(0x6b, 0x98, 6)
    raw_x, raw_y, raw_z = struct.unpack_from('<hhh', data)
    print("X: %d   Y: %d   Z: %d" % (raw_x * 0.00875, raw_y * 0.00875, raw_z * 0.00875))
    print("Reading accel ... ", end="")
    i2c.writeto(0x6b, 0xa8 & 0xff)
    data = i2c.readfrom_mem(0x6b, 0xa8, 6)
    raw_x, raw_y, raw_z = struct.unpack_from('<hhh', data)
    print("X: %d   Y: %d   Z: %d -- %f - %f" % (raw_x * 0.00875, raw_y * 0.00875,
          raw_z * 0.00875, math.atan2(-raw_x, math.sqrt(raw_y * raw_y + raw_z * raw_z)) * 180.0 / math.pi,
          math.atan2(raw_y, raw_z) * 180.0 / math.pi))
