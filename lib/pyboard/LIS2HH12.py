import math
import struct
import time

import ubinascii
from machine import Pin

FULL_SCALE_2G = const(0)
FULL_SCALE_4G = const(2)
FULL_SCALE_8G = const(3)

ODR_POWER_DOWN = const(0)
ODR_10_HZ = const(1)
ODR_50_HZ = const(2)
ODR_100_HZ = const(3)
ODR_200_HZ = const(4)
ODR_400_HZ = const(5)
ODR_800_HZ = const(6)

ACC_G_DIV = 1000 * 65536


class LIS2HH12:
    ACC_I2CADDR = const(30)

    PRODUCTID_REG = const(0x0F)
    ACT_THS = const(0x1E)
    ACT_DUR = const(0x1F)
    CTRL1_REG = const(0x20)
    CTRL2_REG = const(0x21)
    CTRL3_REG = const(0x22)
    CTRL4_REG = const(0x23)
    CTRL5_REG = const(0x24)
    CTRL6_REG = const(0x25)
    CTRL7_REG = const(0x26)
    STATUS_REG = const(0x27)
    ACC_X_L_REG = const(0x28)
    ACC_X_H_REG = const(0x29)
    ACC_Y_L_REG = const(0x2A)
    ACC_Y_H_REG = const(0x2B)
    ACC_Z_L_REG = const(0x2C)
    ACC_Z_H_REG = const(0x2D)
    FIFO_CTRL_REG = const(0x2E)
    FIFO_SRC_REG = const(0x2F)

    SCALES = {FULL_SCALE_2G: 4000, FULL_SCALE_4G: 8000, FULL_SCALE_8G: 16000}
    ODRS = [0, 10, 50, 100, 200, 400, 800]

    def __init__(self, pysense=None, sda='P22', scl='P21'):
        if pysense is not None:
            self.i2c = pysense.i2c
        else:
            from machine import I2C
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))

        self.odr = 0
        self.full_scale = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.int_pin = None
        self.act_dur = 0
        self.debounced = False

        whoami = self.i2c.readfrom_mem(ACC_I2CADDR, PRODUCTID_REG, 1)
        if (whoami[0] != 0x41):
            raise ValueError("LIS2HH12 not found")

        # enable acceleration readings at 50Hz
        self.set_odr(ODR_50_HZ)

        # change the full-scale to 4g
        self.set_full_scale(FULL_SCALE_4G)

        # set the interrupt pin as active low and open drain
        self.set_register(CTRL5_REG, 3, 0, 3)

        # make a first read
        self.acceleration()

    FIFO_EN = const(0x01 << 7)  # CTRL3: FIFO enable. Default value 0. (0: disable; 1: enable)
    INT1_OVR = const(0x01 << 2)  # FIFO overrun signal on INT1
    FMODE = const(0x01 << 5)  # FIFO_CTRL: FIFO mode selection: FIFO mode. Stops collecting data when FIFO is full

    def setup_fifo(self):
        # set the register to enable the FIFO
        self.set_register(CTRL3_REG, (FIFO_EN | INT1_OVR), 0, (FIFO_EN | INT1_OVR))
        # set mode to FIFO mode
        self.set_register(FIFO_CTRL_REG, FMODE, 0, FMODE)

    def restart_fifo(self):
        # set mode to FIFO mode
        self.set_register(FIFO_CTRL_REG, 0, 0, FMODE)
        self.set_register(FIFO_CTRL_REG, FMODE, 0, FMODE)

    def enable_fifo_interrupt(self, handler=None):
        # enable the interrupt, which occurs, when fifo is full and set the corresponding handler
        self._user_handler = handler
        self.int_pin = Pin('P13', mode=Pin.IN)
        self.int_pin.callback(trigger=Pin.IRQ_FALLING, handler=self._int_handler)

    def acceleration(self):
        x = self.i2c.readfrom_mem(ACC_I2CADDR, ACC_X_L_REG, 2)
        self.x = struct.unpack('<h', x)
        y = self.i2c.readfrom_mem(ACC_I2CADDR, ACC_Y_L_REG, 2)
        self.y = struct.unpack('<h', y)
        z = self.i2c.readfrom_mem(ACC_I2CADDR, ACC_Z_L_REG, 2)
        self.z = struct.unpack('<h', z)
        _mult = self.SCALES[self.full_scale] / ACC_G_DIV
        return (self.x[0] * _mult, self.y[0] * _mult, self.z[0] * _mult)

    def roll(self):
        x, y, z = self.acceleration()
        rad = math.atan2(-x, z)
        return (180 / math.pi) * rad

    def pitch(self):
        x, y, z = self.acceleration()
        rad = -math.atan2(y, (math.sqrt(x * x + z * z)))
        return (180 / math.pi) * rad

    def get_all_register(self):
        reg = bytearray(self.i2c.readfrom_mem(ACC_I2CADDR, CTRL1_REG, 8))
        print(ubinascii.hexlify(reg))

    def set_register(self, register, value, offset, mask):
        reg = bytearray(self.i2c.readfrom_mem(ACC_I2CADDR, register, 1))
        reg[0] &= ~(mask << offset)
        reg[0] |= ((value & mask) << offset)
        self.i2c.writeto_mem(ACC_I2CADDR, register, reg)

    def set_full_scale(self, scale):
        self.set_register(CTRL4_REG, scale, 4, 3)
        self.full_scale = scale

    def set_odr(self, odr):
        self.set_register(CTRL1_REG, odr, 4, 7)
        self.odr = odr

    def set_high_pass(self, hp):
        self.set_register(CTRL2_REG, 1 if hp else 0, 2, 1)

    def enable_activity_interrupt(self, threshold, duration, handler=None):
        # Threshold is in mg, duration is ms
        self.act_dur = duration

        if threshold > self.SCALES[self.full_scale]:
            error = "threshold %d exceeds full scale %d" % (threshold, self.SCALES[self.full_scale])
            print(error)
            raise ValueError(error)

        if threshold < self.SCALES[self.full_scale] / 128:
            error = "threshold %d below resolution %d" % (threshold, self.SCALES[self.full_scale] / 128)
            print(error)
            raise ValueError(error)

        if duration > 255 * 1000 * 8 / self.ODRS[self.odr]:
            error = "duration %d exceeds max possible value %d" % (duration, 255 * 1000 * 8 / self.ODRS[self.odr])
            print(error)
            raise ValueError(error)

        if duration < 1000 * 8 / self.ODRS[self.odr]:
            error = "duration %d below resolution %d" % (duration, 1000 * 8 / self.ODRS[self.odr])
            print(error)
            raise ValueError(error)

        _ths = int(127 * threshold / self.SCALES[self.full_scale]) & 0x7F
        _dur = int((duration * self.ODRS[self.odr]) / 1000 / 8)

        self.i2c.writeto_mem(ACC_I2CADDR, ACT_THS, _ths)
        self.i2c.writeto_mem(ACC_I2CADDR, ACT_DUR, _dur)

        # enable the activity/inactivity interrupt
        self.set_register(CTRL3_REG, 1, 5, 1)

        self._user_handler = handler
        self.int_pin = Pin('P13', mode=Pin.IN)
        self.int_pin.callback(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._int_handler)

        # return actual used threshold and duration
        return (_ths * self.SCALES[self.full_scale] / 128, _dur * 8 * 1000 / self.ODRS[self.odr])

    def activity(self):
        if not self.debounced:
            time.sleep_ms(self.act_dur)
            self.debounced = True
        if self.int_pin():
            return True
        return False

    def _int_handler(self, pin_o):
        if self._user_handler is not None:
            self._user_handler(pin_o)
        else:
            if pin_o():
                print('Activity interrupt')
            else:
                print('Inactivity interrupt')
