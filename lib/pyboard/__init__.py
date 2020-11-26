from .L76GNSS import L76GNSS
from .LIS2HH12 import LIS2HH12
from .LTR329ALS01 import LTR329ALS01
from .MPL3115A2 import MPL3115A2, ALTITUDE, PRESSURE
from .SI7006A20 import SI7006A20
from .pycoproc import Pycoproc


class Pysense(Pycoproc):
    def __init__(self):
        """Initialized sensors on Pysense"""
        super().__init__(i2c=None, sda='P22', scl='P21')
        self.accelerometer = LIS2HH12(self)
        self.light = LTR329ALS01(self).light
        self.humidity = SI7006A20(self)
        self.altimeter = MPL3115A2(self, mode=ALTITUDE)
        self.voltage = self.read_battery_voltage


class Pytrack(Pycoproc):
    def __init__(self):
        """Initialize sensors on Pytrack"""
        super().__init__(i2c=None, sda='P22', scl='P21')

        self.accelerometer = LIS2HH12(self)
        self.location = L76GNSS(self, timeout=30)
        self.voltage = self.read_battery_voltage


def get_pyboard(type: str) -> Pyboard:  # TODO maybe clean up here
    if type == "pysense":
        return Pysense()
    elif type == "pytrack":
        return Pytrack()
    elif type == "sht31":
        return Sht31()
    else:
        raise Exception("Expansion board type {} not supported. Supported types: 'pysense' and 'pytrack'".format(type))


def print_data(data: dict) -> None:
    print("{")
    for key in sorted(data):
        print("  \"{}\": {},".format(key, data[key]))
    print("}\n")
