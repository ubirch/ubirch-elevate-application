import machine
import utime as time
import struct
import math
import wifi
import pyboard
import pycom
import json
import _thread
from ubirch import UbirchDataClient
from uuid import UUID
from network import WLAN
from machine import *

# bigger thread stack needed for the requests module used in UbirchDataClient (default: 4096)
_thread.stack_size(16384)

# address of the LIS2HH12 on the I2C bus
LIS2HH12_ADDR = 30

wlan = WLAN(mode=WLAN.STA)

class Main:
    def __init__(self) -> None:
        self.uuid = UUID(b'UBIR' + 2 * machine.unique_id())
        print("\n** UUID   : " + str(self.uuid) + "\n")

        with open('config.json', 'r') as c:
            self.cfg = json.load(c)

        # try to connect via wifi, throws exception if no success
        wifi.connect(self.cfg['networks'])

        # ubirch data client for setting up ubirch protocol, authentication and data service
        self.ubirch_data = UbirchDataClient(self.uuid, self.cfg)

        # disable blue heartbeat blink
        pycom.heartbeat(False)

        # initialise the accelerometer
        self.pytrack = pyboard.Pytrack()

        # taken from LIS2HH12.py
        #   ARG => threshold
        #   3 => max 8G; resolution: 125   micro G
        #   2 => max 4G; resolution: 62.5  micro G
        #   0 => max 2G; resolution: 31.25 micro G
        self.pytrack.accelerometer.set_full_scale(3)

        # taken from LIS2HH12.py
        #   ARG => duration
        #   0 => POWER DOWN
        #   1 => 10  Hz; resolution: 800 milli seconds; max duration: 204000 ms
        #   2 => 50  Hz; resolution: 160 milli seconds; max duration: 40800  ms
        #   3 => 100 Hz; resolution: 80  milli seconds; max duration: 20400  ms
        #   4 => 200 Hz; resolution: 40  milli seconds; max duration: 10200  ms
        #   5 => 400 Hz; resolution: 20  milli seconds; max duration: 5100   ms
        #   6 => 500 Hz; resolution: 10  milli seconds; max duration: 2550   ms
        self.pytrack.accelerometer.set_odr(6)

        # enable activity interrupt
        self.pytrack.accelerometer.enable_activity_interrupt(self.cfg['interrupt_threshold_mg'], self.cfg['threshold_duration_ms'], self.interrup_cb)

    def interrup_cb(self, pin):
        print("ALARM!")

        # disable interrupt
        self.pytrack.accelerometer.enable_activity_interrupt(self.cfg['interrupt_threshold_mg'], self.cfg['threshold_duration_ms'], None)

        self.send_data({
            "timestamp": time.time(),
            "data": {
                "interrupt_threshold_mg": self.cfg['interrupt_threshold_mg'],
                "threshold_duration_ms": self.cfg['threshold_duration_ms']
            }
        })

        # re-enable interrupt
        self.pytrack.accelerometer.enable_activity_interrupt(self.cfg['interrupt_threshold_mg'], self.cfg['threshold_duration_ms'], self.interrup_cb)

    def send_data(self, data):
        try:
            print("SENDING:", data)
            self.ubirch_data.send(data)
        except Exception as e:
            print("ERROR      sending data to ubirch:", e)
            time.sleep(3)

main = Main()
