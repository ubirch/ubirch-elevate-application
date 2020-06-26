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

    def read_loop(self):
        # get intervals
        m_interval = self.cfg['measure_interval_s']
        s_interval = self.cfg['send_interval_measurements']

        total_measurements = 0

        # data dict
        data = {
            "AccX": 0, # positive x accel
            "AccY": 0, # positive y accel
            "AccZ": 0, # positive z accel
            "IAccX": 0, # negative x accel
            "IAccY": 0, # negative y accel
            "IAccZ": 0, # negative z accel
        }

        while True:
            start_time = time.time()

            # make sure device is still connected
            if not wlan.isconnected():
                wifi.connect(self.cfg['networks'])

            # get data
            try:
                accel = self.pytrack.accelerometer.acceleration()
                print("M[%6d]  x: %10f # y: %10f # z: %10f" % (total_measurements, accel[0], accel[1], accel[2]))
            except Exception as e:
                print("ERROR      can't read data:", e)

            total_measurements += 1

            # set values
            if accel[0] < 0:
                if abs(accel[0]) > data['IAccX']:
                    data['AccX'] = 0
                    data['IAccX'] = abs(accel[0])
            else:
                if accel[0] > data['AccX']:
                    data['AccX'] = accel[0]
                    data['IAccX'] = 0

            if accel[1] < 0:
                if abs(accel[1]) > data['IAccY']:
                    data['AccY'] = 0
                    data['IAccY'] = abs(accel[1])
            else:
                if accel[1] > data['AccY']:
                    data['AccY'] = accel[1]
                    data['IAccY'] = 0

            if accel[2] < 0:
                if abs(accel[2]) > data['IAccZ']:
                    data['AccZ'] = 0
                    data['IAccZ'] = abs(accel[2])
            else:
                if accel[2] > data['AccZ']:
                    data['AccZ'] = accel[2]
                    data['IAccZ'] = 0

            # send data to ubirch data service and certificate to ubirch auth service
            if total_measurements % s_interval == 0:
                _thread.start_new_thread(self.send_data, [data.copy()])

                # reset data
                data = {
                    "AccX": 0,
                    "AccY": 0,
                    "AccZ": 0,
                    "IAccX": 0,
                    "IAccY": 0,
                    "IAccZ": 0,
                }

            passed_time = time.time() - start_time
            if m_interval > passed_time:
                time.sleep(m_interval - passed_time)

main = Main()
main.read_loop()
