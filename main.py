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
import network
import sys

g_accel_abs = list(range(32))
g_accel_smooth = list(range(32))
g_speed = list(range(32))
g_speed_smooth = list(range(32))
g_speed_filtered = list(range(32))
g_speed_filtered_smooth = list(range(32))
g_trigger = False
g_alpha = 0.02

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
        # wifi.connect(self.cfg['networks'])
        # while not wifi.isconnected():
        #     time.sleep_ms(50)
        # print("wifi connected")
        # ubirch data client for setting up ubirch protocol, authentication and data service
        self.ubirch_data = UbirchDataClient(self.uuid, self.cfg)

        # disable blue heartbeat blink
        pycom.heartbeat(False)

        # initialise the accelerometer
        self.pysense = pyboard.Pysense()

        # taken from LIS2HH12.py
        #   ARG => threshold
        #   3 => max 8G; resolution: 125   micro G
        #   2 => max 4G; resolution: 62.5  micro G
        #   0 => max 2G; resolution: 31.25 micro G
        self.pysense.accelerometer.set_full_scale(3)

        # taken from LIS2HH12.py
        #   ARG => duration
        #   0 => POWER DOWN
        #   1 => 10  Hz; resolution: 800 milli seconds; max duration: 204000 ms
        #   2 => 50  Hz; resolution: 160 milli seconds; max duration: 40800  ms
        #   3 => 100 Hz; resolution: 80  milli seconds; max duration: 20400  ms
        #   4 => 200 Hz; resolution: 40  milli seconds; max duration: 10200  ms
        #   5 => 400 Hz; resolution: 20  milli seconds; max duration: 5100   ms
        #   6 => 500 Hz; resolution: 10  milli seconds; max duration: 2550   ms
        self.pysense.accelerometer.set_odr(5)

        # enable activity interrupt
        print("start")
        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.get_all_register()
        self.pysense.accelerometer.setup_fifo()
        self.pysense.accelerometer.enable_fifo_interrupt(self.interrup_cb)
        print("int enabled")
        self.pysense.accelerometer.get_all_register()

    def interrup_cb(self, pin):
        global g_accel_abs
        global g_trigger
        self.pysense.accelerometer.enable_fifo_interrupt(handler = None)

        try:
            accel_tuple = self.pysense.accelerometer.acceleration()
            accel_list = list(accel_tuple)
        except Exception as e:
            print("ERROR      can't read data:", e)

        # get data
        for i in range(32):
            try:
                accel_tuple = self.pysense.accelerometer.acceleration()
                accel_list.extend(accel_tuple)
            except Exception as e:
                print("ERROR      can't read data:", e)
        g_accel_abs = list(range(32))
        x=0
        while x < 32:
            g_accel_abs[x] = math.sqrt(
                accel_list[x * 3 + 0] * accel_list[x * 3 + 0] +
                accel_list[x * 3 + 1] * accel_list[x * 3 + 1] +
                accel_list[x * 3 + 2] * accel_list[x * 3 + 2])
            x+=1
        # print(g_accel_abs)
        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.enable_fifo_interrupt(self.interrup_cb)
        g_trigger = True



    def send_data(self, data):
        try:
            print("SENDING:", data)
            self.ubirch_data.send(data)
        except Exception as e:
            print("ERROR      sending data to ubirch:", e)
            time.sleep(3)

    def calc_speed(self):
        global g_accel_abs, g_accel_smooth, g_speed, g_speed_smooth, g_speed_filtered, g_trigger, g_alpha
        # print("calcing")
        # close the ring
        g_accel_smooth[0] = g_alpha * g_accel_abs[0] + (1 - g_alpha) * g_accel_smooth[-1]
        g_speed[0] = g_speed[-1] + g_accel_abs[0] - g_accel_smooth[0]
        g_speed_smooth[0] = g_alpha * g_speed[0] + (1 - g_alpha) * g_speed_smooth[-1]
        g_speed_filtered[0] = g_speed[0] - g_speed_smooth[0]
        g_speed_filtered_smooth[0] = g_alpha * g_speed_filtered[0] + (1 - g_alpha) * g_speed_filtered_smooth[-1]
        # run through the ring
        i = 1
        while i < 32:
            g_accel_smooth[i] = g_alpha * g_accel_abs[i] + (1 - g_alpha) * g_accel_smooth[i-1]
            g_speed[i] = g_speed[i-1] + g_accel_abs[i] - g_accel_smooth[i]
            g_speed_smooth[i] = g_alpha * g_speed[i] + (1 - g_alpha) * g_speed_smooth[i-1]
            g_speed_filtered[i] = g_speed[i] - g_speed_smooth[i]
            g_speed_filtered_smooth[i] = g_alpha * g_speed_filtered[i] + (1 - g_alpha) * g_speed_filtered_smooth[i-1]
            i+=1
        # print("calc_finished")
        print(g_speed_filtered_smooth, end= '', flush=True)

    def read_loop(self):
        global g_accel_abs, g_accel_smooth, g_speed, g_speed_smooth, g_speed_filtered, g_trigger, g_alpha
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
            # print("time:", start_time)
            if g_trigger:
                g_trigger = False
                self.calc_speed()

            # make sure device is still connected
            # if not wlan.isconnected():
            #     wifi.connect(self.cfg['networks'])

            # # get data
            # try:
            #     accel = self.pysense.accelerometer.acceleration()
            #     print("M[%6d]  x: %10f # y: %10f # z: %10f" % (total_measurements, accel[0], accel[1], accel[2]))
            # except Exception as e:
            #     print("ERROR      can't read data:", e)

            total_measurements += 1

            # # set values
            # if accel[0] < 0:
            #     if abs(accel[0]) > data['IAccX']:
            #         data['AccX'] = 0
            #         data['IAccX'] = abs(accel[0])
            # else:
            #     if accel[0] > data['AccX']:
            #         data['AccX'] = accel[0]
            #         data['IAccX'] = 0
            #
            # if accel[1] < 0:
            #     if abs(accel[1]) > data['IAccY']:
            #         data['AccY'] = 0
            #         data['IAccY'] = abs(accel[1])
            # else:
            #     if accel[1] > data['AccY']:
            #         data['AccY'] = accel[1]
            #         data['IAccY'] = 0
            #
            # if accel[2] < 0:
            #     if abs(accel[2]) > data['IAccZ']:
            #         data['AccZ'] = 0
            #         data['IAccZ'] = abs(accel[2])
            # else:
            #     if accel[2] > data['AccZ']:
            #         data['AccZ'] = accel[2]
            #         data['IAccZ'] = 0

            # send data to ubirch data service and certificate to ubirch auth service
            if total_measurements % s_interval == 0:
                # _thread.start_new_thread(self.send_data, [data.copy()])

                # reset data
                data = {
                    "AccX": 0,
                    "AccY": 0,
                    "AccZ": 0,
                    "IAccX": 0,
                    "IAccY": 0,
                    "IAccZ": 0,
                }

            # passed_time = time.time() - start_time
            # if m_interval > passed_time:
            #     time.sleep(m_interval - passed_time)

main = Main()
main.read_loop()
