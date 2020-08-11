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
from state_machine import *



# bigger thread stack needed for the requests module used in UbirchDataClient (default: 4096)
_thread.stack_size(16384)

# address of the LIS2HH12 on the I2C bus
LIS2HH12_ADDR = 30

wlan = WLAN(mode=WLAN.STA)


# while True:
#     root_controller.update()

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

        # create the root controller as a state machine and add all the necessary states
        self.root_controller = StateMachine()
        self.root_controller.add_state(ConnectingState())
        self.root_controller.add_state(SendingVersionDiagnosticsState())
        self.root_controller.add_state(SendingCellularDiagnosticsState())
        self.root_controller.add_state(WaitingForOvershootState())
        self.root_controller.add_state(IdleState())
        self.root_controller.add_state(RaisingState())
        self.root_controller.add_state(PausedState())
        # start with the connecting state
        self.root_controller.go_to_state('connecting')


    # TODO this has to be done later
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

        while True:

            self.root_controller.update()
            # start_time = time.time()
            # # print("time:", start_time)
            # if g_trigger:
            #     g_trigger = False
            #     self.calc_speed()

            # make sure device is still connected
            # if not wlan.isconnected():
            #     wifi.connect(self.cfg['networks'])

            # # get data
            # try:
            #     accel = self.pysense.accelerometer.acceleration()
            #     print("M[%6d]  x: %10f # y: %10f # z: %10f" % (total_measurements, accel[0], accel[1], accel[2]))
            # except Exception as e:
            #     print("ERROR      can't read data:", e)


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


            # passed_time = time.time() - start_time
            # if m_interval > passed_time:
            #     time.sleep(m_interval - passed_time)

main = Main()
main.read_loop()
