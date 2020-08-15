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

from sensor import MovementSensor

from microWorkers import MicroWorkers
from time         import sleep

print()

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

        # create the root controller as a state machine and add all the necessary states
        self.root_controller = StateMachine()
        self.root_controller.add_state(ConnectingState())
        self.root_controller.add_state(SendingVersionDiagnosticsState())
        self.root_controller.add_state(SendingCellularDiagnosticsState())
        self.root_controller.add_state(WaitingForOvershootState())
        self.root_controller.add_state(MeasuringPausedState())
        self.root_controller.add_state(ErrorState())
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
            time.sleep(0.1)

main = Main()
main.read_loop()
