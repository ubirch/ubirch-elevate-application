import machine
import utime as time
import struct

import wifi
from pyboard import *
import pycom
import json
import _thread
import ubirch
from uuid import UUID
from network import WLAN, LTE
from machine import *
import network
import sys
from state_machine import *
from helpers import *
from error_handling import *
from config import *
from modem import get_imsi
from connection import get_connection, NB_IoT

from sensor import MovementSensor

from microWorkers import MicroWorkers
from time         import sleep

# # error color codes
# COLOR_INET_FAIL = LED_PURPLE_BRIGHT
# COLOR_BACKEND_FAIL = LED_ORANGE_BRIGHT
# COLOR_SIM_FAIL = LED_RED_BRIGHT
# COLOR_CONFIG_FAIL = LED_YELLOW_BRIGHT
# COLOR_MODEM_FAIL = LED_PINK_BRIGHT
# COLOR_UNKNOWN_FAIL = LED_WHITE_BRIGHT

# COMING_FROM_DEEPSLEEP = (machine.reset_cause() == machine.DEEPSLEEP_RESET)

# # mount SD card if there is one
# print("++ mounting SD")
# SD_CARD_MOUNTED = mount_sd()
# if SD_CARD_MOUNTED:
#     print("\tSD card mounted")
# else:
#     print("\tno SD card found")

# set up error handling
# max_file_size_kb = 10240 if SD_CARD_MOUNTED else 20
# error_handler = ErrorHandler(file_logging_enabled=True, max_file_size_kb=max_file_size_kb,
#                              sd_card=SD_CARD_MOUNTED)
print()

# bigger thread stack needed for the requests module used in UbirchDataClient (default: 4096)
_thread.stack_size(16384)


wlan = WLAN(mode=WLAN.STA)


class Main:
    def __init__(self) -> None:

        # self.breath = LedBreath()

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
            # self.ubirch_data.send(data)
        except Exception as e:
            print("ERROR      sending data to ubirch:", e)
            time.sleep(3)


    def read_loop(self):
        # get intervals
        # m_interval = self.cfg['measure_interval_s']
        # s_interval = self.cfg['send_interval_measurements']

        while True:

            self.root_controller.update()
            # self.breath.update()
            time.sleep(0.01)

main = Main()
main.read_loop()
