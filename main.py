# TODO:
#  check all print statements
#  adopt README.md
import utime as time
import micropython
import sys

from logging.handlers import RotatingFileHandler

import pycom
from network import Server # WLAN,
from state_machine import *
from helpers import *
import gc

import machine
import logging

# set watchdog: if execution hangs/takes longer than 'timeout' an automatic reset is triggered
# we need to do this as early as possible in case an import cause a freeze for some reason
# wdt = machine.WDT(timeout=60 * 1000)  # we set it to 5 minutes here and will reconfigure it when we have loaded the configuration
# wdt.feed()  # we only feed it once since this code hopefully finishes with deepsleep (=no WDT) before reset_after_ms

# disable the FTP Server
server = Server()
server.deinit() # disable the server
# disable the wifi on boot
pycom.wifi_on_boot(False)

# allocate extra buffer for emergency exception
micropython.alloc_emergency_exception_buf(128)

# bigger thread stack needed for the requests module used in UbirchDataClient (default: 4096)
# _thread.stack_size(16384)

# enable the garbage collector
gc.enable()

# create a logging for the system and store the information in a file
FMT = "{\'t\':\'%(asctime)s\'," \
      "\'l\':\'%(levelname)s\'," \
      "\'m\': \'%(message)s\'}"
#       "\'n\':\'%(name)s\'," \
fileHandler = RotatingFileHandler(filename=logging.FILENAME, maxBytes=16384, backupCount=4)
fileHandler.setFormatter(logging.Formatter(fmt=FMT))

logging.basicConfig(level=logging.DEBUG,
                    format=FMT)
log = logging.getLogger()
log.addHandler(fileHandler)

log.warning("coming from reset")


class Main:
    def __init__(self) -> None:

        # disable blue heartbeat blink
        pycom.heartbeat(False)

        # create the root controller as a state machine and add all the necessary states
        self.root_controller = StateMachine()
        self.root_controller.add_state(StateInitSystem())
        self.root_controller.add_state(StateConnecting())
        self.root_controller.add_state(StateSendingDiagnostics())
        self.root_controller.add_state(StateWaitingForOvershoot())
        self.root_controller.add_state(StateMeasuringPaused())
        self.root_controller.add_state(StateInactive())
        self.root_controller.add_state(StateBlinking())
        self.root_controller.add_state(StateError())
        self.root_controller.add_state(StateBootloader())
        # start with the connecting state
        self.root_controller.go_to_state('initSystem')

    def read_loop(self):
        while True:
            try:
                self.root_controller.update()
                time.sleep_ms(33)
                self.root_controller.wdt.feed() # CHECK: This way, the watchdog will never trigger as long as update() returns without exception.
                                                # Might be worth thinking about only feeding watchdog if something meaningful is done. (I.e. in the states.)
                print("stack {} heap {}".format(micropython.stack_use(), micropython.mem_info()))

            except Exception as e:
                print("\r\n\n\n\033[1;31mMAIN ERROR CAUGHT:  {}\033[0m\r\n\n\n".format(repr(e)))
                try:
                    log.exception(str(e))
                finally:
                    pass
                time.sleep(10)

                # check if the system (-> pysense) is initialised and try to reset
                if self.root_controller.system != None and self.root_controller.system.sensor != None:
                    self.root_controller.system.hard_reset()
                else:
                    machine.reset()


main = Main()
main.read_loop()
