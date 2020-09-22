import utime as time

import logging

from pyboard import *
import _thread
from network import WLAN
from state_machine import *
from helpers import *
from error_handling import *
from config import *

import machine

# set watchdog: if execution hangs/takes longer than 'timeout' an automatic reset is triggered
# we need to do this as early as possible in case an import cause a freeze for some reason
wdt = machine.WDT(timeout=5 * 60 * 1000)  # we set it to 5 minutes here and will reconfigure it when we have loaded the configuration
wdt.feed()  # we only feed it once since this code hopefully finishes with deepsleep (=no WDT) before reset_after_ms

print()

# bigger thread stack needed for the requests module used in UbirchDataClient (default: 4096)
_thread.stack_size(16384)

wlan = WLAN(mode=WLAN.STA)

# create a logging for the system and store the information in a file
FMT = "{\'t\':\'%(asctime)s\'," \
      "\'l\':\'%(levelname)s\'," \
      "\'m\': \'%(message)s\'}"
#       "\'n\':\'%(name)s\'," \
fileHandler = logging.FileHandler(filename="/flash/testlog2.txt")
fileHandler.setFormatter(logging.Formatter(fmt=FMT))
streamHandler = logging.StreamHandler(sys.stdout)
logging.basicConfig(level=logging.DEBUG,
                    format=FMT)
log = logging.getLogger()
log.addHandler(fileHandler)

status_code = 200
cntnt = "we love tno code"
content = "submitting CSR failed: ({}) {}".format(status_code, str(cntnt))
try:
    1/0
except:
    log.exception("backend (UPP) returned error: ({}) {}".format(status_code, str(content)))


class Main:
    def __init__(self) -> None:

        # disable blue heartbeat blink
        pycom.heartbeat(False)

        # create the root controller as a state machine and add all the necessary states
        self.root_controller = StateMachine()
        self.root_controller.add_state(StateConnecting())
        self.root_controller.add_state(StateSendingVersionDiagnostics())
        self.root_controller.add_state(StateSendingCellularDiagnostics())
        self.root_controller.add_state(StateWaitingForOvershot())
        self.root_controller.add_state(StateMeasuringPaused())
        self.root_controller.add_state(StateInactive())
        self.root_controller.add_state(StateBlinking())
        self.root_controller.add_state(StateError())
        # start with the connecting state
        self.root_controller.go_to_state('connecting')


    def read_loop(self):
        # get intervals
        # m_interval = self.cfg['measure_interval_s']
        # s_interval = self.cfg['send_interval_measurements']

        while True:
            self.root_controller.update()
            time.sleep(0.01)
            wdt.feed()

main = Main()
main.read_loop()
