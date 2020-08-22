## from:
## https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code

import time
import machine as pycom_machine
import ubinascii
from pyboard import *

from sensor import MovementSensor
from sensor_config import *
import pycom
from network import WLAN, LTE
import network
import sys
from state_machine import *
from helpers import *
from error_handling import *
from config import *
from modem import get_imsi
from connection import get_connection, NB_IoT

from helpers import LedBreath


LED_OFF = 0x000000

#standard brightness: 12.5% (low-power)
LED_WHITE     = 0x202020
LED_GREEN     = 0x002000
LED_YELLOW    = 0x202000
LED_RED       = 0x200000
LED_PURPLE    = 0x200020
LED_BLUE      = 0x000020
LED_TURQUOISE = 0x002020

#full brightness (for errors etc)
LED_WHITE_BRIGHT     = 0xffffff
LED_GREEN_BRIGHT     = 0x00ff00
LED_YELLOW_BRIGHT    = 0xffff00
LED_ORANGE_BRIGHT    = 0xffa500
LED_RED_BRIGHT       = 0xff0000
LED_PURPLE_BRIGHT    = 0x800080
LED_BLUE_BRIGHT      = 0x0000ff
LED_TURQUOISE_BRIGHT = 0x40E0D0
LED_PINK_BRIGHT      = 0xFF1493

# error color codes
COLOR_INET_FAIL = LED_PURPLE_BRIGHT
COLOR_BACKEND_FAIL = LED_ORANGE_BRIGHT
COLOR_SIM_FAIL = LED_RED_BRIGHT
COLOR_CONFIG_FAIL = LED_YELLOW_BRIGHT
COLOR_MODEM_FAIL = LED_PINK_BRIGHT
COLOR_UNKNOWN_FAIL = LED_WHITE_BRIGHT


DROP_DURATION = 10
################################################################################
# State Machine

class StateMachine(object):

    def __init__(self):
        self.state = None
        self.states = {}
        self.sensor = MovementSensor()
        self.lte = LTE()
        self.cfg = []
        self.connection = None

        self.breath = LedBreath()


        print("\n\n\n\n\n[Core] Initializing magic... ✨ ")
        print("[Core] Hello, I am " , ubinascii.hexlify(pycom_machine.unique_id()))

        try:
            # reset modem on any non-normal loop (modem might be in a strange state)
            if not COMING_FROM_DEEPSLEEP:
                print("++ not coming from sleep, resetting modem")
                reset_modem(self.lte)

            print("++ getting IMSI")
            imsi = get_imsi(self.lte)
            print("IMSI: " + imsi)
        except Exception as e:
            print("\tERROR setting up modem")
            error_handler.log(e, COLOR_MODEM_FAIL)
            while True:
                pycom_machine.idle()

        # write IMSI to SD card
        if not COMING_FROM_DEEPSLEEP and SD_CARD_MOUNTED: store_imsi(imsi)

        # set_led(LED_TURQUOISE)

        # load configuration, blocks in case of failure
        print("++ loading config")
        try:
            self.cfg = load_config(sd_card_mounted=SD_CARD_MOUNTED)

            lvl_debug = self.cfg['debug']  # set debug level
            if lvl_debug: print("\t" + repr(self.cfg))

            # sensors = get_pyboard(self.cfg['board'])  # initialise the sensors on the pyboard
            self.connection = get_connection(self.lte, self.cfg)  # initialize connection object depending on config
            api = ubirch.API(self.cfg)  # set up API for backend communication
        except Exception as e:
            print("\tERROR loading configuration")
            error_handler.log(e, COLOR_CONFIG_FAIL)
            while True:
                pycom_machine.idle()

        # configure connection timeouts according to config
        if isinstance(self.connection, NB_IoT):
            self.connection.setattachtimeout(self.cfg["nbiot_extended_attach_timeout"])
            self.connection.setconnecttimeout(self.cfg["nbiot_extended_connect_timeout"])

        # get PIN from flash, or bootstrap from backend and then save PIN to flash
        pin_file = imsi + ".bin"
        pin = get_pin_from_flash(pin_file, imsi)
        if pin is None:
            try:
                self.connection.connect()
            except Exception as e:
                error_handler.log(e, COLOR_INET_FAIL, reset=True)

            try:
                pin = bootstrap(imsi, api)
                with open(pin_file, "wb") as f:
                    f.write(pin.encode())
            except Exception as e:
                error_handler.log(e, COLOR_BACKEND_FAIL, reset=True)

        # disconnect from LTE connection before accessing SIM application
        # (this is only necessary if we are connected via LTE)
        if isinstance(self.connection, NB_IoT):
            print("\tdisconnecting")
            self.connection.disconnect()

        # initialise ubirch SIM protocol
        print("++ initializing ubirch SIM protocol")
        try:
            sim = ubirch.SimProtocol(lte=self.lte, at_debug=lvl_debug)
        except Exception as e:
            error_handler.log(e, COLOR_SIM_FAIL, reset=True)

        # unlock SIM
        try:
            sim.sim_auth(pin)
        except Exception as e:
            error_handler.log(e, COLOR_SIM_FAIL)
            # if PIN is invalid, there is nothing we can do -> block
            if isinstance(e, ValueError):
                print("PIN is invalid, can't continue")
                while True:
                    wdt.feed()  # avert reset from watchdog
                    self.breath.set_color(COLOR_SIM_FAIL)
            else:
                pycom_machine.reset()

        # get UUID from SIM
        key_name = "ukey"
        uuid = sim.get_uuid(key_name)
        print("UUID: " + str(uuid))


    def add_state(self, state):
        self.states[state.name] = state

    def go_to_state(self, state_name):
        if self.state:
            print('Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        print('Entering %s' % (self.state.name))
        self.state.enter(self)

    def update(self):
        if self.state:
            print('Updating %s' % (self.state.name))
            self.state.update(self)

    # When pausing, don't exit the state
    def pause(self):
        self.state = self.states['paused']
        print('Pausing')
        self.state.enter(self)

    # When resuming, don't re-enter the state
    def resume_state(self, state_name):
        if self.state:
            print('Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        print('Resuming %s' % (self.state.name))

    def reset_state_machine(self):
        """As indicated, reset the machines system's variables."""
        print('Resetting the machine')

################################################################################
# States


# Abstract parent state class.
class State(object):

    def __init__(self):
        pass

    @property
    def name(self):
        return ''

    def enter(self, machine):
        pass

    def exit(self, machine):
        pass

    def update(self, machine):
        machine.breath.update()
        # if switch.fell:
        #     machine.paused_state = machine.state.name
        #     machine.pause()
        #     return False
        return True


#Connecting State to connect to network
class ConnectingState(State):

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'connecting'

    def enter(self, machine):
        State.enter(self, machine)
        set_led(LED_TURQUOISE)

    def exit(self, machine):
        State.exit(self, machine)

    def _connect(self,machine):
        # connect to network
        if machine.connection.isconnected() :
            return True
        machine.connection.connect()
        if machine.connection.isconnected() :
            return True
        return False

    def update(self, machine):
        if self._connect(machine):
            machine.go_to_state('sendingVersionDiagnostics')

# Sending Version Diagnostics State
# TODO figure out, what to do here
class SendingVersionDiagnosticsState(State):

    def __init__(self):
        super().__init__()
        self.version_wait_time = 0

    @property
    def name(self):
        return 'sendingVersionDiagnostics'

    def enter(self, machine):
        State.enter(self, machine)
        now = time.ticks_ms()
        self.version_wait_time = now + DROP_DURATION
        set_led(LED_BLUE)

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.version_wait_time:
                machine.go_to_state('sendingCellularDiagnostics')


# Sending Cellular Diagnostics State
# todo figure out, how to get cellular diagnostics and transmit them
class SendingCellularDiagnosticsState(State):

    def __init__(self):
        super().__init__()
        self.cellular_wait_time = 0

    @property
    def name(self):
        return 'sendingCellularDiagnostics'

    def enter(self, machine):
        State.enter(self, machine)
        now = time.ticks_ms()
        self.cellular_wait_time = now + DROP_DURATION
        set_led(LED_PURPLE)

    def exit(self, machine):
        State.exit(self, machine)
        machine.shower_count = 0

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.cellular_wait_time:
                machine.go_to_state('waitingForOvershoot')


# wait here for acceleration to overshoot the threshold
class WaitingForOvershootState(State):

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'waitingForOvershoot'

    def enter(self, machine):
        State.enter(self, machine)
        machine.breath.set_color(LED_PURPLE)

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            if machine.sensor.trigger:
                speed_max, speed_min = machine.sensor.calc_speed()
                i = 0
                while i < 3:
                    if speed_max[i] > g_THRESHOLD:
                        machine.breath.set_color(LED_GREEN)
                        machine.go_to_state('measuringPaused')
                    if speed_min[i] < -g_THRESHOLD:
                        machine.breath.set_color(LED_RED)
                        machine.go_to_state('measuringPaused')
                    i += 1


# Send the data away
class MeasuringPausedState(State):

    def __init__(self):
        super().__init__()
        self.cellular_wait_time = 0

    @property
    def name(self):
        return 'measuringPaused'

    def enter(self, machine):
        State.enter(self, machine)
        now = time.ticks_ms()
        self.cellular_wait_time = now + 1000
        print(machine.sensor.speed_max, machine.sensor.speed_min)
        machine.sensor.speed_max[0] = 0.0
        machine.sensor.speed_max[1] = 0.0
        machine.sensor.speed_max[2] = 0.0
        machine.sensor.speed_min[0] = 0.0
        machine.sensor.speed_min[1] = 0.0
        machine.sensor.speed_min[2] = 0.0

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.cellular_wait_time:
                set_led(LED_OFF)
                machine.go_to_state('waitingForOvershoot')

# Reset the LEDs and audio, start the servo raising the ball
# When the switch is released, stop the ball and move to waiting

class ErrorState(State):

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'error'

    def enter(self, machine):
        State.enter(self, machine)
        strip.fill(0)
        strip.brightness = 1.0
        strip.show()
        if audio.playing:
            audio.stop()
        servo.throttle = RAISE_THROTTLE

    def exit(self, machine):
        State.exit(self, machine)
        servo.throttle = 0.0

    def update(self, machine):
        if State.update(self, machine):
            if switch.rose:
                machine.go_to_state('waiting')


################################################################################

def set_led(led_color):
    pass # pycom.heartbeat(False)  # disable blue heartbeat blink
    # pycom.rgbled(led_color)

def set_heartbeat():
    pass # pycom.heartbeat(True)


COMING_FROM_DEEPSLEEP = (pycom_machine.reset_cause() == pycom_machine.DEEPSLEEP_RESET)


# mount SD card if there is one
print("++ mounting SD")
SD_CARD_MOUNTED = mount_sd()
if SD_CARD_MOUNTED:
    print("\tSD card mounted")
else:
    print("\tno SD card found")

max_file_size_kb = 10240 if SD_CARD_MOUNTED else 20
error_handler = ErrorHandler(file_logging_enabled=True, max_file_size_kb=max_file_size_kb,
                             sd_card=SD_CARD_MOUNTED)
