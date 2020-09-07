## from:
## https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code

import time
import machine as pycom_machine
import ubinascii
from config import *
from connection import get_connection, NB_IoT
from elevate_api import ElevateAPI
from error_handling import *
from helpers import *
from modem import get_imsi
from network import LTE
from pyboard import *
from realtimeclock import enable_time_sync, wait_for_sync
from sensor import MovementSensor
from sensor_config import *
from state_machine import *

LED_OFF = 0x000000

# standard brightness: 25% (low-power)
LED_WHITE = 0x202020  # StateConnecting
LED_GREEN = 0x002000  # StateMeasuringPaused
LED_YELLOW = 0x202000  # StateSendingVersionDiagnostics
LED_RED = 0x200000  # StateError
LED_PURPLE = 0x200020  # StateWaitingForOvershot
LED_BLUE = 0x000020  # StateInactive
LED_TURQUOISE = 0x002020  # StateSendingCellularDiagnostics

# full brightness (for errors etc)
LED_WHITE_BRIGHT = 0xffffff
LED_GREEN_BRIGHT = 0x00ff00
LED_YELLOW_BRIGHT = 0xffff00
LED_ORANGE_BRIGHT = 0xffa500
LED_RED_BRIGHT = 0xff0000
LED_PURPLE_BRIGHT = 0x800080
LED_BLUE_BRIGHT = 0x0000ff
LED_TURQUOISE_BRIGHT = 0x40E0D0
LED_PINK_BRIGHT = 0xFF1493

# error color codes
COLOR_INET_FAIL = LED_PURPLE_BRIGHT
COLOR_BACKEND_FAIL = LED_ORANGE_BRIGHT
COLOR_SIM_FAIL = LED_RED_BRIGHT
COLOR_CONFIG_FAIL = LED_YELLOW_BRIGHT
COLOR_MODEM_FAIL = LED_PINK_BRIGHT
COLOR_UNKNOWN_FAIL = LED_WHITE_BRIGHT

STANDARD_DURATION = 500


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
        self.uuid = {}
        self.sim = None
        self.key_name = ""
        self.api = None

        self.breath = LedBreath()

        self.latError = None
        # set all necessary time values
        self.IntervalForDetectingInactivityMs = 10000
        self.OvershotDetectionPauseIntervalMs = 10000
        self.FirstIntervalForInactivityEventMs = 60000
        self.ExponentialBackoffFactorForInactivityEvent = 2
        self.intervalForInactivityEventMs = self.FirstIntervalForInactivityEventMs
        self.intervalForErrorEventMs = 15000

        self.timerActivity = 0.0
        self.timerLastActivity = 0.0
        self.timerInactivity = 0.0

        print("\n\n\n\n\n[Core] Initializing magic... ✨ ")
        print("[Core] Hello, I am ", ubinascii.hexlify(pycom_machine.unique_id()))

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
        # load configuration, blocks in case of failure
        print("++ loading config")
        try:
            self.cfg = load_config(sd_card_mounted=SD_CARD_MOUNTED)

            lvl_debug = self.cfg['debug']  # set debug level
            if lvl_debug: print("\t" + repr(self.cfg))

            self.connection = get_connection(self.lte, self.cfg)  # initialize connection object depending on config
            self.api = ubirch.API(self.cfg)  # set up API for backend communication
        except Exception as e:
            print("\tERROR loading configuration")
            error_handler.log(e, COLOR_CONFIG_FAIL)
            while True:
                pycom_machine.idle()

        # create an instance of the elevate API, which needs the configuration
        self.elevate_api = ElevateAPI(self.cfg)

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
                pin = bootstrap(imsi, self.api)
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
            self.sim = ubirch.SimProtocol(lte=self.lte, at_debug=lvl_debug)
        except Exception as e:
            error_handler.log(e, COLOR_SIM_FAIL, reset=True)

        # unlock SIM
        try:
            self.sim.sim_auth(pin)
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
        self.key_name = "ukey"
        self.uuid = self.sim.get_uuid(self.key_name)
        print("UUID: " + str(self.uuid))

    def speed(self):
        max_speed = 0.0
        i = 0
        while i < 3:
            if self.sensor.speed_max[i] > max_speed:
                max_speed = self.sensor.speed_max[i]
            if math.fabs(self.sensor.speed_min[i]) > max_speed:
                max_speed = math.fabs(self.sensor.speed_min[i])
            i += 1
        return max_speed

    def add_state(self, state):
        self.states[state.name] = state

    def go_to_state(self, state_name):
        if self.state:
            print('> > Exiting {} A:{} L:{} I:{}'.format(self.state.name, self.timerActivity, self.timerLastActivity,
                                                         self.timerInactivity))
            self.state.exit(self)
        self.state = self.states[state_name]
        print('> > Entering {} A:{} L:{} I:{}'.format(self.state.name, self.timerActivity, self.timerLastActivity,
                                                      self.timerInactivity))
        self.state.enter(self)

    def update(self):
        if self.state:
            # print('Updating %s' % (self.state.name))
            self.state.update(self)

    # When pausing, don't exit the state
    def pause(self):
        self.state = self.states['paused']
        print('> > Pausing')
        self.state.enter(self)

    # When resuming, don't re-enter the state
    def resume_state(self, state_name):
        if self.state:
            print('> > Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        print('> > Resuming %s' % (self.state.name))

    def reset_state_machine(self):
        """As indicated, reset the machines system's variables."""
        print('> > Resetting the machine')


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


# Connecting State to connect to network
class StateConnecting(State):

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'connecting'

    def enter(self, machine):
        State.enter(self, machine)
        machine.breath.set_color(LED_WHITE)

    def exit(self, machine):
        State.exit(self, machine)

    def _connect(self, machine):
        # connect to network
        try:
            machine.connection.connect()
            enable_time_sync()
            print("\twaiting for time sync")
            wait_for_sync(print_dots=False)
        except Exception as e:
            error_handler.log(e, COLOR_INET_FAIL, reset=True)  # todo check reset
            return False

        return True

    def update(self, machine):
        State.update(self, machine)
        if self._connect(machine):
            machine.go_to_state('sendingVersionDiagnostics')


# Sending Version Diagnostics State
# TODO figure out, what to do here
class StateSendingVersionDiagnostics(State):

    def __init__(self):
        super().__init__()
        self._version_wait_time = 0

    @property
    def name(self):
        return 'sendingVersionDiagnostics'

    def enter(self, machine):
        State.enter(self, machine)
        # TODO Diagnostics::sendVersionDiagnostics()
        self._version_wait_time = time.ticks_ms()
        machine.breath.set_color(LED_YELLOW)

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self._version_wait_time + STANDARD_DURATION:
                machine.go_to_state('sendingCellularDiagnostics')


# Sending Cellular Diagnostics State
# todo figure out, how to get cellular diagnostics and transmit them
class StateSendingCellularDiagnostics(State):

    def __init__(self):
        super().__init__()
        self._cellular_wait_time = 0

    @property
    def name(self):
        return 'sendingCellularDiagnostics'

    def enter(self, machine):
        State.enter(self, machine)
        # TODO 	Diagnostics::sendCellularDiagnostics();   rssi -> AT+CSQ
        # 		Diagnostics::sendNetworkRegistrationStatus();   AT+COPS?
        self._cellular_wait_time = time.ticks_ms()
        machine.breath.set_color(LED_TURQUOISE)

    def exit(self, machine):
        State.exit(self, machine)
        machine.shower_count = 0

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self._cellular_wait_time + STANDARD_DURATION:
                machine.go_to_state('waitingForOvershoot')


# wait here for acceleration to overshoot the threshold
class StateWaitingForOvershot(State):

    def __init__(self):
        super().__init__()
        self._waiting_time = 0

    @property
    def name(self):
        return 'waitingForOvershoot'

    def enter(self, machine):
        State.enter(self, machine)
        self._waiting_time = time.ticks_ms()
        machine.breath.set_color(LED_PURPLE)

    def exit(self, machine):
        State.exit(self, machine)
        machine.timerInactivity = time.time()

    def update(self, machine):
        if State.update(self, machine):
            if machine.sensor.trigger:
                machine.sensor.calc_speed()
                if machine.speed() > g_THRESHOLD:
                    machine.go_to_state('measuringPaused')

            now = time.ticks_ms()
            if now >= self._waiting_time + machine.IntervalForDetectingInactivityMs:
                machine.go_to_state('inactive')


class StateMeasuringPaused(State):
    """
    this state is entered, when the threshold of activity was reached.
    Here the activity is transmitted to the backends
    """

    def __init__(self):
        super().__init__()
        self._measuring_paused_time = 0

    @property
    def name(self):
        return 'measuringPaused'

    # send the event to data service and upp to niomon

    def enter(self, machine):
        State.enter(self, machine)
        self._measuring_paused_time = time.ticks_ms()
        machine.timerActivity = time.time()
        machine.breath.set_color(LED_GREEN)
        machine.intervalForInactivityEventMs = machine.FirstIntervalForInactivityEventMs
        print("SPEED:", machine.sensor.speed_max, machine.sensor.speed_min)

        _send_event(machine, "speed", machine.speed(), machine.timerActivity, machine.timerLastActivity)

        # reset the speed values for next time
        machine.sensor.speed_max[0] = 0.0
        machine.sensor.speed_max[1] = 0.0
        machine.sensor.speed_max[2] = 0.0
        machine.sensor.speed_min[0] = 0.0
        machine.sensor.speed_min[1] = 0.0
        machine.sensor.speed_min[2] = 0.0

    def exit(self, machine):
        State.exit(self, machine)
        # save the time of this activity for the next time as last activity
        machine.timerLastActivity = machine.timerActivity

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self._measuring_paused_time + machine.OvershotDetectionPauseIntervalMs:
                machine.go_to_state('waitingForOvershoot')


class StateInactive(State):

    def __init__(self):
        super().__init__()
        self._inactive_time = 0

    @property
    def name(self):
        return 'inactive'

    def enter(self, machine):
        State.enter(self, machine)
        self._inactive_time = time.ticks_ms()
        machine.breath.set_color(LED_BLUE)
        _send_event(machine, "inactiveS", machine.timerInactivity - machine.timerLastActivity, machine.timerActivity,
                    machine.timerInactivity)
        machine.intervalForInactivityEventMs *= machine.ExponentialBackoffFactorForInactivityEvent
        print("[Core] Increased interval for inactivity events to {}".format(machine.intervalForInactivityEventMs))

    def exit(self, machine):
        State.exit(self, machine)
        machine.timerInactivity = time.time()

    def update(self, machine):
        if State.update(self, machine):
            if machine.sensor.trigger:
                machine.sensor.calc_speed()
                if machine.speed() > g_THRESHOLD:
                    machine.go_to_state('measuringPaused')

            now = time.ticks_ms()
            if now >= self._inactive_time + machine.intervalForInactivityEventMs:
                machine.go_to_state('inactive')  # todo check if this can be simplified


class StateError(State):

    def __init__(self):
        super().__init__()
        self._error_time = 0

    @property
    def name(self):
        return 'error'

    def enter(self, machine):
        State.enter(self, machine)
        self._error_time = time.ticks_ms()
        machine.breath.set_color(LED_RED)
        machine.intervalForErrorEventMs *= 2
        if machine.lastError:
            print("[Core] Last error: {}".format(machine.lastError))
        else:
            print("[Core] Unknown error.")

    def exit(self, machine):
        State.exit(self, machine)
        machine.lastError = None

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self._error_time + machine.intervalForErrorEventMs:
                machine.go_to_state('sendingCellularDiagnostics')


################################################################################

# Send the data away
def _send_event(machine, event_name: str, event_value, current_time: float, last_time: float):
    print("SENDING")
    # current_time = time.time()
    # pack the message
    event_data = {
        'msg_type': 1,
        'uuid': str(machine.uuid),
        'timestamp': int(current_time),
        'data': {
            'x_max': machine.sensor.speed_max[0],
            'y_max': machine.sensor.speed_max[1],
            'z_max': machine.sensor.speed_max[2],
            'x_min': machine.sensor.speed_min[0],
            'y_min': machine.sensor.speed_min[1],
            'z_min': machine.sensor.speed_min[2]
        }
    }

    serialized = serialize_json(event_data)
    print("SERIALIZED:", serialized)
    # seal the data message (data message will be hashed and inserted into UPP as payload by SIM card)
    try:
        print("++ creating UPP")
        upp = machine.sim.message_chained(machine.key_name, serialized, hash_before_sign=True)
        print("\tUPP: {}\n".format(ubinascii.hexlify(upp).decode()))
    except Exception as e:
        error_handler.log(e, COLOR_SIM_FAIL, reset=True)  # todo check the reset

    # get the time
    t = time.gmtime(current_time)  # (1970, 1, 1, 0, 0, 15, 3, 1)
    iso8601_fmt = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z"  # "2020-08-24T14:23:50Z"
    iso8601_time = iso8601_fmt.format(t[0], t[1], t[2], t[3], t[4], t[5])

    # make the elevate data package
    elevate_data = {
        'equipmentToken': machine.cfg['equipmentToken'],
        'sourceId': machine.cfg['sourceId'],
        'equipmentInfoId': machine.cfg['equipmentInfoId'],
        'isWorking': True,
        'customData': {
            'deviceId': str(machine.uuid),
            'event': {
                'name': event_name,
                'value': event_value,
                'productUserId': 'testSensor1',
                'productVersion': '0.9'
            }
        },
        'timeSinceLastActivityInSeconds': int(current_time - last_time),
        'isoDate': iso8601_time
    }
    # elevate_serialized = serialize_json(elevate_data) # todo, the serialization is not working yet
    print("ELEVATE RAW:", json.dumps(elevate_data))
    # print("ELEVATE SER:", elevate_serialized)

    machine.connection.connect()

    try:
        # send data message to data service, with reconnects/modem resets if necessary
        print("++ sending elevate")
        try:
            status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                     machine.elevate_api.send_data, machine.uuid,
                                                     json.dumps(elevate_data))
            print("RESPONSE:", content)
        except Exception as e:
            error_handler.log(e, COLOR_MODEM_FAIL, reset=True)  # todo check the reset
        # send data message to data service, with reconnects/modem resets if necessary
        print("++ sending data")
        try:
            status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                     machine.api.send_data, machine.uuid, serialized)
        except Exception as e:
            error_handler.log(e, COLOR_MODEM_FAIL, reset=True)  # todo check the reset

        # communication worked in general, now check server response
        if not 200 <= status_code < 300:
            raise Exception("backend (data) returned error: ({}) {}".format(status_code, str(content)))

        # send UPP to the ubirch authentication service to be anchored to the blockchain
        print("++ sending UPP")
        try:
            status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                     machine.api.send_upp, machine.uuid, upp)
        except Exception as e:
            error_handler.log(e, COLOR_MODEM_FAIL, reset=True)  # todo check the reset

        # communication worked in general, now check server response
        if not 200 <= status_code < 300:
            raise Exception("backend (UPP) returned error: ({}) {}".format(status_code, str(content)))

    except Exception as e:
        error_handler.log(e, COLOR_BACKEND_FAIL)


# TODO: do this anyway
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
