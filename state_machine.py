"""
from: https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code
"""
import machine as pycom_machine
import ubinascii
import ujson as json
import uos as os
from network import LTE

import logging
import ubirch
import system
from lib.config import *
from lib.connection import get_connection, NB_IoT
from lib.elevate_api import ElevateAPI
from lib.elevate_sim import ElevateSim
from lib.helpers import *
from lib.modem import get_imsi
from lib.realtimeclock import enable_time_sync, wait_for_sync, board_time_valid, NTP_SERVER_BACKUP
from sensor import MovementSensor
from sensor_config import *

# backlog constants
EVENT_BACKLOG_FILE = "event_backlog.txt"
UPP_BACKLOG_FILE = "upp_backlog.txt"
BACKLOG_MAX_LEN = 10  # max number of events / UPPs in the backlogs

VERSION_FILE = "OTA_VERSION.txt"

# timing
STANDARD_DURATION_S = 1
BLINKING_DURATION_S = 60
WAIT_FOR_TUNING_S = 30

WATCHDOG_TIMEOUT_MS = 6 * 60 * 1000

MAX_INACTIVITY_TIME_S = 60 * 60  # min * sec
FIRST_INTERVAL_INACTIVITY_S = MAX_INACTIVITY_TIME_S / 16  # =225 sec
EXP_BACKOFF_INACTIVITY = 2
OVERSHOOT_DETECTION_PAUSE_S = 60  # sec

RESTART_OFFSET_TIME_S = 24 * 60 * 60 + (int.from_bytes(os.urandom(2), "big") % 0x0FFF) # define restart time = 1 day + (0 .. 4095) seconds
print("RESTART IN: {} s".format(RESTART_OFFSET_TIME_S))

log = logging.getLogger()
SENSOR_CYCLE_COUNTER = 10


################################################################################
# State Machine

class StateMachine(object):

    def __init__(self):
        self.wdt = pycom_machine.WDT(timeout=WATCHDOG_TIMEOUT_MS)
        self.wdt.feed()
        self.state = None
        self.states = {}
        self.lastError = None
        self.failedBackendCommunications = 0
        self.timeStateLog = []
        self.sensorOverThresholdFlag = False
        self.system = None

        # set all necessary time values
        self.intervalForInactivityEventS = FIRST_INTERVAL_INACTIVITY_S

        self.startTime = 0

        log.info("\033[0;35m[Core] Initializing magic... \033[0m ✨ ")
        log.info("[Core] Hello, I am %s", ubinascii.hexlify(pycom_machine.unique_id()))

    def add_state(self, state):
        """
        Add a new state to the state machine
        :param state: new state to add
        """
        if not state.name in self.states: # check if state already exists
            self.states[state.name] = state
        else:
            log.error("cannot add state :({}), it already exists".format(state.name))

    def go_to_state(self, state_name):
        """
        Go to the state, which is indicated in the state_name
        :param state_name: new state to go to.
        """
        if not state_name in self.states: # check if state already exists
            log.error("cannot go to unknown state: ({})".format(state_name))
            state_name = 'error' # go to error state instead
        if self.state:
            self.state.exit(self)
        self.state = self.states[state_name]
        self.state.enter(self)

    def update(self):
        """
        Run update function of the current state.
        """
        if self.state:
            # print('Updating %s' % (self.state.name))
            try:
                self.state.update(self)
            except Exception as e:
                log.exception('Uncaught exception while processing state %s: %s', self.state, str(e))
                self.go_to_state('error')
        # CHECK: if there is no current state set (e.g. faulty update() function in some state),
        # this silently returns without errors. Throwing an exception here might be better.


################################################################################
# States

class State(object):
    """
    Abstract Parent State Class.
    """
    def __init__(self):
        self.enter_timestamp = 0
        pass

    @property
    def name(self):
        """
        Name of state for state interaction.
        This is an abstract property, which has to be implemented in every child class
        :return name string of the state.
        """
        raise NotImplementedError()

    def enter(self, machine):
        """
        Enter a specific state. This is called, when a new state is entered.
        Get the timestamp for entering, so it can be used in all states
        :param machine: state machine, which has the state
        """
        try:
            log.debug('Entering {}'.format(self.name))
            self.enter_timestamp = time.time()
            # add the timestamp and state name to a log, for later sending
            machine.timeStateLog.append(formated_time() + ":" + self.name)
            self._enter(machine)
        except Exception as e:
            log.exception("Enter: {}".format(str(e)))
            raise e

    def _enter(self, machine):
        """
        Enter a specific state
        This is an abstract method, which has to be implemented in every child class.
        :param machine: state machine, which has the state
        """
        raise NotImplementedError()

    def exit(self, machine):
        """
        Exit a specific state. This is called, when the old state is left.
        :param machine: state machine, which has the state.
        """
        try:
            log.debug('Exiting {}'.format(self.name))
            self._exit(machine)
        except Exception as e:
            log.exception("Exit: {}".format(str(e)))
            raise e

    def _exit(self, machine):
        """
        Enter a specific state
        This is an abstract method, which has to be implemented in every child class.
        :param machine: state machine, which has the state
        """
        raise NotImplementedError()

    def update(self, machine):
        """
        Update the current state, which means to run through the update routine.
        Breath with the LED. and calculate the speed from the measured accelerometer values.
        :param machine: state machine, which has the state.
        :return: True, to indicate, the function was called.
        """
        try:
            movement = False

            # check if the system is already initialised - skip if it is not
            if machine.system != None:
                machine.system.led_breath.update()

                # CHECK: this is always called, regardless of if the state actually needs to know about movement
                # should probably be located somewhere else
                if machine.system.sensor.trigger:
                    # CHECK: there is no real guarantee how often update is executed for each state (blocking operations), since other states
                    # depend on the values updated by calc_speed I am unsure if it works as expected when called from here
                    # if this needs to run regularly regardless of state, machine Timer module or simnilar callbacks/interrupts might be a solution
                    machine.system.sensor.calc_speed()
                    if machine.system.sensor.movement():
                        movement = True

            self._update(machine, movement)
        except Exception as e:
            log.exception("Update: {}".format(str(e)))
            raise e

    def _update(self, machine, movement):
        """
        Update a specific state.
        This is an abstract method, which has to be implemented in every child class.
        :param machine: state machine, which has the state
        :param movement: boolean telling, if the elevator has moved
        """
        raise NotImplementedError()


class StateInitSystem(State):
    """
    Initialize the System.
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'initSystem'

    def _enter(self, machine):
        # TODO breath not possible here since the system class is not yet initialized
        #machine.system.led_breath.set_color(LED_TURQUOISE)
        return

    def _exit(self, machine):
        pass

    def _update(self, machine, movement):
        try:
            machine.system = system.System()
        except OSError as e:
            log.exception(str(e))
            machine.lastError = str(e)

            pycom_machine.reset()

        now = time.time()
        if now >= self.enter_timestamp + STANDARD_DURATION_S:
            machine.go_to_state('connecting')


class StateConnecting(State):
    """
    Connecting State to connect to network.
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'connecting'

    def _enter(self, machine):
        machine.system.led_breath.set_color(LED_WHITE)

    def _exit(self, machine):
        pass

    def _connect(self, machine):
        """
        Connect to the given network
        :param machine: state machine, which holds this state
        :return: True, if connection established, otherwise false
        """
        try:
            machine.system.connection.ensure_connection()
            enable_time_sync()
            log.info("\twaiting for time sync")
            wait_for_sync(print_dots=True)
            # print("RTC SYC = {}".format(time.time()))
            if not board_time_valid():
                enable_time_sync(server=NTP_SERVER_BACKUP)
                log.info("\twaiting for backup time sync")
                wait_for_sync(print_dots=True)
                if not board_time_valid():
                    raise Exception("Time sync failed", time.time())
            # update the start time
            machine.startTime = time.time()

        except Exception as e:
            machine.lastError = str(e)
            return False

        finally:
            machine.system.connection.disconnect()

        return True

    def _update(self, machine, movement):
        if self._connect(machine):
            machine.go_to_state('sendingDiagnostics')
        else:
            machine.go_to_state('error')


class StateSendingDiagnostics(State):
    """
    Sending Version Diagnostics to the backend.
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'sendingDiagnostics'

    def _enter(self, machine):
        machine.system.led_breath.set_color(LED_YELLOW)

    def _exit(self, machine):
        pass

    def _update(self, machine, movement):
        global RESET_REASON
        # check the errors in the log and send it
        last_log = self._read_log(2)
        if not last_log == "":
            print("LOG: {}".format(last_log))
            event = ({'properties.variables.lastLogContent': {'value': last_log}})
            machine.system.send_event(event)  # CHECK: This might raise an exception which will not be caught

        # get the firmware version from OTA
        version = self._get_current_version()

        # get the signal quality and network status
        rssi, ber = machine.system.sim.get_signal_quality(machine.system.debug)
        cops = machine.system.sim.get_network_stats(machine.system.debug)  # TODO this is not yet decyphered
        event = ({'properties.variables.cellSignalPower': {'value': rssi},
                  'properties.variables.cellSignalQuality': {'value': ber},
                  'properties.variables.cellTechnology': {'value': cops},
                  'properties.variables.hardwareVersion': {'value': '0.9.0'},
                  'properties.variables.firmwareVersion': {'value': version},
                  'properties.variables.resetCause': {'value': RESET_REASON, "sentAt": formated_time()}})

        machine.system.send_event(event)  # CHECK: This might raise an exception which will not be caught

        now = time.time()
        if now >= self.enter_timestamp + STANDARD_DURATION_S:
            machine.go_to_state('waitingForOvershoot')

    def _get_current_version(self):
        try:
            from OTA_VERSION import VERSION
        except ImportError:
            VERSION = '1.0.0'
        return VERSION

    def _read_log(self, num_errors: int = 3):
        """
        Read the last ERRORs from log and form a string of json like list.
        :param num_errors: number of errors to return
        :return: String of the last errors
        :example: {'t':'1970-01-01T00:00:23Z','l':'ERROR','m':...}
        """
        last_log = ""
        error_counter = 0
        file_index = 1
        filename = logging.FILENAME
        # make a list of all log files
        all_logfiles_list = []
        if filename in os.listdir():
            all_logfiles_list.append(filename)
        while (filename + '.{}'.format(file_index)) in os.listdir():
            all_logfiles_list.append(filename + '.{}'.format(file_index))
            file_index += 1

        # iterate over all log files to get the required ERROR messages
        for logfile in all_logfiles_list:
            with open(logfile, 'r') as reader:
                lines = reader.readlines()
            for line in reversed(lines):
                # only take the error messages from the log
                if "ERROR" in line[:42]:  # only look at the beginning of the line, otherwise the string can appear recursively
                    error_counter += 1
                    if error_counter > num_errors:
                        break
                    last_log += (line.rstrip('\n'))
                    # check if the message was closed with "}", if not, add it to ensure json
                    if not "}" in line:
                        last_log += "},"
                    else:
                        last_log += ","
                else:
                    pass
            if error_counter > num_errors:
                break
        return last_log.rstrip(',')


class StateWaitingForOvershoot(State):
    """
    Wait here for acceleration to overshoot the threshold,
    or until waiting time was exceeded.
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'waitingForOvershoot'

    def _enter(self, machine):
        machine.system.led_breath.set_color(LED_PURPLE)

    def _exit(self, machine):
        pass

    def _update(self, machine, movement):
        # wait 30 seconds for filter to tune in
        now = time.time()
        if now >= self.enter_timestamp + WAIT_FOR_TUNING_S:
            if movement:
                machine.go_to_state('measuringPaused')
                return

        if now >= self.enter_timestamp + machine.intervalForInactivityEventS:
            machine.go_to_state('inactive')
            return


class StateMeasuringPaused(State):
    """
    this state is entered, when the threshold of activity was reached.
    Here the activity is transmitted to the backends
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'measuringPaused'

    def _enter(self, machine):
        machine.system.led_breath.set_color(LED_GREEN)

    def _exit(self, machine):
        pass

    def _update(self, machine, movement):
        machine.intervalForInactivityEventS = FIRST_INTERVAL_INACTIVITY_S
        machine.system.sensor.poll_sensors()
        event = ({
            'properties.variables.isWorking': {'value': True, 'sentAt': formated_time()},
            'properties.variables.acceleration': {
                'value': 1 if machine.system.sensor.speed_max > abs(machine.system.sensor.speed_min) else -1},
            'properties.variables.accelerationMax': {'value': machine.system.sensor.speed_max},
            'properties.variables.accelerationMin': {'value': machine.system.sensor.speed_min},
            'properties.variables.altitude': {'value': machine.system.sensor.altitude},
            'properties.variables.temperature': {'value': machine.system.sensor.temperature}
        })
        machine.system.send_event(event, ubirching=True)  # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())
        # now send the state log also
        last_log = concat_state_log(machine)
        if not last_log == "":
            event = ({'properties.variables.lastLogContent': {'value': last_log}})
            machine.system.send_event(event)  # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())

        now = time.time()
        if now >= self.enter_timestamp + STANDARD_DURATION_S:
            machine.go_to_state('waitingForOvershoot')


class StateInactive(State):
    """
    Inactive State,
    at entering, the current state is get from the backend
    and the waiting interval is increased
    """
    def __init__(self):
        super().__init__()
        self.new_log_level = ""
        self.new_state = ""

    @property
    def name(self):
        return 'inactive'

    def _enter(self, machine):
        machine.system.led_breath.set_color(LED_BLUE)

        if machine.intervalForInactivityEventS < MAX_INACTIVITY_TIME_S:
            machine.intervalForInactivityEventS *= EXP_BACKOFF_INACTIVITY
        else:
            machine.intervalForInactivityEventS = MAX_INACTIVITY_TIME_S

        machine.system.sensor.poll_sensors()
        event = ({
            'properties.variables.altitude': {'value': machine.system.sensor.altitude},
            'properties.variables.temperature': {'value': machine.system.sensor.temperature}
        })
        last_log = concat_state_log(machine)
        if not last_log == "":
            event.update({'properties.variables.lastLogContent': {'value': last_log}})

        machine.system.send_event(event)  # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())

        self.new_log_level, self.new_state = machine.get_state_from_backend() # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())
        log.info("New log level: ({}), new backend state:({})".format(self.new_log_level, self.new_state))
        log.debug("Increased interval for inactivity events to {}".format(machine.intervalForInactivityEventS))

    def _exit(self, machine):
        pass

    def _update(self, machine, movement):
        # wait for filter to tune in
        now = time.time()
        if now >= machine.startTime + RESTART_OFFSET_TIME_S:
            log.info("its time to restart")
            machine.go_to_state('bootloader')
            return

        if now >= self.enter_timestamp + WAIT_FOR_TUNING_S:
            if movement:
                machine.go_to_state('measuringPaused')
                return

        if now >= self.enter_timestamp + machine.intervalForInactivityEventS:
            machine.go_to_state('inactive')
            return

        self._adjust_level_state(machine, self.new_log_level, self.new_state)

    def _adjust_level_state(self, machine, level, state):
        """
        Adjust the logging level and the current state
        :param machine: state machine holding this state
        :param level: new logging level to adjust
        :param state: new sensor state to adjust
        """
        log.setLevel(_log_switcher(level))
        machine.go_to_state(_state_switcher(state))


class StateBlinking(State):
    """
    Blinking State is a special state, with a bright and fast led blinking
    to make the sensor more visible. This state is intended to identify
    sensors in the field.
    After some time, the state switches back to inactive.
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'blinking'

    def _enter(self, machine):
        machine.system.led_breath.set_blinking()

    def _exit(self, machine):
        machine.system.led_breath.reset_blinking()

    def _update(self, machine, movement):
        now = time.time()
        if now >= self.enter_timestamp + BLINKING_DURATION_S:
            machine.go_to_state('inactive')  # this is neccessary to fetch a new state from backend


class StateError(State):
    """
    Error State, which is entered, whenever a critical error occurs.
    This state should end with a reset of the complete system
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'error'

    def _enter(self, machine):
        if machine.system != None and machine.system.led_breath != None:
            machine.system.led_breath.set_color(LED_RED)

    def _exit(self, machine):
        raise SystemError("exiting the error state should never happen here")

    def _update(self, machine, movement):
        try:  # just build that in, because of recent error, which caused the controller to be BRICKED TODO: check this again
            if machine.lastError:
                log.error("Last error: {}".format(machine.lastError))

            # try to send the error message
            event = ({
                'properties.variables.lastError': {
                    'value': machine.lastError if machine.lastError is not None else "unknown",
                    'sentAt': formated_time()}
            })
            machine.system.send_emergency_event(event)

            machine.system.sim.deinit()
            machine.system.lte.deinit(detach=True, reset=True)

        finally:
            time.sleep(3)
            pycom_machine.deepsleep(1)  # this will wakeup with reset in the main again


class StateBootloader(State):
    """
    Bootloader State, which is entered, when the bootloader should be entered.
    This state has the purpose to shutdown the system and reset the machine
    to try an OTA (Over The Air Update)
    """
    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'bootloader'

    def _enter(self, machine):
        machine.system.breath.set_color(LED_WHITE_BRIGHT)

    def _exit(self, machine):
        raise SystemError("exiting the bootloader state should never happen here")

    def _update(self, machine, movement):
        try:  # just build tha in, because of recent error, which caused the controller to hang
            machine.system.sim.deinit()

        finally:
            time.sleep(1)
            pycom_machine.reset()


################################################################################
#         'isWorking'#(Bool) DONE
#         'lastActivityOn' #(date) todo
#         'lastLogContent' #(String) DONE for ERRORs
#         'firmwareVersion'# (String) DONE
#         'hardwareVersion'# (String) todo currently fixed string
#         'cellSignalPower'# (Number) DONE
#         'cellSignalQuality'# (Number) DONE
#         'cellOperator'# (String) DONE by backend
#         'cellTechnology'# (String) DONE by backend
#         'cellGlobalIdentity'# (String) todo figure out
#         'cellDeviceIdentity'# (String) todo figure out
#         'ping'# (Number) todo
#         'totalAvailableMemory'# (Number) todo
#         'usedMemory'# (Number) todo
#         'totalAvailableStorage'# (Number) todo
#         'usedStorage'# (Number) todo
#         'lastError' # (string) DONE in case of emergency
#         'resetCause' # (string) DONE

def _log_switcher(log_level: str):
    """
    Logging level switcher for handling different logging levels from backend.
    :param log_level: ne logging level from backend
    :return: translated logging level for logger
    """
    switcher = {
        'error': logging.ERROR,
        'warning': logging.WARNING,
        'info': logging.INFO,
        'debug': logging.DEBUG
    }
    return switcher.get(log_level, logging.INFO)


def _state_switcher(state: str):
    """
    State recognition switcher for handling state changes from backend.
    :param state: new state given from the backend
    :return: translated state for state_machine
    """
    # CHECK: TODO: document what the backend might reply, e.g. especially if "empty" state information is possible
    switcher = {
        'installation': 'waitingForOvershoot',
        'blinking': 'blinking',
        'sensing': 'waitingForOvershoot',
        'custom1': 'waitingForOvershoot',
        'custom2': 'waitingForOvershoot',
        'custom3': 'bootloader'
    }
    return switcher.get(state, 'error')


def _reset_cause_switcher(reset_cause: int):
    """
    Reset cause switcher for translating the reset cause into readable string.
    :param state: new state given from the backend
    :return: translated state for state_machine
    """
    switcher = {
        machine.PWRON_RESET: 'Power On',
        machine.HARD_RESET: 'Hard',
        machine.WDT_RESET: 'Watchdog',
        machine.DEEPSLEEP_RESET: 'Deepsleep',
        machine.SOFT_RESET: 'Soft',
        machine.BROWN_OUT_RESET: 'Brown Out'
    }
    return switcher.get(reset_cause, 'Unknown')


# todo find a good place for these functions
RESET_REASON = _reset_cause_switcher(pycom_machine.reset_cause())
