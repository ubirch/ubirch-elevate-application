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
STANDARD_DURATION_MS = 500
BLINKING_DURATION_MS = 60000
WAIT_FOR_TUNING_MS = 30000
WATCHDOG_TIMEOUT_MS = 6 * 60 * 1000

MAX_INACTIVITY_TIME_MS = 60 * 60 * 1000  # min * sec * msec
FIRST_INTERVAL_INACTIVITY_MS = MAX_INACTIVITY_TIME_MS / 16  # =225000 msec
EXP_BACKOFF_INACTIVITY = 2
OVERSHOOT_DETECTION_PAUSE_MS = 60 * 1000  # sec * msec

RESTART_OFFSET_TIME_MS = 24 * 60 * 60 * 1000 + (int.from_bytes(os.urandom(3), "big") % 0x3FFFFF)
print("RESTART IN: {}ms".format(RESTART_OFFSET_TIME_MS))

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
        self.cfg = []
        self.connection = None
        self.uuid = {}
        self.sim = None
        self.pin = None
        self.key_name = ""
        self.api = None
        self.debug = False
        self.lastError = None
        self.failedBackendCommunications = 0
        self.timeStateLog = []
        self.sensorOverThresholdFlag = False

        # create instances of required objects
        try:
            self.sensor = MovementSensor()
            self.lte = LTE()  # psm_period_value=1, psm_period_unit=LTE.PSM_PERIOD_1H,
            #  psm_active_value=5, psm_active_unit=LTE.PSM_ACTIVE_1M)
            self.breath = LedBreath()
        except OSError as e:
            log.exception(str(e))
            self.lastError = str(e)
            self.hard_reset()
            log.error("I should not get here") # CHECK: maybe be more specific with the message in case this line actually is triggered one day.
            # self.go_to_state('error')
            pycom_machine.reset()

        # set all necessary time values
        self.intervalForInactivityEventMs = FIRST_INTERVAL_INACTIVITY_MS

        self.startTime = 0 # CHECK: since the reference point is arbitrary per the micropython docs,  time.ticks_ms() might be safer

        log.info("\033[0;35m[Core] Initializing magic... \033[0m ✨ ")
        log.info("[Core] Hello, I am %s", ubinascii.hexlify(pycom_machine.unique_id()))

    def speed(self): # CHECK: why is this part of the state machine?
        """
        Calculate th maximum absolute speed value from current sensor values,
        over all axis.
        :return: the maximum value of the currently measured speed.
        """
        if self.sensor.speed_max > g_THRESHOLD:
            print("SPEED MAX= {}".format(self.sensor.speed_max))
            return True

        if abs(self.sensor.speed_min) > g_THRESHOLD:
            print("SPEED MIN= {}".format(self.sensor.speed_min))
            return True

        return False

    def add_state(self, state):
        """
        Add a new state to the state machine
        :param state: new state to add
        """
        self.states[state.name] = state # CHECK: this allows for overwriting of states (intended?), might need check for 'if not state.name in self.states:... else: error'

    def go_to_state(self, state_name):
        """
        Go to the state, which is indicated in the state_name
        :param state_name: new state to go to.
        """
        # CHECK: does this need a check if the state_name exists and/or an exception handler?
        # right now, an exception would propagate to the caller, which is probably also fine 
        if self.state:
            log.debug('Exiting {}'.format(self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        log.debug('Entering {}'.format(self.state.name))
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

    def hard_reset(self):
        """
        Hard-Reset GPy and Modem, by triggering power OFF/ON from Pysense.
        """
        time.sleep(1)
        self.sensor.pysense.reset_cmd()


############################

# CHECK: maybe add remark what these are for, as far as i can see this section is print helper functions, right?
def _formated_time():
    ct = time.localtime()
    return "{0:04d}-{1:02d}-{2:02d}T{3:02d}:{4:02d}:{5:02d}Z".format(*ct)  # modified to fit the correct format


def _concat_state_log(machine):
    state_log = ""
    for lines in machine.timeStateLog:
        state_log += lines + ","
    machine.timeStateLog.clear()
    return state_log


################################################################################
# States

class State(object):
    """
    Abstract Parent State Class.
    """

    def __init__(self):
        self.enter_timestamp = 0 # CHECK: since the reference point is arbitrary per the micropython docs,  time.ticks_ms() might be safer
        pass

    @property
    def name(self):
        """
        Name of state for state interaction.
        :return name string of the state.
        """
        return '' # CHECK: this must raise a "not implemented" error instead of returning something, as a proper state name is needed for the state machine class to work

    def enter(self, machine):
        """
        Enter a specific state. This is called, when a new state is entered.
        Get the timestamp for entering, so it can be used in all states
        :param machine: state machine, which has the state
        """
        self.enter_timestamp = time.ticks_ms()
        # add the timestamp and state name to a log, for later sending
        machine.timeStateLog.append(_formated_time() + ":" + self.name)
        pass
        # CHECK: if we want to ensure that all states are logged properly (avoiding copy paste errors in inheriting classes or forgetting State.enter()),
        # it might be better to split logging and entering in this base class:
        # def enter(...):
        #     log_enter(...)
        #     do_entry(...)
        # Only do_entry (probably not the best name, just an example...) is then implemented in inheriting specific state class
        # and proper logging always ensured via the base class implentation of enter (remember to raise not implemented in this base class for do_entry())

    def exit(self, machine):
        """
        Exit a specific state. This is called, when the old state is left.
        :param machine: state machine, which has the state.
        """
        pass

    def update(self, machine):
        """
        Update the current state, which means to run through the update routine.
        Breath with the LED. and calculate the speed from the measured accelerometer values.
        :param machine: state machine, which has the state.
        :return: True, to indicate, the function was called.
        """
        # CHECK: maybe for this, the same remark as for enter (forgetting to call State.update()) applies
        # i.e. update() could consist of breath_update() and do_update() or similar. (See enter())
        machine.breath.update()

        # CHECK: this is always called, regardless of if the state actually needs to know about movement
        # should probably be located somewhere else
        if machine.sensor.trigger:
            machine.sensor.calc_speed() # CHECK: there is no real guarantee how often update is executed for each state (blocking operations), since other states
                                        # depend on the values updated by calc_speed I am unsure if it works as expected when called from here
                                        # if this needs to run regularly regardless of state, machine Timer module or simnilar callbacks/interrupts might be a solution
            if machine.speed():
                return True
        return False


class StateInitSystem(State):
    """
    Initialize the System.
    """

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'initSystem'

    def enter(self, machine):
        State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        machine.breath.set_color(LED_TURQUOISE)

        try:
            # reset modem (modem might be in a strange state)
            log.warning("not coming from sleep, resetting modem") # CHECK: message seems wrong, there is no check for sleep here
            reset_modem(machine.lte)

            log.info("getting IMSI")
            imsi = get_imsi(machine.lte)
            log.info("IMSI: " + imsi)
        except Exception as e:
            log.exception("failed setting up modem {}".format(str(e)))
            while True:  # watchdog will get out of this
                pycom_machine.idle() # CHECK: shouldn't this transition to the error state instead of endlessly looping?

        # load configuration, blocks in case of failure
        log.info("loading config")
        try:
            machine.cfg = load_config()

            machine.debug = machine.cfg['debug']  # set debug level
            machine.connection = get_connection(machine.lte,
                                                machine.cfg)  # initialize connection object depending on config
            machine.api = ubirch.API(machine.cfg)  # set up API for backend communication
        except Exception as e:
            log.exception("failed loading configuration {}".format(str(e)))
            # generate and try to send an emergency event
            event = ({
                'properties.variables.lastError': {'value': str(e), 'sentAt': _formated_time()}
            })
            _send_emergency_event(machine, event) # CHECK: if transitions to an error state are implemented for SysInit, it might be better to send the
                                                  #  emergency message in the error state (?)
            while True:  # watchdog will get out of here
                pycom_machine.idle() # CHECK: shouldn't this transition to the error state instead of endlessly looping?

        # create an instance of the elevate API, which needs the configuration
        machine.elevate_api = ElevateAPI(machine.cfg)

        # configure connection timeouts according to config
        if isinstance(machine.connection, NB_IoT):
            machine.connection.setattachtimeout(machine.cfg["nbiot_extended_attach_timeout"])
            machine.connection.setconnecttimeout(machine.cfg["nbiot_extended_connect_timeout"])

        # get PIN from flash, or bootstrap from backend and then save PIN to flash
        pin_file = imsi + ".bin"
        machine.pin = get_pin_from_flash(pin_file, imsi)
        if machine.pin is None:
            try:
                machine.connection.ensure_connection()
            except Exception as e:
                machine.lastError = str(e)
                machine.go_to_state('error')
                return

            try:
                machine.pin = bootstrap(imsi, machine.api)
                with open(pin_file, "wb") as f:
                    f.write(machine.pin.encode())
            except Exception as e:
                machine.lastError = str(e)
                machine.go_to_state('error') # CHECK: state changes should not happen during enter() as it would recursively call enter of the next state
                                             # if this code is moved into update() later the go_to_state() would be fine here
                return

        # initialise ubirch SIM protocol
        log.info("Initializing ubirch SIM protocol")
        try:
            machine.sim = ElevateSim(lte=machine.lte, at_debug=machine.debug)
        except Exception as e:
            machine.lastError = str(e)
            machine.go_to_state('error')    # CHECK: state changes should not happen during enter() as it would recursively call enter of the next state
                                            # if this code is moved into update() later the go_to_state() would be fine here
            return

        # unlock SIM
        try:
            machine.sim.sim_auth(machine.pin)
        except Exception as e:
            # if PIN is invalid, there is nothing we can do -> block
            if isinstance(e, ValueError):
                machine.breath.set_color(COLOR_SIM_FAIL)
                log.critical("PIN is invalid, can't continue")
                # create an emergency event and try to send it
                event = ({
                    'properties.variables.lastError': {'value': 'PIN is invalid, can\'t continue',
                                                       'sentAt': _formated_time()}
                })
                _send_emergency_event(machine, event) # CHECK: if transitions to an error state are implemented for SysInit, it might be better to send the
                                                      #  emergency message in the error state (?)
                # while True:
                #     machine.wdt.feed()  # avert reset from watchdog   TODO check this, do not want to be stuck here
                #     machine.breath.update()
                time.sleep(180)
                machine.hard_reset() # CHECK: if the PIN is invalid, it will probably still be on next boot, and thus the SIM will become unusable after 3 resets
            else:
                machine.lastError = str(e)
                machine.go_to_state('error')
                return

        # get UUID from SIM
        machine.key_name = "ukey"
        try:
            machine.uuid = machine.sim.get_uuid(machine.key_name)
        except Exception as e:
            machine.lastError = str(e)
            machine.go_to_state('error')
            return
            # log.exception(str(e))
        log.info("UUID: %s", str(machine.uuid))

        # send a X.509 Certificate Signing Request for the public key to the ubirch identity service (once)
        csr_file = "csr_{}_{}.der".format(machine.uuid, machine.api.env)
        if csr_file not in os.listdir():
            try:
                machine.connection.ensure_connection()
            except Exception as e:
                log.exception(str(e))
                # CHECK: shouldn't this transition to an error state? With the current implementation the system will transition to next state without sending CSR

            try:
                csr = submit_csr(machine.key_name, machine.cfg["CSR_country"], machine.cfg["CSR_organization"],
                                 machine.sim, machine.api)
                with open(csr_file, "wb") as f:
                    f.write(csr)
            except Exception as e:
                log.exception(str(e))
                # CHECK: shouldn't this transition to an error state? With the current implementation the system will transition to next state without sending CSR

    def exit(self, machine):
        State.exit(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme

    def update(self, machine):
        State.update(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        now = time.ticks_ms()
        if now >= self.enter_timestamp + STANDARD_DURATION_MS: # CHECK: this is not overflow/wraparound-safe, as "now" can go back to zero.
                                                               # Should usually be "if (now - back_then) >= wait_duration:" for uint, but additionally the docs say
                                                               # 'Performing standard mathematical operations (+, -) or relational operators (<, <=, >, >=) 
                                                               # directly on these value (=ticks) will lead to invalid result.' Instead ticks_diff() must be used:
                                                               # start = time.ticks_ms()
                                                               # while true:
                                                               #     if time.ticks_diff(time.ticks_um(), start) > 500:
                                                               #         print("Time over")
                                                               # Order of arguments matters, too: (newer,older). Probably needs to be checked for every occurence of time.ticks_ms()
                                                               # Documentation also states 'This function should not be used to measure arbitrarily long periods of time.' See also
                                                               # micropython documentation for utime.
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

    def enter(self, machine): # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        State.enter(self, machine)
        machine.breath.set_color(LED_WHITE)

    def exit(self, machine): # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        State.exit(self, machine)

    def _connect(self, machine):
        """
        Connect to the given network
        :param machine: state machine, which holds this state
        :return: True, if connection established, otherwise false
        """
        try:
            machine.connection.ensure_connection()
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
            machine.startTime = time.ticks_ms()

        except Exception as e:
            machine.lastError = str(e)
            machine.go_to_state('error') # CHECK: it might be cleaner to do the state transition in the update functions only (i.e. move this line to update())
            return False

        finally:
            machine.connection.disconnect()

        return True

    def update(self, machine): # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        State.update(self, machine)
        if self._connect(machine):
            machine.go_to_state('sendingDiagnostics')


class StateSendingDiagnostics(State):
    """
    Sending Version Diagnostics to the backend.
    """

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'sendingDiagnostics'

    def enter(self, machine):
        global RESET_REASON
        State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        machine.breath.set_color(LED_YELLOW)

        # CHECK: unsure if sending might not be better placed in update(), especially because of possible exceptions?
        # check the errors in the log and send it
        last_log = self._read_log(2)
        print("LOG: {}".format(last_log)) # CHECK: move print into if clause? or do we want to print an empty 'LOG:' line?
        if last_log != "":
            event = ({'properties.variables.lastLogContent': {'value': last_log}})
            _send_event(machine, event) # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())

        # get the firmware version from OTA
        version = self._get_current_version()

        # get the signal quality and network status
        rssi, ber = machine.sim.get_signal_quality(machine.debug)
        cops = machine.sim.get_network_stats(machine.debug)  # TODO this is not yet decyphered
        event = ({'properties.variables.cellSignalPower': {'value': rssi},
                  'properties.variables.cellSignalQuality': {'value': ber},
                  'properties.variables.cellTechnology': {'value': cops},
                  'properties.variables.hardwareVersion': {'value': '0.9.0'},
                  'properties.variables.firmwareVersion': {'value': version},
                  'properties.variables.resetCause': {'value': RESET_REASON, "sentAt": _formated_time()}})

        _send_event(machine, event) # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())

    def exit(self, machine):
        # set the restart timer for filter tuning # CHECK: lost comment or missing code
        State.exit(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme

    def update(self, machine):
        State.update(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        now = time.ticks_ms()
        if now >= self.enter_timestamp + STANDARD_DURATION_MS: # CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
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
        BACKUP_COUNT = 4 + 1 # CHECK: why '4', why '+1' ?
        last_log = ""
        error_counter = 0
        file_index = 1
        filename = logging.FILENAME
        # CHECK: might be nicer to build a list of files to parse by checking for existence in a for loop first (list = mylog.txt, mylog.txt.1, etc) 
        # and then handle parsing for errors by iterating over that list ('for logfile in all_logfiles_list'), to avoid the repetitive code below i.e.
        # determine which logfiles exist, then parse them starting with the newest one and break when num_errors is reached
        if filename in os.listdir():
            with open(filename, 'r') as reader:
                lines = reader.readlines() # CHECK: i think we can close the reader after this point (not needed while analyzing lines)
                for line in reversed(lines):
                    # only take the error messages from the log
                    if "ERROR" in line[:42]:  # only look at the beginning of the line # CHECK: why '42'?
                        error_counter += 1
                        if error_counter > num_errors:
                            file_index = BACKUP_COUNT
                            break
                        last_log += (line.rstrip('\n'))
                        # check if the message was closed with "}", if not, add it to ensure json
                        if not "}" in line:
                            last_log += "}," # CHECK: if the last '}' is missing, doesnt this create a bracket imbalance? why would there be a '{' without an '}'?
                        else:
                            last_log += ","
                    else:
                        pass
        while file_index < BACKUP_COUNT:
            if (filename + '.{}'.format(file_index)) in os.listdir():
                with open((filename + '.{}'.format(file_index)), 'r') as reader:
                    # print("file {} opened".format(file_index))
                    lines = reader.readlines() # CHECK: i think we can close the reader after this point (not needed while analyzing lines)
                    for line in reversed(lines):
                        # only take the error messages from the log
                        if "ERROR" in line[:42]:  # only look at the beginning of the line # CHECK: why '42'?
                            error_counter += 1
                            if error_counter > num_errors:
                                file_index = BACKUP_COUNT
                                break
                            last_log += (line.rstrip('\n'))
                            if not "}" in line:
                                last_log += "}," # CHECK: if the last '}' is missing, doesnt this create a bracket imbalance? why would there be a '{' without an '}'?
                            else:
                                last_log += ","
                        else:
                            pass
            file_index += 1
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

    def enter(self, machine):
        try:
            State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
            machine.breath.set_color(LED_PURPLE)
        except Exception as e:
            log.exception(str(e))
            machine.go_to_state('error') # CHECK: recursive enter()

    def exit(self, machine):
        State.exit(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme

    def update(self, machine):
        sensor_moved = State.update(self, machine) # CHECK: the movement detection is kind of hidden in update() and seems misplaced, see also the comment in State.update()
        # wait 30 seconds for filter to tune in
        now = time.ticks_ms()
        if now >= self.enter_timestamp + WAIT_FOR_TUNING_MS:# CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
            if sensor_moved:
                machine.go_to_state('measuringPaused')
                return
            # else:
            #     machine.speed()
            # print("sensor tuning in with ({})".format(machine.speed()))

            now = time.ticks_ms() # CHECK: kind of redundant
            if now >= self.enter_timestamp + machine.intervalForInactivityEventMs: # CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
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

    def enter(self, machine):
        State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        machine.breath.set_color(LED_GREEN)
        machine.intervalForInactivityEventMs = FIRST_INTERVAL_INACTIVITY_MS
        machine.sensor.poll_sensors()
        event = ({
            'properties.variables.isWorking': {'value': True, 'sentAt': _formated_time()},
            'properties.variables.acceleration': {
                'value': 1 if machine.sensor.speed_max > abs(machine.sensor.speed_min) else -1}, # CHECK: what should this detect? direction? if yes: unsure if this works with single max/min values
            'properties.variables.accelerationMax': {'value': machine.sensor.speed_max},
            'properties.variables.accelerationMin': {'value': machine.sensor.speed_min},
            'properties.variables.altitude': {'value': machine.sensor.altitude},
            'properties.variables.temperature': {'value': machine.sensor.temperature}
        })
        _send_event(machine, event, ubirching=True) # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())
        # now send the state log also
        event = ({'properties.variables.lastLogContent': {'value': _concat_state_log(machine)}}) # CHECK: if _send_event (on next line) fails, the state log is lost (_concat_state_log clears it immediately)
        _send_event(machine, event) # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())

    def exit(self, machine):
        # # reset the speed values for next time
        # machine.sensor.speed_max[0] = 0.0
        # machine.sensor.speed_max[1] = 0.0
        # machine.sensor.speed_max[2] = 0.0
        # machine.sensor.speed_min[0] = 0.0
        # machine.sensor.speed_min[1] = 0.0
        # machine.sensor.speed_min[2] = 0.0
        #
        # machine.sensor.accel_max = 0.0
        # machine.sensor.accel_min = 0.0
        State.exit(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme

    def update(self, machine):
        State.update(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        now = time.ticks_ms()
        if now >= self.enter_timestamp + OVERSHOOT_DETECTION_PAUSE_MS: # CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
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

    def enter(self, machine):
        State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        machine.breath.set_color(LED_BLUE)

        if machine.intervalForInactivityEventMs < MAX_INACTIVITY_TIME_MS:
            machine.intervalForInactivityEventMs *= EXP_BACKOFF_INACTIVITY # CHECK: in the worst case, max inactivity interval might be almost twice as big as MAX_INACTIVITY_TIME_MS. might need additional 'if' for capping

        machine.sensor.poll_sensors()
        event = ({
            'properties.variables.altitude': {'value': machine.sensor.altitude},
            'properties.variables.temperature': {'value': machine.sensor.temperature},
            'properties.variables.lastLogContent': {'value': _concat_state_log(machine)} # CHECK: if _send_event (on next line) fails, the state log is lost (_concat_state_log clears it immediately)
        })
        _send_event(machine, event)# CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())

        self.new_log_level, self.new_state = _get_state_from_backend(machine) # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())
        log.info("New log level: ({}), new backend state:({})".format(self.new_log_level, self.new_state))
        log.debug("Increased interval for inactivity events to {}".format(machine.intervalForInactivityEventMs))

    def exit(self, machine):
        State.exit(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme

    def update(self, machine):
        sensor_moved = State.update(self, machine) # CHECK: the movement detection is kind of hidden in update() and seems misplaced, see also the comment in State.update()
        # wait for filter to tune in
        now = time.ticks_ms()
        if now >= self.enter_timestamp + WAIT_FOR_TUNING_MS: # CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
            if sensor_moved:
                machine.go_to_state('measuringPaused')
                return
        # else:
        #     machine.speed()
        # print("sensor tuning in with ({})".format(machine.speed()))

        if now >= self.enter_timestamp + machine.intervalForInactivityEventMs: # CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
            machine.go_to_state('inactive')
            return

        if now >= machine.startTime + RESTART_OFFSET_TIME_MS: # CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
            log.info("its time to restart")
            machine.go_to_state('bootloader')
            return

        self._adjust_level_state(machine, self.new_log_level, self.new_state) # CHECK: Possible BUG: this *always* sets a new state (either blinking, bootloader, error, or waitingForOvershoot)
                                                                              # and thus, we will always *immediately* leave the 'inactive' state, making the if cases/transistions above useless
                                                                              # (unless restart time is already over when entering 'inactive' state)

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

    def enter(self, machine):
        State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        machine.breath.set_blinking()

    def exit(self, machine):
        State.exit(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        machine.breath.reset_blinking()

    def update(self, machine):
        State.update(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
        now = time.ticks_ms()
        if now >= self.enter_timestamp + BLINKING_DURATION_MS: # CHECK: this is not overflow/wraparound-safe, check StateInitSystem.update() comment for details
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

    def enter(self, machine):
        try:  # just build that in, because of recent error, which caused the controller to be BRICKED TODO: check this again
            State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
            machine.breath.set_color(LED_RED)

            if machine.lastError:
                log.error("Last error: {}".format(machine.lastError))

            # try to send the error message
            event = ({
                'properties.variables.lastError': {
                    'value': machine.lastError if machine.lastError is not None else "unknown",
                    'sentAt': _formated_time()}
            })
            _send_emergency_event(machine, event)

            machine.sim.deinit()
            machine.lte.deinit(detach=True, reset=True)

        finally:
            time.sleep(3)
            pycom_machine.deepsleep(1)  # this will wakeup with reset in the main again

    def exit(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass # CHECK: if this should never be entered unless something goes very wrong, it might be better to raise an error instead of silently passing here

    def update(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass # CHECK: if this should never be entered unless something goes very wrong, it might be better to raise an error instead of silently passing here


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

    def enter(self, machine):
        try:  # just build tha in, because of recent error, which caused the controller to be BRICKED TODO: check this again
            State.enter(self, machine) # CHECK: adapt to new base class/specific class enter/exit/update function scheme
            machine.breath.set_color(LED_RED)
            machine.sim.deinit()

        finally:
            time.sleep(1)
            pycom_machine.reset()

    def exit(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass

    def update(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass


################################################################################

def _send_event(machine, event: dict, ubirching: bool = False): # CHECK: maybe move this into machine? but its a bit unclear if machine class represents 'the state machine' or 'the system'
    """
    Send the data to elevate and the UPP to ubirch
    :param ubirching:
    :param machine: state machine, which provides the connection and the ubirch protocol
    :param event: name of the event to send
    :param current_time: time variable
    :return:
    """
    print("SENDING")

    # make the elevate data package
    elevate_serialized = serialize_json(event)
    log.debug("Sending Elevate HTTP request body: {}".format(json.dumps(event)))

    try:
        if ubirching:
            elevate_serialized = serialize_json(event)
            # unlock SIM
            machine.sim.sim_auth(machine.pin)

            # seal the data message (data message will be hashed and inserted into UPP as payload by SIM card)
            print("++ creating UPP")
            upp = machine.sim.message_chained(machine.key_name, elevate_serialized, hash_before_sign=True)
            print("\tUPP: {}\n".format(ubinascii.hexlify(upp).decode()))

            # get UPP backlog from flash and add new UPP
            upps = get_backlog(UPP_BACKLOG_FILE)
            upps.append(ubinascii.hexlify(upp).decode())

        # get event backlog from flash and add new event
        events = get_backlog(EVENT_BACKLOG_FILE)
        events.append(json.dumps(event))

        # send events
        machine.connection.ensure_connection()

        try:
            while len(events) > 0:
                # send data message to data service, with reconnects/modem resets if necessary
                status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                         machine.elevate_api.send_data, machine.uuid,
                                                         events[0])
                log.debug("RESPONSE: {}".format(content))

                if not 200 <= status_code < 300:
                    log.error("BACKEND RESP {}: {}".format(status_code, content))  # TODO check error log content!
                    write_backlog(events, EVENT_BACKLOG_FILE, BACKLOG_MAX_LEN)
                    if ubirching:
                        write_backlog(upps, UPP_BACKLOG_FILE, BACKLOG_MAX_LEN)
                    machine.connection.disconnect()
                    return
                else:
                    # event was sent successfully and can be removed from backlog
                    events.pop(0)

        except Exception:
            # sending failed, write unsent messages to flash and terminate
            write_backlog(events, EVENT_BACKLOG_FILE, BACKLOG_MAX_LEN)
            if ubirching:
                write_backlog(upps, UPP_BACKLOG_FILE, BACKLOG_MAX_LEN)
            raise

        # send UPPs
        machine.connection.ensure_connection()

        try:
            if ubirching:
                while len(upps) > 0:
                    # send UPP to the ubirch authentication service to be anchored to the blockchain
                    status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                             machine.api.send_upp, machine.uuid,
                                                             ubinascii.unhexlify(upps[0]))
                    try:
                        log.debug("NIOMON RESPONSE: ({}) {}".format(status_code,
                                                                    "" if status_code == 200 else ubinascii.hexlify(
                                                                        content).decode()))
                    except Exception:  # this is only excaption handling in case the content can not be decyphered
                        pass
                    # communication worked in general, now check server response
                    if not 200 <= status_code < 300 and not status_code == 409:
                        log.error("NIOMON RESP {}".format(status_code))
                    else:
                        # UPP was sent successfully and can be removed from backlog
                        upps.pop(0)

            else:
                pass
        except Exception:
            # sending failed, write unsent messages to flash and terminate
            raise
        finally:
            write_backlog(events, EVENT_BACKLOG_FILE, BACKLOG_MAX_LEN)
            if ubirching:
                write_backlog(upps, UPP_BACKLOG_FILE, BACKLOG_MAX_LEN)

    except Exception as e:
        # if too many communications fail, reset the system
        machine.failedBackendCommunications += 1
        if machine.failedBackendCommunications > 3:
            log.exception(str(e) + "doing RESET")
            machine.go_to_state('error') # CHECK: state transistion in global function (outside of state / state machine)
        else:
            log.exception(str(e))

    finally:
        machine.connection.disconnect()


def _send_emergency_event(machine, event: dict):# CHECK: maybe move this into machine? but its a bit unclear if machine class represents 'the state machine' or 'the system'
    """
    # Send an emergency event to elevate
    :param machine: state machine, which provides the connection and the ubirch protocol
    :param event: name of the event to send
    :return:
    """
    print("SENDING")

    # make the elevate data package
    log.debug("Sending Elevate HTTP request body: {}".format(json.dumps(event)))

    try:
        machine.connection.ensure_connection()
        # send data message to data service, with reconnects/modem resets if necessary
        status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                 machine.elevate_api.send_data, machine.uuid,
                                                 json.dumps(event))
        log.debug("RESPONSE: {}".format(content))

    except Exception as e:
        log.exception(str(e))

    finally:
        machine.connection.disconnect()


def _get_state_from_backend(machine):
    """
    Get the current state and log level from the elevate backend
    :param machine: state machine, providing the connection
    :return: log level and new state
    """
    level = ""
    state = ""
    # CHECK: TODO: document what the backend might reply, e.g. especially if "empty" state information is possible

    # send data message to data service, with reconnects/modem resets if necessary
    try:
        machine.connection.ensure_connection()
        status_code, level, state = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                      machine.elevate_api.get_state, machine.uuid, '')
        # communication worked in general, now check server response
        if not 200 <= status_code < 300:
            log.error("Elevate backend returned HTTP error code {}".format(status_code))
    except Exception as e:
        log.exception(str(e))
        machine.go_to_state('error')

    finally:
        machine.connection.disconnect()

    return level, state


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
