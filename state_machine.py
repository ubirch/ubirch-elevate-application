"""
from: https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code
"""
import uos as os
import utime as time
import machine as pycom_machine
import ubinascii
from lib.config import *
from lib.connection import get_connection, NB_IoT
from lib.elevate_api import ElevateAPI
from lib.elevate_sim import ElevateSim
from lib.helpers import *
from lib.modem import get_imsi
from network import LTE
from lib.pyboard import *
from lib.realtimeclock import enable_time_sync, wait_for_sync, board_time_valid, NTP_SERVER_BACKUP
from sensor import MovementSensor
from sensor_config import *
import ujson as json
import logging
import ubirch

# set watchdog: if execution hangs/takes longer than 'timeout' an automatic reset is triggered
# we need to do this as early as possible in case an import cause a freeze for some reason
wdt = machine.WDT(timeout=6 * 60 * 1000)  # we set it to 6 minutes here and will reconfigure it when we have loaded the configuration
wdt.feed()

# backlog constants
EVENT_BACKLOG_FILE = "event_backlog.txt"
UPP_BACKLOG_FILE = "upp_backlog.txt"
BACKLOG_MAX_LEN = 10   # max number of events / UPPs in the backlogs

VERSION_FILE = "OTA_VERSION.txt"

# timing
STANDARD_DURATION_MS = 500
BLINKING_DURATION_MS = 60000
WAIT_FOR_TUNING_MS = 10000

MAX_INACTIVITY_TIME_MS = 60 * 60 * 1000 # min * sec * msec
FIRST_INTERVAL_INACTIVITY_MS = MAX_INACTIVITY_TIME_MS / 32 # =112500 msec
EXP_BACKOFF_INACTIVITY = 2
OVERSHOOT_DETECTION_PAUSE_MS = 60 * 1000 # sec * msec

RESTART_OFFSET_TIME_MS = 24 * 60 * 60 * 1000 + (int.from_bytes(os.urandom(3), "big") % 0x3FFFFF)
print("RESTART IN: {}ms".format(RESTART_OFFSET_TIME_MS))


log = logging.getLogger()

################################################################################
# State Machine

class StateMachine(object):

    def __init__(self):
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

        # create instances of required objects
        try:
            self.sensor = MovementSensor()
            self.lte = LTE() #psm_period_value=1, psm_period_unit=LTE.PSM_PERIOD_1H,
                     #  psm_active_value=5, psm_active_unit=LTE.PSM_ACTIVE_1M)
            self.breath = LedBreath()
        except OSError as e:
            log.exception(str(e))
            self.lastError = str(e)
            self.hard_reset()
            log.error("I should not get here")
            # self.go_to_state('error')
            pycom_machine.reset()

        # set all necessary time values
        self.intervalForInactivityEventMs = FIRST_INTERVAL_INACTIVITY_MS

        self.startTime = 0

        log.info("\033[0;35m[Core] Initializing magic... \033[0m ✨ ")
        log.info("[Core] Hello, I am %s",  ubinascii.hexlify(pycom_machine.unique_id()))


    def speed(self):
        """
        Calculate th maximum absolute speed value from current sensor values,
        over all axis.
        :return: the maximum value of the currently measured speed.
        """
        self.max_speed = 0.0
        self.min_speed = 0.0
        i = 0

        while i < 3:
            if self.sensor.speed_max[i] > self.max_speed:
                self.max_speed = self.sensor.speed_max[i]

            if self.sensor.speed_min[i] < self.min_speed:
                self.min_speed = self.sensor.speed_min[i]

            self.sensor.speed_min[i] = 0.0
            self.sensor.speed_max[i] = 0.0
            i += 1

        # now check the movement into the same direction
        if self.max_speed > g_THRESHOLD:
            # print("SPEED MAX= {}".format(self.max_speed))
            return True
        if abs(self.min_speed) > g_THRESHOLD:
            # print("SPEED MIN= {}".format(self.min_speed))
            return True

        return False

    def add_state(self, state):
        """
        Add a new state to the state machine
        :param state: new state to add
        """
        self.states[state.name] = state

    def go_to_state(self, state_name):
        """
        Go to the state, which is indicated in the state_name
        :param state_name: new state to go to.
        """
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

    def hard_reset(self):
        """
        Hard-Reset GPy and Modem, by triggering power OFF/ON from Pysense.
        """
        time.sleep(1)
        self.sensor.pysense.reset_cmd()

############################
def _formated_time():
    ct = time.localtime()
    return "{0:04d}-{1:02d}-{2:02d}T{3:02d}:{4:02d}:{5:02d}Z".format(*ct)  # modified to fit the correct format


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
        :return name string of the state.
        """
        return ''

    def enter(self, machine):
        """
        Enter a specific state. This is called, when a new state is entered.
        Get the timestamp for entering, so it can be used in all states
        :param machine: state machine, which has the state
        """
        self.enter_timestamp = time.ticks_ms()
        pass

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
        machine.breath.update()

        if machine.sensor.trigger:
            machine.sensor.calc_speed()
        return True


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
        State.enter(self, machine)
        machine.breath.set_color(LED_TURQUOISE)

        try:
            # reset modem (modem might be in a strange state)
            log.warning("not coming from sleep, resetting modem")
            reset_modem(machine.lte)

            log.info("getting IMSI")
            imsi = get_imsi(machine.lte)
            log.info("IMSI: " + imsi)
        except Exception as e:
            log.exception("failed setting up modem {}".format(str(e)))
            while True: # watchdog will get out of this
                pycom_machine.idle()

        # load configuration, blocks in case of failure
        log.info("loading config")
        try:
            machine.cfg = load_config()

            machine.debug = machine.cfg['debug']  # set debug level
            machine.connection = get_connection(machine.lte, machine.cfg)  # initialize connection object depending on config
            machine.api = ubirch.API(machine.cfg)  # set up API for backend communication
        except Exception as e:
            log.exception("failed loading configuration {}".format(str(e)))
            # generate and try to send an emergency event
            event = ({
                'properties.variables.lastError': {'value': str(e), 'sentAt': _formated_time()}
            })
            _send_emergency_event(machine, event)
            while True: # watchdog will get out of here
                pycom_machine.idle()

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
                machine.go_to_state('error')
                return

        # initialise ubirch SIM protocol
        log.info("Initializing ubirch SIM protocol")
        try:
            machine.sim = ElevateSim(lte=machine.lte, at_debug=machine.debug)
        except Exception as e:
            machine.lastError = str(e)
            machine.go_to_state('error')
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
                    'properties.variables.lastError': {'value': 'PIN is invalid, can\'t continue', 'sentAt': _formated_time()}
                })
                _send_emergency_event(machine, event)
                while True:
                    wdt.feed()  # avert reset from watchdog   TODO check this, do not want to be stuck here
                    machine.breath.update()
            else:
                machine.lastError = str(e)
                machine.go_to_state('error')
                return

        # get UUID from SIM
        machine.key_name = "ukey"
        try:
            machine.uuid = machine.sim.get_uuid(machine.key_name)
        except Exception as e:
            log.exception(str(e))
        log.info("UUID: %s", str(machine.uuid))

        # send a X.509 Certificate Signing Request for the public key to the ubirch identity service (once)
        csr_file = "csr_{}_{}.der".format(machine.uuid, machine.api.env)
        if csr_file not in os.listdir():
            try:
                machine.connection.ensure_connection()
            except Exception as e:
                log.exception(str(e))

            try:
                csr = submit_csr(machine.key_name, machine.cfg["CSR_country"], machine.cfg["CSR_organization"], machine.sim, machine.api)
                with open(csr_file, "wb") as f:
                    f.write(csr)
            except Exception as e:
                log.exception(str(e))

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.enter_timestamp + STANDARD_DURATION_MS:
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

    def enter(self, machine):
        State.enter(self, machine)
        machine.breath.set_color(LED_WHITE)

    def exit(self, machine):
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
            machine.go_to_state('error')
            return False

        finally:
            machine.connection.disconnect()

        return True

    def update(self, machine):
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
        State.enter(self, machine)
        machine.breath.set_color(LED_YELLOW)

        # check the errors in the log and send it
        last_log = self._read_log(3)
        print("LOG: {}".format(last_log))
        if last_log != "":
            event = ({'properties.variables.lastLogContent':{'value': last_log}})
            _send_event(machine, event)

        # get the firmware version from OTA
        version = self._get_current_version()

        # get the signal quality and network status
        rssi, ber = machine.sim.get_signal_quality(machine.debug)
        cops = machine.sim.get_network_stats(machine.debug)  # TODO this is not yet decyphered
        event = ({'properties.variables.cellSignalPower': {'value': rssi},
                  'properties.variables.cellSignalQuality': {'value': ber},
                  'properties.variables.cellTechnology': {'value': cops},
                  'properties.variables.hardwareVersion':{'value': '0.9.0'},
                  'properties.variables.firmwareVersion':{'value': version},
                  'properties.variables.resetCause':{'value':RESET_REASON, "sentAt": _formated_time()}})

        _send_event(machine, event)

    def exit(self, machine):
        # set the restart timer for filter tuning
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.enter_timestamp + STANDARD_DURATION_MS:
                machine.go_to_state('waitingForOvershoot')


    def _get_current_version(self):
        try:
            from OTA_VERSION import VERSION
        except ImportError:
            VERSION = '1.0.0'
        return VERSION


    def _read_log(self, num_errors:int=3):
        """
        Read the last ERRORs from log and form a string of json like list.
        :param num_errors: number of errors to return
        :return: String of the last errors
        :example: {'t':'1970-01-01T00:00:23Z','l':'ERROR','m':...}
        """
        BACKUP_COUNT = 4 + 1
        last_log = ""
        error_counter = 0
        file_index = 1
        filename = logging.FILENAME
        if filename in os.listdir():
            with open(filename, 'r') as reader:
                lines = reader.readlines()
                for line in reversed(lines):
                    # only take the error messages from the log
                    if "ERROR" in line[:42]: # only look at the beginning of the line
                        error_counter += 1
                        if error_counter > num_errors:
                            file_index = BACKUP_COUNT
                            break
                        last_log += (line.rstrip('\n'))
                        # check if the message was closed with "}", if not, add it to ensure json
                        if not "}" in line:
                            last_log += "},"
                        else:
                            last_log += ","
                    else:
                        pass
        while file_index < BACKUP_COUNT:
            if (filename + '.{}'.format(file_index)) in os.listdir():
                with open((filename + '.{}'.format(file_index)), 'r') as reader:
                    # print("file {} opened".format(file_index))
                    lines = reader.readlines()
                    for line in reversed(lines):
                        # only take the error messages from the log
                        if "ERROR" in line[:42]: # only look at the beginning of the line
                            error_counter += 1
                            if error_counter > num_errors:
                                file_index = BACKUP_COUNT
                                break
                            last_log += (line.rstrip('\n'))
                            if not "}" in line:
                                last_log += "},"
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
            State.enter(self, machine)
            machine.breath.set_color(LED_PURPLE)
        except Exception as e:
            log.exception(str(e))
            machine.go_to_state('error')

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            # wait 30 seconds for filter to tune in
            now = time.ticks_ms()
            if now >= self.enter_timestamp + WAIT_FOR_TUNING_MS:
                if machine.speed():
                    machine.go_to_state('measuringPaused')
                    return
            else:
                machine.speed()
                # print("sensor tuning in with ({})".format(machine.speed()))

            if now >= self.enter_timestamp + machine.intervalForInactivityEventMs:
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
        State.enter(self, machine)
        machine.breath.set_color(LED_GREEN)
        machine.intervalForInactivityEventMs = FIRST_INTERVAL_INACTIVITY_MS
        machine.sensor.poll_sensors()
        event = ({
            'properties.variables.isWorking': { 'value': True, 'sentAt': _formated_time()},
            'properties.variables.acceleration': { 'value': 1 if machine.max_speed > abs(machine.min_speed) else -1},
            'properties.variables.accelerationMax': { 'value': machine.max_speed},
            'properties.variables.accelerationMin': { 'value': machine.min_speed},
            'properties.variables.altitude': { 'value': machine.sensor.altitude},
            'properties.variables.temperature': { 'value': machine.sensor.temperature}
        })
        _send_event(machine, event, ubirching=True)

    def exit(self, machine):
        # reset the speed values for next time
        machine.sensor.speed_max[0] = 0.0
        machine.sensor.speed_max[1] = 0.0
        machine.sensor.speed_max[2] = 0.0
        machine.sensor.speed_min[0] = 0.0
        machine.sensor.speed_min[1] = 0.0
        machine.sensor.speed_min[2] = 0.0

        machine.sensor.accel_max = 0.0
        machine.sensor.accel_min = 0.0
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.enter_timestamp + OVERSHOOT_DETECTION_PAUSE_MS:
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
        State.enter(self, machine)
        machine.breath.set_color(LED_BLUE)

        if machine.intervalForInactivityEventMs < MAX_INACTIVITY_TIME_MS:
            machine.intervalForInactivityEventMs *= EXP_BACKOFF_INACTIVITY

        machine.sensor.poll_sensors()
        event = ({
            'properties.variables.altitude': { 'value': machine.sensor.altitude },
            'properties.variables.temperature': { 'value': machine.sensor.temperature }
        })
        _send_event(machine, event)
        self.new_log_level, self.new_state = _get_state_from_backend(machine)
        log.info("New log level: ({}), new backend state:({})".format(self.new_log_level, self.new_state))
        log.debug("Increased interval for inactivity events to {}".format(machine.intervalForInactivityEventMs))

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            # wait for filter to tune in
            now = time.ticks_ms()
            if now >= self.enter_timestamp + WAIT_FOR_TUNING_MS:
                if machine.speed():
                    machine.go_to_state('measuringPaused')
                    return
            else:
                machine.speed()
                # print("sensor tuning in with ({})".format(machine.speed()))

            if now >= self.enter_timestamp + machine.intervalForInactivityEventMs:
                machine.go_to_state('inactive')
                return

            if now >= machine.startTime + RESTART_OFFSET_TIME_MS:
                log.info("its time to restart")
                machine.go_to_state('bootloader')
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

    def enter(self, machine):
        State.enter(self, machine)
        machine.breath.set_blinking()

    def exit(self, machine):
        State.exit(self, machine)
        machine.breath.reset_blinking()

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.enter_timestamp + BLINKING_DURATION_MS:
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
        try: # just build that in, because of recent error, which caused the controller to be BRICKED TODO: check this again
            State.enter(self, machine)
            machine.breath.set_color(LED_RED)

            if machine.lastError:
                log.error("Last error: {}".format(machine.lastError))

            # try to send the error message
            event = ({
                'properties.variables.lastError': {'value': machine.lastError, 'sentAt': _formated_time()}
            })
            _send_emergency_event(machine, event)

            machine.sim.deinit()
            machine.lte.deinit(detach=True, reset=True)

        finally:
            time.sleep(3)
            pycom_machine.deepsleep(1) # this will wakeup with reset in the main again

    def exit(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass

    def update(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass


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
        try: # just build tha in, because of recent error, which caused the controller to be BRICKED TODO: check this again
            State.enter(self, machine)
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

def _send_event(machine, event: dict, ubirching:bool=False):
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
                    log.error("BACKEND RESP {}: {}".format(status_code, content))      # TODO check error log content!
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
                                                                 machine.api.send_upp, machine.uuid, ubinascii.unhexlify(upps[0]))
                    try:
                        log.debug("NIOMON RESPONSE: ({}) {}".format(status_code, "" if status_code == 200 else ubinascii.hexlify(content).decode()))
                    except Exception: # this is only excaption handling in case the content can not be decyphered
                        pass
                    # communication worked in general, now check server response
                    if not 200 <= status_code < 300 and not status_code == 409:
                        log.error("NIOMON RESP {}".format(status_code))
                    else:
                        # UPP was sent successfully and can be removed from backlog
                        upps.pop(0)

            else: pass
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
            machine.go_to_state('error')
        else:
            log.exception(str(e))

    finally:
        machine.connection.disconnect()


def _send_emergency_event(machine, event: dict):
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

