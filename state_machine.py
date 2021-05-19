"""
from: https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code
"""
import machine
import ubinascii
import uos as os
import system

import lib.logging as logging
from lib.helpers import *
from lib.realtimeclock import enable_time_sync, wait_for_sync, board_time_valid, NTP_SERVER_BACKUP


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

RESTART_OFFSET_TIME_S = 24 * 60 * 60 + (
            int.from_bytes(os.urandom(2), "big") % 0x0FFF)  # define restart time = 1 day + (0 .. 4095) seconds

# setup the logging
log = logging.getLogger()
log.info("RESTART IN: {} s".format(RESTART_OFFSET_TIME_S))

# get the reason for reset in readable form
RESET_REASON = translate_reset_cause(machine.reset_cause())


################################################################################
# State Machine

class StateMachine(object):

    def __init__(self):
        self.wdt = machine.WDT(timeout=WATCHDOG_TIMEOUT_MS)
        self.wdt.feed()
        self.state = None
        self.states = {}
        self.lastError = None
        self.failedBackend_Communications = 0
        self.timeStateLog = []
        self.sensorOverThresholdFlag = False
        self.system = None

        # set all necessary time values
        self.intervalForInactivityEventS = FIRST_INTERVAL_INACTIVITY_S

        self.startTime = 0

        log.info("\033[0;35m[Core] Initializing magic... \033[0m ✨ ")
        log.info("[Core] Hello, I am %s", ubinascii.hexlify(machine.unique_id()))

    def add_state(self, state):
        """
        Add a new state to the state machine
        :param state: new state to add
        """
        if not state.name in self.states:  # check if state already exists
            self.states[state.name] = state
        else:
            log.error("cannot add state :({}), it already exists".format(state.name))

    def go_to_state(self, state_name):
        """
        Go to the state, which is indicated in the state_name
        :param state_name: new state to go to.
        """
        if not state_name in self.states:  # check if state already exists
            log.error("cannot go to unknown state: ({})".format(state_name))
            state_name = 'error'  # go to error state instead
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
                if 'error' in self.states:
                    self.go_to_state('error')
                else:
                    raise Exception("Faulty state {}".format(repr(e)))

    def concat_state_log(self):
        """
        Helper function to concatenate the state transition log and clear it.
        :return comma separated state transition log string
        """
        state_log = ""
        for lines in self.timeStateLog:
            state_log += lines + ","
        self.timeStateLog.clear()
        return state_log.rstrip(",")


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

    def enter(self, state_machine):
        """
        Enter a specific state. This is called, when a new state is entered.
        Get the timestamp for entering, so it can be used in all states
        :param state_machine: state machine, which has the state
        """
        try:
            log.debug('Entering {}'.format(self.name))
            self.enter_timestamp = time.time()
            # add the timestamp and state name to a log, for later sending
            state_machine.timeStateLog.append(formated_time() + ":" + self.name)
            self._enter(state_machine)
        except Exception as e:
            log.exception("Enter: {}".format(str(e)))
            raise e

    def _enter(self, state_machine):
        """
        Enter a specific state
        This is an abstract method, which has to be implemented in every child class.
        :param state_machine: state machine, which has the state
        """
        raise NotImplementedError()

    def exit(self, state_machine):
        """
        Exit a specific state. This is called, when the old state is left.
        :param state_machine: state machine, which has the state.
        """
        try:
            log.debug('Exiting {}'.format(self.name))
            self._exit(state_machine)
        except Exception as e:
            log.exception("Exit: {}".format(str(e)))
            raise e

    def _exit(self, state_machine):
        """
        Enter a specific state
        This is an abstract method, which has to be implemented in every child class.
        :param state_machine: state machine, which has the state
        """
        raise NotImplementedError()

    def update(self, state_machine):
        """
        Update the current state, which means to run through the update routine.
        Breath with the LED. and calculate the speed from the measured accelerometer values.
        :param state_machine: state machine, which has the state.
        :return: True, to indicate, the function was called.
        """
        try:
            # check if the system is already initialised - skip if it is not
            if state_machine.system is not None:
                state_machine.system.led_breath.update()

            self._update(state_machine)
        except Exception as e:
            log.exception("Update: {}".format(str(e)))
            raise e

    def _update(self, state_machine):
        """
        Update a specific state.
        This is an abstract method, which has to be implemented in every child class.
        :param state_machine: state machine, which has the state
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

    def _enter(self, state_machine):
        # TODO breath not possible here since the system class is not yet initialized
        # state_machine.system.led_breath.set_color(LED_TURQUOISE)
        return

    def _exit(self, state_machine):
        pass

    def _update(self, state_machine):
        try:
            state_machine.system = system.System()
        except OSError as e:
            log.exception(str(e))
            state_machine.lastError = str(e)

            machine.reset()

        now = time.time()
        if now >= self.enter_timestamp + STANDARD_DURATION_S:
            state_machine.go_to_state('connecting')


class StateConnecting(State):
    """
    Connecting State to connect to network.
    """

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'connecting'

    def _enter(self, state_machine):
        state_machine.system.led_breath.set_color(LED_WHITE)

    def _exit(self, state_machine):
        pass

    def _connect(self, state_machine):
        """
        Connect to the given network
        :param state_machine: state machine, which holds this state
        :return: True, if connection established, otherwise false
        """
        try:
            state_machine.system.connection.ensure_connection()
            enable_time_sync()
            log.info("\twaiting for time sync")
            wait_for_sync(print_dots=True)
            if not board_time_valid():
                enable_time_sync(server=NTP_SERVER_BACKUP)
                log.info("\twaiting for backup time sync")
                wait_for_sync(print_dots=True)
                if not board_time_valid():
                    raise Exception("Time sync failed", time.time())
            # update the start time
            state_machine.startTime = time.time()

        except Exception as e:
            state_machine.lastError = str(e)
            return False

        finally:
            state_machine.system.connection.disconnect()

        return True

    def _update(self, state_machine):
        if self._connect(state_machine):
            state_machine.go_to_state('sendingDiagnostics')
        else:
            state_machine.go_to_state('error')


class StateSendingDiagnostics(State):
    """
    Sending Version Diagnostics to the backend.
    """

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'sendingDiagnostics'

    def _enter(self, state_machine):
        state_machine.system.led_breath.set_color(LED_YELLOW)

    def _exit(self, state_machine):
        pass

    def _update(self, state_machine):
        global RESET_REASON
        # check the errors in the log and send it
        last_log = read_log(2)
        if not last_log == "":
            print("LOG: {}".format(last_log))
            event = ({'properties.variables.lastLogContent': {'value': last_log}})
            state_machine.system.send_event(event)

        # get the firmware version from OTA
        version = get_current_version()

        # get the signal quality and network status
        rssi, ber = state_machine.system.sim.get_signal_quality()
        cops = state_machine.system.sim.get_network_stats()  # TODO this is not yet decyphered
        event = ({'properties.variables.cellSignalPower': {'value': rssi},
                  'properties.variables.cellSignalQuality': {'value': ber},
                  'properties.variables.cellTechnology': {'value': cops},
                  'properties.variables.hardwareVersion': {'value': '0.9.0'},
                  'properties.variables.firmwareVersion': {'value': version},
                  'properties.variables.resetCause': {'value': RESET_REASON, "sentAt": formated_time()}})

        state_machine.system.send_event(event)

        now = time.time()
        if now >= self.enter_timestamp + STANDARD_DURATION_S:
            state_machine.go_to_state('waitingForOvershoot')


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

    def _enter(self, state_machine):
        state_machine.system.led_breath.set_color(LED_PURPLE)

    def _exit(self, state_machine):
        pass

    def _update(self, state_machine):
        now = time.time()
        if now >= state_machine.startTime + RESTART_OFFSET_TIME_S:
            log.info("its time to restart")
            state_machine.go_to_state('bootloader')
            return

        # wait 30 seconds for filter to tune in
        if now >= self.enter_timestamp + WAIT_FOR_TUNING_S:
            if state_machine.system.get_movement():  # movement:
                state_machine.go_to_state('measuringPaused')
                return

        if now >= self.enter_timestamp + state_machine.intervalForInactivityEventS:
            state_machine.go_to_state('inactive')
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

    def _enter(self, state_machine):
        state_machine.system.led_breath.set_color(LED_GREEN)

    def _exit(self, state_machine):
        pass

    def _update(self, state_machine):
        state_machine.intervalForInactivityEventS = FIRST_INTERVAL_INACTIVITY_S
        state_machine.system.poll_sensors()
        event = ({
            'properties.variables.isWorking': {'value': True, 'sentAt': formated_time()},
            'properties.variables.acceleration': {
                'value': 1 if state_machine.system.get_speed_max() > abs(state_machine.system.get_speed_min()) else -1},
            'properties.variables.accelerationMax': {'value': state_machine.system.get_speed_max()},
            'properties.variables.accelerationMin': {'value': state_machine.system.get_speed_min()},
            'properties.variables.altitude': {'value': state_machine.system.get_altitude()},
            'properties.variables.temperature': {'value': state_machine.system.get_temperature()}
        })
        state_machine.system.send_event(event, ubirching=True)
        # now send the state log also
        # last_log = state_machine.concat_state_log() TODO CHECK this might be obsolete
        # if not last_log == "":
        #     event = ({'properties.variables.lastLogContent': {'value': last_log}})
        #     state_machine.system.send_event(event)

        now = time.time()
        if now >= self.enter_timestamp + STANDARD_DURATION_S:
            state_machine.go_to_state('waitingForOvershoot')


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

    def _enter(self, state_machine):
        state_machine.system.led_breath.set_color(LED_BLUE)

        if state_machine.intervalForInactivityEventS < MAX_INACTIVITY_TIME_S:
            state_machine.intervalForInactivityEventS *= EXP_BACKOFF_INACTIVITY
        else:
            state_machine.intervalForInactivityEventS = MAX_INACTIVITY_TIME_S

    def _exit(self, state_machine):
        pass

    def _update(self, state_machine):

        state_machine.system.poll_sensors()
        event = ({
            'properties.variables.altitude': {'value': state_machine.system.get_altitude()},
            'properties.variables.temperature': {'value': state_machine.system.get_temperature()}
        })
        last_log = state_machine.concat_state_log()
        if not last_log == "":
            event.update({'properties.variables.lastLogContent': {'value': last_log}})

        state_machine.system.send_event(event)

        self.new_log_level, self.new_state = state_machine.system.get_state_from_backend()  # CHECK: This might raise an exception which will not be caught, also contains state transitions (recursive enter())
        log.info("New log level: ({}), new backend state:({})".format(self.new_log_level, self.new_state))
        log.debug("Increased interval for inactivity events to {}".format(state_machine.intervalForInactivityEventS))

        self._adjust_level_state(state_machine, self.new_log_level, self.new_state)

    def _adjust_level_state(self, state_machine, level, state):
        """
        Adjust the logging level and the current state
        :param state_machine: state machine holding this state
        :param level: new logging level to adjust
        :param state: new sensor state to adjust
        """
        log.setLevel(translate_backend_log_level(level))
        state_machine.go_to_state(translate_backend_state_name(state))


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

    def _enter(self, state_machine):
        state_machine.system.led_breath.set_blinking()

    def _exit(self, state_machine):
        state_machine.system.led_breath.reset_blinking()

    def _update(self, state_machine):
        now = time.time()
        if now >= self.enter_timestamp + BLINKING_DURATION_S:
            state_machine.go_to_state('inactive')  # this is necessary to fetch a new state from backend


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

    def _enter(self, state_machine):
        if state_machine.system is not None and state_machine.system.led_breath is not None:
            state_machine.system.led_breath.set_color(LED_RED)

    def _exit(self, state_machine):
        raise SystemError("exiting the error state should never happen here")

    def _update(self, state_machine):
        try:  # just build that in, because of recent error, which caused the controller to be BRICKED TODO: check this again
            if state_machine.lastError:
                log.error("Last error: {}".format(state_machine.lastError))

            # try to send the error message
            event = ({
                'properties.variables.lastError': {
                    'value': state_machine.lastError if state_machine.lastError is not None else "unknown",
                    'sentAt': formated_time()}
            })
            state_machine.system.send_emergency_event(event)

            state_machine.system.sim.deinit()
            state_machine.system.lte.deinit(detach=True, reset=True)

        finally:
            time.sleep(3)
            machine.deepsleep(1)  # this will wakeup with reset in the main again


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

    def _enter(self, state_machine):
        state_machine.system.led_breath.set_color(LED_WHITE_BRIGHT)

    def _exit(self, state_machine):
        raise SystemError("exiting the bootloader state should never happen here")

    def _update(self, state_machine):
        try:  # just build that in, because of recent error, which caused the controller to hang
            state_machine.system.sim.deinit()

        finally:
            time.sleep(1)
            machine.reset()

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
