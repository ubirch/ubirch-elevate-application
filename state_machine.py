"""
from: https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code
"""
import uos
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
from lib.realtimeclock import enable_time_sync, wait_for_sync
from sensor import MovementSensor
from sensor_config import *
import ujson as json
import logging
import ubirch


STANDARD_DURATION_MS = 500

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

        # create instances of required objects
        self.sensor = MovementSensor()
        self.lte = LTE(psm_period_value=1, psm_period_unit=LTE.PSM_PERIOD_1H,
                       psm_active_value=5, psm_active_unit=LTE.PSM_ACTIVE_2S)
        self.breath = LedBreath()

        # set all necessary time values
        self.IntervalForDetectingInactivityMs = 60000
        self.OvershotDetectionPauseIntervalMs = 60000
        self.FirstIntervalForInactivityEventMs = 60000
        self.ExponentialBackoffFactorForInactivityEvent = 2
        self.intervalForInactivityEventMs = self.FirstIntervalForInactivityEventMs
        self.intervalForErrorEventMs = 15000
        self.intervalForBlinkingMs = 30000

        self.timerActivity = 0.0
        self.timerLastActivity = 0.0
        self.timerInactivity = 0.0

        log.info("\033[0;35m[Core] Initializing magic... \033[0m ✨ ")
        log.info("[Core] Hello, I am %s",  ubinascii.hexlify(pycom_machine.unique_id()))


    def speed(self):
        """
        Calculate th maximum absolute speed value from current sensor values,
        over all axis.
        :return: the maximum value of the currently measured speed.
        """
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
            log.debug('> > Exiting {} A:{} L:{} I:{}'.format(self.state.name, self.timerActivity, self.timerLastActivity,
                                                         self.timerInactivity))
            self.state.exit(self)
        self.state = self.states[state_name]
        log.debug('> > Entering {} A:{} L:{} I:{}'.format(self.state.name, self.timerActivity, self.timerLastActivity,
                                                      self.timerInactivity))
        self.state.enter(self)

    def update(self):
        """
        Run update function of the current state.
        """
        if self.state:
            # print('Updating %s' % (self.state.name))
            self.state.update(self)

    # When pausing, don't exit the state
    def pause(self):
        """
        Pause the current state. # TODO currently not in use, maybe not necessary.
        """
        self.state = self.states['paused']
        log.debug('> > Pausing')
        self.state.enter(self)

    # When resuming, don't re-enter the state
    def resume_state(self, state_name):
        """
        Resume the paused state. # TODO currently not in use, maybe not necessary.
        :param state_name: indicating the state to resume.
        :note this function does not go through state.enter(),
        but directly back into the state.
        """
        if self.state:
            log.debug('> > Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        log.debug('> > Resuming %s' % (self.state.name))

    def reset_state_machine(self):
        """
        As indicated, reset the machines system's variables. # TODO currently not used.
        """
        log.debug('> > Resetting the machine')

    def enable_interrupt(self):
        """
        Enable the Accelerometer Interrupt for fifo buffer. # TODO currently not in use, maybe unnecessary.
        """
        self.sensor.pysense.accelerometer.restart_fifo()
        self.sensor.pysense.accelerometer.enable_fifo_interrupt(self.sensor.accelerometer_interrupt_cb)

    def disable_interrupt(self):
        """
        Disable the Accelerometer Interrupt for fifo interrupt. # TODO currently not used, maybe not necessary.
        """
        self.sensor.pysense.accelerometer.enable_fifo_interrupt(handler=None)


################################################################################
# States  todo include error handling here

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
        Also breath with the LED.
        :param machine: state machine, which has the state.
        :return: True, to indicate, the function was called.
        """
        machine.breath.update()
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
        global COMING_FROM_DEEPSLEEP
        State.enter(self, machine)
        machine.breath.set_color(LED_TURQUOISE)

        try:
            # reset modem on any non-normal loop (modem might be in a strange state)
            if not COMING_FROM_DEEPSLEEP:
                log.warning("++ not coming from sleep, resetting modem")
                reset_modem(machine.lte)

            log.info("++ getting IMSI")
            imsi = get_imsi(machine.lte)
            log.info("IMSI: " + imsi)
        except Exception as e:
            log.exception("\tERROR setting up modem (%s)", str(e))  # todo long sleep and then reset.
            while True:
                pycom_machine.idle()

        # write IMSI to SD card
        if not COMING_FROM_DEEPSLEEP and SD_CARD_MOUNTED: store_imsi(imsi)
        # load configuration, blocks in case of failure
        log.info("++ loading config")
        try:
            machine.cfg = load_config(sd_card_mounted=SD_CARD_MOUNTED)

            machine.debug = machine.cfg['debug']  # set debug level
            # if machine.debug: print("\t" + repr(machine.cfg))

            machine.connection = get_connection(machine.lte, machine.cfg)  # initialize connection object depending on config
            machine.api = ubirch.API(machine.cfg)  # set up API for backend communication
        except Exception as e:
            log.exception("\tERROR loading configuration (%s)", str(e)) #todo figure out how to behave and not to make a BRICK
            while True:
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
                machine.go_to_state('reset')

            try:
                machine.pin = bootstrap(imsi, machine.api)
                with open(pin_file, "wb") as f:
                    f.write(machine.pin.encode())
            except Exception as e:
                machine.lastError = str(e)
                machine.go_to_state('reset')

        # # disconnect from LTE connection before accessing SIM application
        # # (this is only necessary if we are connected via LTE)
        # if isinstance(machine.connection, NB_IoT):
        #     log.info("\tdisconnecting")
        #     machine.connection.disconnect()  # todo, check the necessity of this

        # initialise ubirch SIM protocol
        log.info("++ initializing ubirch SIM protocol")
        try:
            machine.sim = ElevateSim(lte=machine.lte, at_debug=machine.debug)
        except Exception as e:
            machine.lastError = str(e)
            machine.go_to_state('reset')

        # unlock SIM
        try:
            machine.sim.sim_auth(machine.pin)
        except Exception as e:
            log.exception(str(e))
            # pycom_machine.reset() # TODO check this again, what to do, if PN is unknown
            # if PIN is invalid, there is nothing we can do -> block
            if isinstance(e, ValueError):
                log.critical("PIN is invalid, can't continue")
                while True:
                    wdt.feed()  # avert reset from watchdog   TODO check this, do not want to be stuck here
                    machine.breath.set_color(COLOR_SIM_FAIL)
            else:
                pycom_machine.reset()

        # get UUID from SIM
        machine.key_name = "ukey"
        try:
            machine.uuid = machine.sim.get_uuid(machine.key_name)
        except Exception as e:
            log.exception(str(e))
        log.info("UUID: %s", str(machine.uuid))

        # send a X.509 Certificate Signing Request for the public key to the ubirch identity service (once)
        csr_file = "csr_{}_{}.der".format(machine.uuid, machine.api.env)
        if csr_file not in uos.listdir():
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

        except Exception as e:
            log.exception(str(e))
            pycom_machine.reset() # todo check for alternatives to RESET
            return False
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

        rssi, ber = machine.sim.get_signal_quality(machine.debug)
        cops = machine.sim.get_network_stats(machine.debug)  # TODO this is not yet decyphered
        event = ({'cellSignalPower': {'value': rssi},
                  'cellSignalQuality': {'value': ber},
                  'cellTechnology': {'value': cops},
                  'hardwareVersion':{'value': '0.9.0'},
                  'firmwareVersion':{'value': '0.9.1'},
                  'resetCause':{'value':RESET_REASON}})

        _send_event(machine, event, time.time())

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.enter_timestamp + STANDARD_DURATION_MS:
                machine.go_to_state('waitingForOvershoot')


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
        State.enter(self, machine)
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
                    return

            now = time.ticks_ms()
            if now >= self.enter_timestamp + machine.IntervalForDetectingInactivityMs:
                machine.go_to_state('inactive')


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
        machine.timerActivity = time.time() # todo check this unit
        machine.breath.set_color(LED_GREEN)
        machine.intervalForInactivityEventMs = machine.FirstIntervalForInactivityEventMs
        log.debug("SPEED: {}{}".format(machine.sensor.speed_max, machine.sensor.speed_min))

        event = ({'isWorking':{'value': True}})
        _send_event(machine, event, time.time())

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
            if now >= self.enter_timestamp + machine.OvershotDetectionPauseIntervalMs:
                machine.go_to_state('waitingForOvershoot')


class StateInactive(State):  # todo check what happens in original code
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

        machine.intervalForInactivityEventMs = \
            machine.intervalForInactivityEventMs * \
            machine.ExponentialBackoffFactorForInactivityEvent

        self.new_log_level, self.new_state = _get_state_from_backend(machine)
        log.info("new LEVEL: ({}) new STATE:({})".format(self.new_log_level, self.new_state))
        log.debug("[Core] Increased interval for inactivity events to {}".format(machine.intervalForInactivityEventMs))

    def exit(self, machine):
        State.exit(self, machine)
        machine.timerInactivity = time.time() # todo check if this is the correct time

    def update(self, machine):
        if State.update(self, machine):
            if machine.sensor.trigger:
                machine.sensor.calc_speed()
                if machine.speed() > g_THRESHOLD:
                    machine.go_to_state('measuringPaused')
                    return

            now = time.ticks_ms()
            if now >= self.enter_timestamp + machine.intervalForInactivityEventMs:
                machine.go_to_state('inactive')  # todo check if this can be simplified
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
            if now >= self.enter_timestamp + machine.intervalForBlinkingMs:
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
        try: # just build tha in, because of recent error, which caused the controller to be BRICKED TODO: check this again
            State.enter(self, machine)
            machine.breath.set_color(LED_RED)

            if machine.lastError:
                log.error("[Core] Last error: {}".format(machine.lastError))
            else:
                log.error("[Core] Unknown error.")

            machine.sim.deinit()

        finally:
            pycom_machine.reset()
            # todo maybe try to send the error to backend, but only once, wait for long time?

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
            pycom_machine.reset()

    def exit(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass

    def update(self, machine):
        """ This is intentionally left empty, because this point should never be entered"""
        pass

################################################################################

def _send_event(machine, event: dict, current_time: float):    # todo handle errors differently
    """
    # Send the data to elevate and the UPP to ubirch # TODO, make this configurable
    :param machine: state machine, which provides the connection and the ubirch protocol
    :param event: name of the event to send
    :param current_time: time variable
    :return:
    """
    print("SENDING")

    # make the elevate data package
    elevate_data = {
        'properties.variables': {}
    }
    elevate_data['properties.variables'].update(event)
    elevate_data['properties.variables'].update({'ts':{'v': current_time}})
    # sort and trim the data and make a json from it
    elevate_serialized = serialize_json(elevate_data)
    log.debug("ELEVATE RAW: {}".format(json.dumps(elevate_data)))
    log.debug("ELEVATE SER: {}".format(elevate_serialized))
    log.debug("ELEVATE SER DEC: {}".format(elevate_serialized.decode()))

    # unlock SIM TODO check if this is really necessary. sometimes it is, but maybe this can be solved differently.
    try:
        machine.sim.sim_auth(machine.pin)
    except Exception as e:
        machine.lastError = str(e)
        machine.go_to_state('reset')
    # seal the data message (data message will be hashed and inserted into UPP as payload by SIM card)
    try:
        print("++ creating UPP")
        upp = machine.sim.message_chained(machine.key_name, elevate_serialized, hash_before_sign=True)
        print("\tUPP: {}\n".format(ubinascii.hexlify(upp).decode()))
        message_hash = get_upp_payload(upp)
        print("\tdata message hash: {}".format(ubinascii.b2a_base64(message_hash).decode()))
    except Exception as e:
        machine.lastError = str(e)
        machine.go_to_state('reset')


    try:
        # send data message to data service, with reconnects/modem resets if necessary
        log.debug("++ sending elevate")
        machine.connection.ensure_connection()
        try:
            status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                     machine.elevate_api.send_data, machine.uuid,
                                                     json.dumps(elevate_data))
            log.debug("RESPONSE: {}".format(content))
        except Exception as e:
            machine.lastError = str(e)
            machine.go_to_state('reset')

        # send UPP to the ubirch authentication service to be anchored to the blockchain
        print("++ sending UPP")
        machine.connection.ensure_connection()
        try:
            status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                     machine.api.send_upp, machine.uuid, upp)
        except Exception as e:
            machine.lastError = str(e)
            machine.go_to_state('reset')

        # communication worked in general, now check server response
        if not 200 <= status_code < 300:
            raise Exception("backend (UPP) returned error: ({}) {}".format(status_code,
                                                                           ubinascii.hexlify(content).decode()))
    except Exception as e:
        log.exception(str(e))
    try:
        log.debug("NIOMON:({}) {}".format(status_code, ubinascii.hexlify(content).decode()))
    except Exception as e:
        log.exception(repr(e))


def _get_state_from_backend(machine):
    """
    Get the current state and log level from the elevate backend
    :param machine: state machine, providing the connection
    :return: log level and new state
    """
    # machine.disable_interrupt()
    print("GET THE STATE")
    level = ""
    state = ""

    # send data message to data service, with reconnects/modem resets if necessary
    try:
        machine.connection.ensure_connection()
        log.debug("++ getting elevate state")
        try:
            status_code, level, state = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                          machine.elevate_api.get_state, machine.uuid, '')
            log.debug("ELEVATE: {} {}".format(level, state))
        except Exception as e:
            log.exception(str(e))
            machine.go_to_state('error')
            # pycom_machine.reset()
            # communication worked in general, now check server response
            if not 200 <= status_code < 300:
                raise Exception("backend (ELEVATE) returned error: ({})".format(status_code))
    except Exception as e:
        log.exception(str(e))
    return level, state


#         'isWorking'#(Bool) DONE
#         'lastActivityOn' #(date) todo
#         'lastLogContent' #(String) todo
#         'firmwareVersion'# (String) todo currently fixed string
#         'hardwareVersion'# (String) todo currently fixed string
#         'cellSignalPower'# (Number) DONE
#         'cellSignalQuality'# (Number) DONE
#         'cellOperator'# (String) todo figure out
#         'cellTechnology'# (String) todo currently COPS
#         'cellGlobalIdentity'# (String) todo figure out
#         'cellDeviceIdentity'# (String) todo figure out
#         'ping'# (Number) todo
#         'totalAvailableMemory'# (Number) todo
#         'usedMemory'# (Number) todo
#         'lastError' # (string)
#         'resetCause' # (string) DONE

def _log_switcher(level: str):
    """
    Logging level switcher for handling different logging levels from backend.
    :param level: ne logging level from backend
    :return: translated logging level for logger
    """
    switcher = {
        'error': logging.ERROR,
        'warning': logging.WARNING,
        'info': logging.INFO,
        'debug': logging.DEBUG
    }
    return switcher.get(level, logging.INFO)


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
if RESET_REASON == 'Deepsleep':
    COMING_FROM_DEEPSLEEP = True
else:
    COMING_FROM_DEEPSLEEP = False

# mount SD card if there is one
print("++ mounting SD")
SD_CARD_MOUNTED = mount_sd()
if SD_CARD_MOUNTED:
    print("\tSD card mounted")
else:
    print("\tno SD card found")

