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


STANDARD_DURATION = 500

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
        self.lte = LTE()
        self.breath = LedBreath()

        # set all necessary time values
        self.IntervalForDetectingInactivityMs = 60000
        self.OvershotDetectionPauseIntervalMs = 60000
        self.FirstIntervalForInactivityEventMs = 60000
        self.ExponentialBackoffFactorForInactivityEvent = 2
        self.intervalForInactivityEventMs = self.FirstIntervalForInactivityEventMs
        self.intervalForErrorEventMs = 15000

        self.timerActivity = 0.0
        self.timerLastActivity = 0.0
        self.timerInactivity = 0.0

        log.info("[Core] Initializing magic... ✨ ")
        log.info("[Core] Hello, I am %s",  ubinascii.hexlify(pycom_machine.unique_id()))

        try:
            # reset modem on any non-normal loop (modem might be in a strange state)
            if not COMING_FROM_DEEPSLEEP:
                log.warning("++ not coming from sleep, resetting modem")
                reset_modem(self.lte)

            log.info("++ getting IMSI")
            imsi = get_imsi(self.lte)
            log.info("IMSI: " + imsi)
        except Exception as e:
            log.exception("\tERROR setting up modem (%s)", str(e))
            while True:
                pycom_machine.idle()

        # write IMSI to SD card
        if not COMING_FROM_DEEPSLEEP and SD_CARD_MOUNTED: store_imsi(imsi)
        # load configuration, blocks in case of failure
        log.info("++ loading config")
        try:
            self.cfg = load_config(sd_card_mounted=SD_CARD_MOUNTED)

            self.debug = self.cfg['debug']  # set debug level
            # if self.debug: print("\t" + repr(self.cfg))

            self.connection = get_connection(self.lte, self.cfg)  # initialize connection object depending on config
            self.api = ubirch.API(self.cfg)  # set up API for backend communication
        except Exception as e:
            log.exception("\tERROR loading configuration (%s)", str(e))
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
        self.pin = get_pin_from_flash(pin_file, imsi)
        if self.pin is None:
            try:
                self.connection.connect()
            except Exception as e:
                log.exception(str(e))
                pycom_machine.reset()

            try:
                self.pin = bootstrap(imsi, self.api)
                with open(pin_file, "wb") as f:
                    f.write(self.pin.encode())
            except Exception as e:
                log.exception(str(e))
                pycom_machine.reset()

        # disconnect from LTE connection before accessing SIM application
        # (this is only necessary if we are connected via LTE)
        if isinstance(self.connection, NB_IoT):
            log.info("\tdisconnecting")
            self.connection.disconnect()

        # initialise ubirch SIM protocol
        log.info("++ initializing ubirch SIM protocol")
        try:
            self.sim = ElevateSim(lte=self.lte, at_debug=self.debug)
        except Exception as e:
            log.exception(str(e))
            pycom_machine.reset()

        # unlock SIM
        try:
            self.sim.sim_auth(self.pin)
        except Exception as e:
            log.exception(str(e))
            # pycom_machine.reset() # TODO check this again, what to do, if PN is unknown
            # if PIN is invalid, there is nothing we can do -> block
            if isinstance(e, ValueError):
                log.critical("PIN is invalid, can't continue")
                while True:
                    wdt.feed()  # avert reset from watchdog   TODO check this, do not want to be stuck here
                    self.breath.set_color(COLOR_SIM_FAIL)
            else:
                pycom_machine.reset()

        # get UUID from SIM
        self.key_name = "ukey"
        try:
            self.uuid = self.sim.get_uuid(self.key_name)
        except Exception as e:
            log.exception(str(e))
        log.info("UUID: %s", str(self.uuid))

        # send a X.509 Certificate Signing Request for the public key to the ubirch identity service (once)
        csr_file = "csr_{}_{}.der".format(self.uuid, self.api.env)
        if csr_file not in uos.listdir():
            try:
                self.connection.connect()
            except Exception as e:
                log.exception(str(e))

            try:
                csr = submit_csr(self.key_name, self.cfg["CSR_country"], self.cfg["CSR_organization"], self.sim, self.api)
                with open(csr_file, "wb") as f:
                    f.write(csr)
            except Exception as e:
                log.exception(str(e))

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
        Enable the Accelerometer Interrupt for fifo buffer. # TODO currently not in use, mybe unnecessary.
        """
        self.sensor.pysense.accelerometer.restart_fifo()
        self.sensor.pysense.accelerometer.enable_fifo_interrupt(self.sensor.accelerometer_interrupt_cb)

    def disable_interrupt(self):
        """
        Disable the Accelerometer Interrupt for fifo interrupt. # TODO currently not used, maybe not necessary.
        """
        self.sensor.pysense.accelerometer.enable_fifo_interrupt(handler=None)


################################################################################
# States

class State(object):
    """
    Abstract Parent State Class.
    """
    def __init__(self):
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
    Initialize the System. # TODO, not really implemented yet, currently not used.
    """
    def __init__(self):
        super().__init__()
        self._cellular_wait_time = 0

    @property
    def name(self):
        return 'initSystem'

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
            machine.connection.connect()
            enable_time_sync()
            log.info("\twaiting for time sync")
            wait_for_sync(print_dots=True)

        except Exception as e:
            log.exception(str(e))
            pycom_machine.reset() # todo check for alternatives to RESET
            # error_handler.log(e, COLOR_INET_FAIL, reset=True)  # todo check reset
            return False
        return True

    def update(self, machine):
        State.update(self, machine)
        if self._connect(machine):
            machine.go_to_state('sendingDiagnostics')


class StateSendingDiagnostics(State):
    """
    Sending Version Diagnostics to the backend.
    TODO figure out, what to do here
    """
    def __init__(self):
        super().__init__()
        self._version_wait_time = 0

    @property
    def name(self):
        return 'sendingDiagnostics'

    def enter(self, machine):
        State.enter(self, machine)
        self._version_wait_time = time.ticks_ms()
        machine.breath.set_color(LED_YELLOW)

        rssi, ber = machine.sim.get_signal_quality(machine.debug)
        cops = machine.sim.get_network_stats(machine.debug)  # TODO this is not yet decyphered
        event = ({'cellSignalPower': {'value': rssi},
                  'cellSignalQuality': {'value': ber},
                  'cellTechnology': {'value': cops},
                  'hardwareVersion':{'value': '0.9.0'},
                  'firmwareVersion':{'value': '0.9.1'}})

        _send_event(machine, event, time.time())

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self._version_wait_time + STANDARD_DURATION:
                machine.go_to_state('waitingForOvershoot')


class StateWaitingForOvershot(State):
    """
    Wait here for acceleration to overshoot the threshold,
    or until waiting time was exceeded.
    """
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

    def enter(self, machine):
        State.enter(self, machine)
        self._measuring_paused_time = time.ticks_ms()
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
            if now >= self._measuring_paused_time + machine.OvershotDetectionPauseIntervalMs:
                machine.go_to_state('waitingForOvershoot')


class StateInactive(State):
    """
    Inactive State,
    at entering, the current state is get from the backend
    and the waiting interval is increased
    """
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

        level, state = _get_state(machine)
        log.info("new LEVEL: ({}) new STATE:({})".format(level, state))
        self._adjust_level_state(machine, level, state)
        machine.intervalForInactivityEventMs = \
            machine.intervalForInactivityEventMs * \
            machine.ExponentialBackoffFactorForInactivityEvent
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

            now = time.ticks_ms()
            if now >= self._inactive_time + machine.intervalForInactivityEventMs:
                machine.go_to_state('inactive')  # todo check if this can be simplified

    def _adjust_level_state(self, machine, level, state):
        """
        Adjust the logging level and the current state
        :param machine: state machine holding this state
        :param level: new logging level to adjust
        :param state: new sensor state to adjust
        """
        log.setLevel(_log_switcher(level))
        machine.go_to_state(_state_switcher(state))


# noinspection MicroPythonRequirements
class StateBlinking(State):
    """
    Blinking State is a special state, with a bright and fast led blinking
    to make the sensor more visible. This state is intended to identify
    sensors in the field.
    After some time, the state switches back to inactive.
    """
    def __init__(self):
        super().__init__()
        self._waiting_time = 0

    @property
    def name(self):
        return 'blinking'

    def enter(self, machine):
        State.enter(self, machine)
        self._waiting_time = time.ticks_ms()
        machine.breath.set_blinking()

    def exit(self, machine):
        State.exit(self, machine)
        machine.breath.reset_blinking()

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self._waiting_time + 10000: # todo set a proper time value here
                machine.go_to_state('inactive')


class StateError(State):
    """
    Error State, which is entered, whenever a critical error occurs.
    This state should end with TODO has to be defined.
    """
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
        if machine.lastError:   # TODO currently lastError is not in use.
            log.error("[Core] Last error: {}".format(machine.lastError))
        else:
            log.error("[Core] Unknown error.")
        
        machine.sim.deinit()
        pycom_machine.reset()

    def exit(self, machine):
        State.exit(self, machine)
        machine.lastError = None

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self._error_time + machine.intervalForErrorEventMs:
                machine.go_to_state('sendingCellularDiagnostics')  # TODO this might not be the correct state to return to


################################################################################

def _send_event(machine, event: dict, current_time: float):
    """
    # Send the data to elevate and the UPP to ubirch # TODO, make this configurable
    :param machine: state machine, which provides the connection and the ubirch protocol
    :param event: name of the event to send
    :param current_time: time variable TODO maybe not necessary
    :return:
    """
    print("SENDING")
    # get the time TODO, this format is currently not recognized by the backend, maybe simple timestamp is better.
    # t = time.gmtime(current_time)  # (1970, 1, 1, 0, 0, 15, 3, 1)
    # iso8601_fmt = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z"  # "2020-08-24T14:23:50Z"
    # iso8601_time = iso8601_fmt.format(t[0], t[1], t[2], t[3], t[4], t[5])

    # make the elevate data package
    elevate_data = {
        'properties.variables': {}
    }
    elevate_data['properties.variables'].update(event)
    elevate_data['properties.variables'].update({'ts':{'v': current_time}})

    elevate_serialized = serialize_json(elevate_data)
    log.debug("ELEVATE RAW: {}".format(json.dumps(elevate_data)))

    # unlock SIM TODO check if this is really necessary. sometimes it is, but maybe this can be solved differently.
    try:
        machine.sim.sim_auth(machine.pin)
    except Exception as e:
        log.exception(str(e))
        machine.go_to_state('error')
    # seal the data message (data message will be hashed and inserted into UPP as payload by SIM card)
    try:
        print("++ creating UPP")
        upp = machine.sim.message_chained(machine.key_name, elevate_serialized, hash_before_sign=True)
        print("\tUPP: {}\n".format(ubinascii.hexlify(upp).decode()))
    except Exception as e:
        log.exception(str(e))
        machine.go_to_state('error')

    machine.connection.connect()

    try:
        # send data message to data service, with reconnects/modem resets if necessary
        log.debug("++ sending elevate")
        try:
            status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                     machine.elevate_api.send_data, machine.uuid,
                                                     json.dumps(elevate_data))
            log.debug("RESPONSE: {}".format(content))
        except Exception as e:
            log.exception(str(e))
            machine.go_to_state('error')

        # send UPP to the ubirch authentication service to be anchored to the blockchain
        print("++ sending UPP")
        try:
            status_code, content = send_backend_data(machine.sim, machine.lte, machine.connection,
                                                     machine.api.send_upp, machine.uuid, upp)
        except Exception as e:
            log.exception(str(e))
            machine.go_to_state('error')
            # pycom_machine.reset()


        # communication worked in general, now check server response
        if not 200 <= status_code < 300:
            raise Exception("backend (UPP) returned error: ({}) {}".format(status_code,
                                                                           ubinascii.hexlify(content).decode()))
    except Exception as e:
        log.exception(str(e))
#     print("T:{} V:{}".format(type(content), repr(content)))
    try:
        log.debug("NIOMON:({}) {}".format(status_code, ubinascii.hexlify(content).decode()))
    except Exception as e:
        log.exception(repr(e))


def _get_state(machine):
    """
    Get the current state and log level from the elevate backend
    :param machine: state machine, providing the connection
    :return: log level and new state
    """
    # machine.disable_interrupt()
    print("GET THE STATE")
    level = ""
    state = ""
    machine.connection.connect()

    # send data message to data service, with reconnects/modem resets if necessary
    try:
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
#         'batteryCharge'# (Number) todo NO BATTERY included
#         'stateOfCharge'# (String) todo NO BATTERY included

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
        'custom3': 'waitingForOvershoot'
    }
    return switcher.get(state, 'error')

# TODO cleanup below

# TODO: do this anyway
COMING_FROM_DEEPSLEEP = (pycom_machine.reset_cause() == pycom_machine.DEEPSLEEP_RESET)

# mount SD card if there is one
print("++ mounting SD")
SD_CARD_MOUNTED = mount_sd()
if SD_CARD_MOUNTED:
    print("\tSD card mounted")
else:
    print("\tno SD card found")

# max_file_size_kb = 10240 if SD_CARD_MOUNTED else 20
# error_handler = ErrorHandler(file_logging_enabled=True, max_file_size_kb=max_file_size_kb,
#                              sd_card=SD_CARD_MOUNTED)
