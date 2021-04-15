import ubinascii
from network import LTE
import ujson as json

from lib.helpers import *
from lib.config import *
from lib.connection import get_connection, NB_IoT
from lib.modem import get_imsi
from sensor import MovementSensor
from lib.elevate_api import ElevateAPI
from lib.elevate_sim import ElevateSim
import ubinascii
import ujson as json
from network import LTE

from lib.config import *
from lib.connection import get_connection, NB_IoT
from lib.elevate_api import ElevateAPI
from lib.elevate_sim import ElevateSim
from lib.helpers import *
from lib.modem import get_imsi
from sensor import MovementSensor

# backlog constants
EVENT_BACKLOG_FILE = "event_backlog.txt"
UPP_BACKLOG_FILE = "upp_backlog.txt"
BACKLOG_MAX_LEN = 10  # max number of events / UPPs in the backlogs

# get the global logger
log = logging.getLogger()


class System:
    """ manages system tasks  """

    def __init__(self):
        self.debug = None

        #### SIM ####
        self.sim = None
        self.sim_pin = None
        self.key_name = "ukey"
        self.sim_imsi = None

        #### Connection ####
        self.connection = None
        self.lte = None
        self.failed_sends = 0

        #### uBirch Protocol ####
        self.uBirch_api = None
        self.uBirch_uuid = None

        #### Elevate API ####
        self.elevate_api = None

        #### Sensor ####
        self.sensor = MovementSensor()

        #### LED Breath ####
        self.led_breath = LedBreath()

        ## Initialise all "modules" ####
        self.init_lte_modem()
        self.load_config()
        self.load_sim_pin()
        self.init_sim_proto()
        self.get_csr()

    def init_lte_modem(self):
        """ initialise the LTE modem """
        try:
            # check if self.lte is already initialised/initialise it
            if not self.lte:
                self.lte = LTE()

            # reset the LTE modem (ensure that it is in a usable state)
            log.warning("Resetting the LTE modem")  # TODO why is this a warning and not info?
            reset_modem(self.lte)
            log.warning("Done resetting the LTE modem")

            # get/log the IMSI
            log.info("Reading the IMSI from the SIM")
            self.sim_imsi = get_imsi(self.lte)
            log.info("SIM IMSI: %s" % str(self.sim_imsi))
        except Exception as e:
            log.exception("Failed to set up the LTE Modem: %s" % str(e))

            # idle until the watchdog resets the system
            while True:
                machine.idle()

        return

    def load_config(self):
        """ load the application config """
        try:
            log.info("Loading the configuration")

            # load the conifguration and init the connection + uBirch API
            # TODO shouldn't config loading and conection setup be separated?
            self.config = load_config()
            self.debug = self.config['debug']
            self.connection = get_connection(self.lte, self.config)
            self.uBirch_api = ubirch.API(self.config)
        except Exception as e:
            log.exception("Failed to load the configuration: %s" % str(e))

            # generate and send an emergency event
            event = ({
                'properties.variables.lastError': {'value': str(e), 'sentAt': formated_time()}
            })

            self.send_emergency_event(event)

            # idle until the watchdog resets the system
            while True:
                machine.idle()

        # create an instance of the elevate API, which needs the configuration
        self.elevate_api = ElevateAPI(self.config)

        # configure connection timeouts according to config
        if isinstance(self.connection, NB_IoT):
            self.connection.setattachtimeout(self.config["nbiot_extended_attach_timeout"])
            self.connection.setconnecttimeout(self.config["nbiot_extended_connect_timeout"])

    def load_sim_pin(self):
        """ load the SIM pin from flash or the backend + save it """
        pin_file = self.sim_imsi + ".bin"

        self.sim_pin = get_pin_from_flash(pin_file, self.sim_imsi)

        # check if a pin was loaded
        if self.sim_pin == None:
            # TODO exception handling not needed, since the caller inside the State Machine has to handle it?
            # ensure that there it a connection
            self.connection.ensure_connection()

            # start the bootstraping process/get the pin
            self.sim_pin = bootstrap(self.sim_imsi, self.uBirch_api)

            # write the pin to flash
            with open(pin_file, "wb") as f:
                f.write(self.sim_pin.encode())

    def init_sim_proto(self):
        """ initialise the uBirch-Protocol using the SIM """
        # initialise the protocl instance
        self.sim = ElevateSim(lte=self.lte, at_debug=self.debug)

        # unlock the SIM
        try:
            self.sim.sim_auth(self.sim_pin)
        except Exception as e:
            # if PIN is invalid, there is nothing we can do -> block
            if isinstance(e, ValueError):
                self.led_breath.set_color(COLOR_SIM_FAIL)
                log.critical("PIN is invalid, can't continue")

                # create an emergency event and try to send it
                event = ({
                    'properties.variables.lastError': {'value': 'PIN is invalid, can\'t continue',
                                                       'sentAt': formated_time()}
                })
                self.send_emergency_event(
                    event)  # CHECK: if transitions to an error state are implemented for SysInit, it might be better to send the
                #  emergency message in the error state (?)

                # while True:
                #     machine.wdt.feed()  # avert reset from watchdog   TODO check this, do not want to be stuck here
                #     machine.breath.update()
                time.sleep(180)
                self.hard_reset()  # CHECK: if the PIN is invalid, it will probably still be on next boot, and thus the SIM will become unusable after 3 resets
            else:
                # pass the exception back to the caller -> into the state machine
                raise (e)

        # read the UUID of the SIM
        self.uBirch_uuid = self.sim.get_uuid(self.key_name)

        log.info("UUID: %s" % str(self.uBirch_uuid))

    def get_csr(self):
        """ check if we already have the csr and get it if not """
        csr_file = "csr_{}_{}.der".format(self.uBirch_uuid, self.uBirch_api.env)

        # check if the file exists
        if csr_file not in os.listdir():
            try:
                self.connection.ensure_connection()
            except Exception as e:
                log.exception(str(e))
                # CHECK: shouldn't this transition to an error state? With the current implementation the system will transition to next state without sending CSR

            try:
                csr = submit_csr(self.key_name, self.config["CSR_country"], self.config["CSR_organization"],
                                 self.sim, self.uBirch_api)

                # write the csr to flash
                with open(csr_file, "wb") as f:
                    f.write(csr)
            except Exception as e:
                log.exception(str(e))
                # CHECK: shouldn't this transition to an error state? With the current implementation the system will transition to next state without sending CSR

    def hard_reset(self):
        """ hard-resets the device by telling the Pysense board to turn the power off/on """
        time.sleep(1)  # TODO Why?

        self.sensor.pysense.reset_cmd()  # TODO raise error when reaching code after this call?

    def get_movement(self):
        """
        Getter for the movement/overshoot of the filtered sensor data.
        :return: bool overshoot flag, which is set, if filtered data is beyond the threshold.
        """
        return self.sensor.overshoot

    def poll_sensors(self):
        """ Poll the current temperature and altitude sensor values. """
        self.sensor.poll_sensors()

    def send_event(self, event: dict, ubirching: bool = False, debug: bool = True) -> bool:
        """
        Send the data to eevate and the UPP to uBirch
        :param event: name of the event to send
        :param ubirching: enable/disable sending of UPPs to uBirch
        :param debug: for extra debugging outputs of messages
        """

        try:
            # check if the event should be uBirched
            if ubirching == True:
                serialized_event = serialize_json(event)

                # unlock the SIM
                self.sim.sim_auth(self.sim_pin)

                # use the SIM to create the UPP
                log.info("Creating a UPP")
                upp = self.sim.message_chained(self.key_name, serialized_event, hash_before_sign=True)
                log.info("UPP: %s\n" % ubinascii.hexlify(upp).decode())

                # get UPP backlog from flash and add new UPP
                upps = get_backlog(UPP_BACKLOG_FILE)
                upps.append(ubinascii.hexlify(upp).decode())

            # get event backlog from flash and add new event
            events = get_backlog(EVENT_BACKLOG_FILE)
            events.append(json.dumps(event))

            # send events
            self.connection.ensure_connection()

            try:
                while len(events) > 0:
                    if debug:
                        log.debug("Sending event: {}".format(events[0]))

                    # send data message to data service, with reconnects/modem resets if necessary
                    status_code, content = send_backend_data(self.sim, self.lte, self.connection,
                                                             self.elevate_api.send_data, self.uBirch_uuid,
                                                             events[0])
                    log.debug("RESPONSE: {}".format(content))

                    if not 200 <= status_code < 300:
                        log.error("BACKEND RESP {}: {}".format(status_code, content))  # TODO check error log content!
                        write_backlog(events, EVENT_BACKLOG_FILE, BACKLOG_MAX_LEN)
                        if ubirching:
                            write_backlog(upps, UPP_BACKLOG_FILE, BACKLOG_MAX_LEN)
                        self.connection.disconnect()

                        return False
                    else:
                        # event was sent successfully and can be removed from backlog
                        events.pop(0)

            except Exception:
                # sending failed, write unsent messages to flash and terminate
                write_backlog(events, EVENT_BACKLOG_FILE, BACKLOG_MAX_LEN)
                if ubirching:
                    write_backlog(upps, UPP_BACKLOG_FILE, BACKLOG_MAX_LEN)

                return False

            # send UPPs
            self.connection.ensure_connection()

            try:
                if ubirching:
                    while len(upps) > 0:
                        # send UPP to the ubirch authentication service to be anchored to the blockchain
                        status_code, content = send_backend_data(self.sim, self.lte, self.connection,
                                                                 self.uBirch_api.send_upp, self.uBirch_uuid,
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
                return False
            finally:
                write_backlog(events, EVENT_BACKLOG_FILE, BACKLOG_MAX_LEN)
                if ubirching:
                    write_backlog(upps, UPP_BACKLOG_FILE, BACKLOG_MAX_LEN)

        except Exception as e:
            # if too many communications fail, reset the system
            self.failed_sends += 1
            if self.failed_sends > 3:
                log.exception(str(e) + "doing RESET")
                # self.go_to_state('error')  # CHECK: state transistion in global function (outside of state / state machine), might be better to return success/fail and handle transition in the state machine

                return False
            else:
                log.exception(str(e))

        finally:
            self.connection.disconnect()
            # CHECK: I'm not 100% sure, but it might be possible to move the write_backlog calls from above here as
            # this 'finally' block should be executed even if there is a return in the code above (will run just before returning from this function)
            # but might also make code less readable maybe

        return True

    def send_emergency_event(self, event) -> bool:
        """
        Send an emergency event to elevate
        :param event: name of the event to send
        :return:
        """
        # make the elevate data package
        event_string = json.dumps(event)
        log.debug("Sending Elevate HTTP request body: {}".format(event_string))

        try:
            self.connection.ensure_connection()
            # send data message to data service, with reconnects/modem resets if necessary
            _, content = send_backend_data(self.sim, self.lte, self.connection,
                                           self.elevate_api.send_data, self.uBirch_uuid,
                                           event_string)
            log.debug("RESPONSE: {}".format(content))

        except Exception as e:
            log.exception(str(e))

            return False
        finally:
            self.connection.disconnect()

        return True

    def get_state_from_backend(self):
        """
        Get the current state and log level from the elevate backend
        :param machine: state machine, providing the connection
        :return: log level and new state or None, None in case of an error
        """
        level = ""
        state = ""
        # CHECK: TODO: document what the backend might reply, e.g. especially if "empty" state information is possible

        # send data message to data service, with reconnects/modem resets if necessary
        try:
            self.connection.ensure_connection()
            status_code, level, state = send_backend_data(self.sim, self.lte, self.connection,
                                                          self.elevate_api.get_state, self.uBirch_uuid, '')
            # communication worked in general, now check server response
            if not 200 <= status_code < 300:
                log.error("Elevate backend returned HTTP error code {}".format(status_code))
        except Exception as e:
            log.exception(str(e))

            return None, None

        finally:
            self.connection.disconnect()

        return level, state
