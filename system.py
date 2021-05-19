import ubinascii
import ujson as json
from network import LTE

from lib.config import *
from lib.connection import get_connection, NB_IoT
from lib.elevate_api import ElevateAPI
from lib.ubirch import SimProtocol, UbirchAPI

import lib.logging as logging
from lib.helpers import *
from lib.led_breath import LedBreath
from lib.modem import Modem
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
        self.config = None

        #### SIM ####
        self.sim = None
        self.sim_pin = None
        self.key_name = "ukey"
        self.sim_imsi = None

        #### Connection ####
        self.connection = None
        self.lte = None
        self.modem = None
        self.failed_sends = 0

        #### uBirch Protocol ####
        self.uBirch_disable = False
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

    def init_lte_modem(self):
        """ initialise the LTE modem """
        try:
            # check if self.lte is already initialised/initialise it
            if not self.lte:
                self.lte = LTE()
            if not self.modem:
                self.modem = Modem(self.lte)

            # reset the LTE modem (ensure that it is in a usable state)
            log.info("Resetting the LTE modem")
            self.modem.reset()
            log.info("Done resetting the LTE modem")

            # get/log the IMSI
            log.info("Reading the IMSI from the SIM")
            self.sim_imsi = self.modem.get_imsi()
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
            self.config = load_config()
            self.debug = self.config['debug']
            self.connection = get_connection(self.lte, self.config)
            # configure connection timeouts according to config
            if isinstance(self.connection, NB_IoT):
                self.connection.setattachtimeout(self.config["nbiot_extended_attach_timeout"])
                self.connection.setconnecttimeout(self.config["nbiot_extended_connect_timeout"])
            self.elevate_api = ElevateAPI(self.config)
            self.uBirch_api = UbirchAPI(self.config)
        except Exception as e:
            log.exception("Failed to load the configuration: %s" % str(e))

            # idle until the watchdog resets the system
            while True:
                machine.idle()

    def load_sim_pin(self):
        """ load the SIM pin from flash or the backend + save it """
        pin_file = self.sim_imsi + ".bin"

        self.sim_pin = get_pin_from_flash(pin_file, self.sim_imsi)

        # check if a pin was loaded
        if self.sim_pin is None:
            try:
                # ensure that there it a connection
                self.connection.ensure_connection()
            except Exception as e:
                raise(Exception("Error ensuring a connection for SIM pin bootstrapping: " + str(e)))

            try:
                # start the bootstraping process/get the pin
                self.sim_pin = bootstrap(self.sim_imsi, self.uBirch_api)
            except Exception as e:
                raise(Exception("Error getting the pin: " + str(e)))

            # write the pin to flash
            with open(pin_file, "wb") as f:
                f.write(self.sim_pin.encode())

    def init_sim_proto(self):
        """ initialise the uBirch-Protocol using the SIM """
        # initialise the protocol instance
        try:
            self.sim = SimProtocol(modem=self.modem, at_debug=self.debug)
            self.sim.sim_pin = self.sim_pin # TODO this might be temporary
        except Exception as e:
            raise(Exception("Error initializing the SimProtocol (ElevateSim)" + str(e)))

        # unlock the SIM
        try:
            self.sim.init()
        except Exception as e:
            # if PIN is invalid, there is nothing we can do -> block
            if isinstance(e, ValueError):
                self.led_breath.set_color(COLOR_SIM_FAIL)
                log.critical("PIN is invalid, disabling uBirch-functionality")

                # create an emergency event and try to send it
                event = ({
                    'properties.variables.lastError': {
                        'value': 'PIN is invalid, disabling uBirch-functionality',
                        'sentAt': formated_time()
                    }
                })

                try:
                    self.send_emergency_event(event)
                except Exception as e:
                    raise(Exception("Error informing elevate backend about invalid SIM Pin: " + str(e)))

                # disable uBirching
                self.uBirch_disable = True

                # delete the loaded - invalid - PIN from flash
                del_pin_from_flash(self.sim_imsi + ".bin")
            else:
                # pass the exception back to the caller -> into the state machine
                raise(Exception("Failed to unlock the uBirch applet on the SIM: " + str(e)))

        # read the UUID of the SIM
        if not self.uBirch_disable:
            try:
                self.uBirch_uuid = self.sim.get_uuid(self.key_name)
            except Exception as e:
                raise(Exception("Error getting the UUID: " + str(e)))

            log.info("UUID: %s" % str(self.uBirch_uuid))

    def hard_reset(self):
        """ hard-resets the device by telling the Pysense board to turn the power off/on """
        self.sensor.pysense.reset_cmd()
        raise Exception("hard reset failed")

    def get_movement(self):
        """
        Getter for the movement/overshoot of the filtered sensor data.
        :return: bool overshoot flag, which is set, if filtered data is beyond the threshold.
        """
        return self.sensor.overshoot

    def poll_sensors(self):
        """ Poll the current temperature and altitude sensor values. """
        self.sensor.poll_sensors()

    def get_temperature(self):
        """ Get the stored temperature from the sensor. """
        return self.sensor.temperature

    def get_altitude(self):
        """ Get stored altitude value from sensor. """
        return self.sensor.altitude

    def get_speed_max(self):
        """ Get the current maximum speed from the filtered sensor. """
        return self.sensor.speed_max

    def get_speed_min(self):
        """ Get the current minimum speed from the filtered sensor. """
        return self.sensor.speed_min

    def send_event(self, event: dict, ubirching: bool = False, debug: bool = True):

        """
        Send the data to eevate and the UPP to uBirch
        :param event: name of the event to send
        :param ubirching: enable/disable sending of UPPs to uBirch
        :param debug: for extra debugging outputs of messages
        """
        # local variable, which decides if ubirch operations are executed
        _ubirching = ubirching and not self.uBirch_disable

        events = list()
        upps = list()

        try:
            # check if the event should be uBirched
            if _ubirching:
                serialized_event = serialize_json(event)

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
                    status_code, content = send_backend_data(self.sim, self.modem, self.connection,
                                                             self.elevate_api.send_data, self.uBirch_uuid,
                                                             events[0])
                    log.debug("RESPONSE: {}".format(content))

                    if not 200 <= status_code < 300:
                        log.error("BACKEND RESP {}: {}".format(status_code, content))
                        return
                    else:
                        # event was sent successfully and can be removed from backlog
                        events.pop(0)

            except:
                return

            # send UPPs
            self.connection.ensure_connection()

            try:
                if _ubirching:
                    while len(upps) > 0:
                        if debug:
                            log.debug("Sending UPP: {}".format(upps[0]))

                        # send UPP to the ubirch authentication service to be anchored to the blockchain
                        status_code, content = send_backend_data(self.sim, self.modem, self.connection,
                                                                 self.uBirch_api.send_upp, self.uBirch_uuid,
                                                                 ubinascii.unhexlify(upps[0]))
                        try:
                            log.debug("NIOMON RESPONSE: ({}) {}".format(status_code, "" if status_code == 200
                                                                        else ubinascii.hexlify(content).decode()))
                        except:
                            # this is only exception handling in case the content can not be decyphered
                            pass
                        # communication worked in general, now check server response
                        if not 200 <= status_code < 300 and not status_code == 409:
                            log.error("NIOMON RESP {}".format(status_code))
                        else:
                            # UPP was sent successfully and can be removed from backlog
                            upps.pop(0)
                else:
                    pass
            except:
                # sending failed, terminate
                return

        except Exception as e:
            # if too many communications fail, reset the system
            self.failed_sends += 1
            if self.failed_sends > 3:
                raise(Exception("Failed to send a message within 3 tries: " + str(e)))
            else:
                log.exception(str(e))

        finally:
            # make sure to store the unsent events and UPPs to the backlog files and disconnect connection
            write_backlog(events, EVENT_BACKLOG_FILE, BACKLOG_MAX_LEN)
            if _ubirching:
                write_backlog(upps, UPP_BACKLOG_FILE, BACKLOG_MAX_LEN)
            self.connection.disconnect()

        return

    def send_emergency_event(self, event):
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
            _, content = send_backend_data(self.sim, self.modem, self.connection,
                                           self.elevate_api.send_data, self.uBirch_uuid,
                                           event_string)
            log.debug("RESPONSE: {}".format(content))

        except Exception as e:
            raise(Exception("Failed to send an emergency event: " + str(e)))
        finally:
            self.connection.disconnect()

        return

    def get_state_from_backend(self):
        """
        Get the current state and log level from the elevate backend
        :return: log level and new state or ("", "") in case of an error
        """
        level = ""      # level will be handled from helpers.translate_backend_log_level()
        state = ""      # state will be handled from helpers.translate_backend_state_name()

        # send data message to data service, with reconnects/modem resets if necessary
        try:
            self.connection.ensure_connection()
            status_code, level, state = send_backend_data(self.sim, self.modem, self.connection,
                                                          self.elevate_api.get_state, self.uBirch_uuid, '')
            # communication worked in general, now check server response
            if not 200 <= status_code < 300:
                log.error("Elevate backend returned HTTP error code {}".format(status_code))
        except Exception as e:
            # only log the exception - error detection is done by the state machine when looking up the level/state
            log.exception(str(e))
        finally:
            self.connection.disconnect()

        return level, state
