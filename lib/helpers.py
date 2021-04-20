import math
import uos as os
import utime as time
from uuid import UUID

import logging
import machine
import pycom
import ubirch
from connection import Connection
from modem import reset_modem
from network import LTE

########
# LED color codes
LED_OFF = 0x000000

# standard brightness: 25% (low-power)
LED_WHITE = 0x202020  # StateConnecting
LED_GREEN = 0x002000  # StateMeasuringPaused
LED_YELLOW = 0x202000  # StateSendingDiagnostics
LED_RED = 0x200000  # StateError
LED_PURPLE = 0x200020  # StateWaitingForOvershoot
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


########
def mount_sd():  # todo check if this is the right place for this function
    try:
        sd = machine.SD()
        try:  # check if sd is already mounted
            os.stat('/sd')
            return True
        except:  # not mounted: continue
            pass
        os.mount(sd, '/sd')
        return True
    except OSError:
        return False


def store_imsi(imsi: str):
    # save imsi to file on SD, SD needs to be mounted
    imsi_file = "imsi.txt"
    if imsi_file not in os.listdir('/sd'):
        print("\twriting IMSI to SD")
        with open('/sd/' + imsi_file, 'w') as f:
            f.write(imsi)


def get_pin_from_flash(pin_file: str, imsi: str) -> str or None:
    if pin_file in os.listdir():
        print("\tloading PIN for " + imsi)
        with open(pin_file, "rb") as f:
            return f.readline().decode()
    else:
        print("\tno PIN found for " + imsi)
        return None


def del_pin_from_flash(pin_file : str) -> bool:
    """ deletes the given pin_file; returns true if found and deleted """
    if pin_file in os.listdir():
        os.remove(pin_file)

        return True
    
    return False


def send_backend_data(sim: ubirch.SimProtocol, lte: LTE, conn: Connection, api_function, uuid, data) -> (int, bytes):
    MAX_MODEM_RESETS = 1  # number of retries with modem reset before giving up
    MAX_RECONNECTS = 1  # number of retries with reconnect before trying a modem reset

    for reset_attempts in range(MAX_MODEM_RESETS + 1):
        # check if this is a retry for reset_attempts
        if reset_attempts > 0:
            print("\tretrying with modem reset")
            sim.deinit()
            reset_modem(lte)  # TODO: should probably be connection.reset_hardware()
            sim.init()
            conn.ensure_connection()

        # try to send multiple times (with reconnect)
        try:
            for send_attempts in range(MAX_RECONNECTS + 1):
                # check if this is a retry for send_attempts
                if send_attempts > 0:
                    print("\tretrying with disconnect/reconnect")
                    conn.disconnect()
                    conn.ensure_connection()
                try:
                    print("\tsending...")
                    return api_function(uuid, data)
                except Exception as e:
                    # TODO: log/print exception?
                    print("\tsending failed: {}".format(e))
                    # (continues to top of send_attempts loop)
            else:
                # all send attempts used up
                raise Exception("all send attempts failed")
        except Exception as e:
            print(repr(e))
            # (continues to top of reset_attempts loop)
    else:
        # all modem resets used up
        raise Exception("could not establish connection to backend")


def bootstrap(imsi: str, api: ubirch.API) -> str:
    """
    Load bootstrap PIN, returns PIN
    """
    print("\tbootstrapping SIM identity " + imsi)
    status_code, content = api.bootstrap_sim_identity(imsi)
    if not 200 <= status_code < 300:
        raise Exception("bootstrapping failed: ({}) {}".format(status_code, str(content)))

    from ujson import loads
    info = loads(content)
    pin = info['pin']

    # sanity check
    try:
        if len(pin) != 4: raise ValueError("len = {}".format(len(pin)))
        int(pin)  # throws ValueError if pin has invalid syntax for integer with base 10
    except ValueError as e:
        raise Exception("bootstrapping returned invalid PIN: {}".format(e))

    return pin


def pack_data_json(uuid: UUID, data: dict) -> bytes:
    """
    Generate a JSON formatted message for the ubirch data service.
    The message contains the device UUID, timestamp and data to ensure unique hash.
    :param uuid: the device UUID
    :param data: the mapped data to be sent to the ubirch data service
    :return: the msgpack formatted message
    """
    # hint for the message format (version)
    MSG_TYPE = 1

    # pack the message
    msg_map = {
        'uuid': str(uuid),
        'msg_type': MSG_TYPE,
        'timestamp': int(time.time()),
        'data': data
    }

    # create a compact sorted rendering of the message to ensure determinism when creating the hash
    # and return serialized message
    return serialize_json(msg_map)


def serialize_json(msg: dict) -> bytes:
    """
    create a compact sorted rendering of a json object since micropython
    implementation of ujson.dumps does not support sorted keys
    :param msg: the json object (dict) to serialize
    :return: the compact sorted rendering
    """
    serialized = "{"
    for key in sorted(msg):
        serialized += "\"{}\":".format(key)
        value = msg[key]
        value_type = type(value)
        if value_type is str:
            serialized += "\"{:s}\"".format(value)
        elif value_type is int:
            serialized += "{:d}".format(value)
        elif isinstance(value, float):
            serialized += "{:.4f}".format(value)  # modified for elevate
        elif value_type is dict:
            serialized += serialize_json(value).decode()
        elif value_type is bool:
            if value == True:
                serialized += "true"
            else:
                serialized += "false"
        elif value is None:
            serialized += "null"
        else:
            raise Exception("unsupported data type {} for serialization in json message".format(value_type))
        serialized += ","
    serialized = serialized.rstrip(",") + "}"  # replace last comma with closing braces
    return serialized.encode()


def get_upp_payload(upp: bytes) -> bytes:
    """
    Get the payload of a Ubirch Protocol Message
    :param upp: received UPP
    :return: payload of the UPP
    """
    if upp[0] == 0x95 and upp[1] == 0x22:  # signed UPP
        payload_start_idx = 23
    elif upp[0] == 0x96 and upp[1] == 0x23:  # chained UPP
        payload_start_idx = 89
    else:
        from ubinascii import hexlify
        raise Exception("!! can't get payload from {} (not a UPP)".format(hexlify(upp).decode()))

    if upp[payload_start_idx - 2] != 0xC4:
        raise Exception("unexpected payload type: {}".format(upp[payload_start_idx - 2]))

    payload_len = upp[payload_start_idx - 1]
    return upp[payload_start_idx:payload_start_idx + payload_len]


class LedBreath(object):
    """
    Class LedBreath lets the RGB LED from the PyCom module breath
    according to the time ticks.
    If the breathing does not work for a while, it means the controller is not running
    """

    def __init__(self):
        self.period = 5000.0
        self.color = 0xFF00FF
        self.brightness = 0xFF
        # storage for backup values, used for blinking method
        self.period_back = self.period
        self.color_back = self.color
        self.brightness_back = self.brightness

    def update(self):
        """
        Update the breathing, means to calculate the new intensity value of the light
        and set the RGB LED accordingly.
        The breathing follows a sinewave with a period of self.period
        and the time is controlled by time.ticks_ms().
        """
        # calculate the intensity
        _intensity = self.brightness / 512.0 * (math.sin(time.ticks_ms() / self.period * 2 * math.pi) + 1)
        if _intensity < 0.1:
            _intensity = 0.1
        # split the color into the RGB components
        _red = self.color >> 16 & 0xFF
        _green = self.color >> 8 & 0xFF
        _blue = self.color & 0xFF
        # combine the intensity and the colors into the new ligh value
        _light = ((int)(_intensity * _red) << 16) + \
                 ((int)(_intensity * _green) << 8) + \
                 ((int)(_intensity * _blue))
        # set the RGBLED to the new value
        pycom.rgbled(_light)

    def set_color(self, color):
        """
        set_color is used to set the color of the RGB LED and update it directly.
        This color will stay until it is changed.
        :param color: is the color value in R,G,B
        """
        self.color = color & 0xFFFFFF
        self.update()

    def set_brightness(self, brightness):
        """
        set_brightness is used to set the brightness of the RGB LED light at maximum.
        The default value is 0xFF
        :param brightness: is the brightness at maximum of the breathing interval
        """
        self.brightness = brightness & 0xFF
        self.update()

    def set_period(self, period_ms):
        """
        set_period is used to set the period [ms] of the RGB LED light breathing.
        The default value is 5000 ms
        :param period_ms: is the period [ms] of the breathing interval
        """
        self.period = period_ms
        self.update()

    def set_blinking(self):
        """
        set_blinking is used to set the LED breathing to a fast a bright mode.
        This Method shall be used to make the sensor visible and recognizable.
        """
        # first backup the old values
        self.period_back = self.period
        self.color_back = self.color
        self.brightness_back = self.brightness
        # now set a bright red color with 1000 ms period
        self.period = 1000
        self.color = 0xFF8010
        self.brightness = 0xFF
        self.update()

    def reset_blinking(self):
        """
        reset_blinking is used to set the LED breathing to a fast a bright mode.
        This Method shall be used to reset the .
        """
        # get back the backuped values
        self.period = self.period_back
        self.color = self.color_back
        self.brightness = self.brightness_back
        self.update()


def write_backlog(unsent_msgs: list, backlog_file: str, max_len: int) -> None:
    """
    write unsent messages to backlog file in flash
    """
    # if there are no unsent messages, remove backlog file
    if not unsent_msgs:
        if backlog_file in os.listdir():
            os.remove(backlog_file)
        return

    # do not let backlog grow too big
    while len(unsent_msgs) > max_len:
        unsent_msgs.pop(0)  # throw away the oldest message

    # store unsent messages
    with open(backlog_file, 'w') as file:
        for msg in unsent_msgs:
            file.write(msg + "\n")


def get_backlog(backlog_file: str) -> list:
    """
    get unsent messages from backlog file in flash
    """
    backlog = []
    if backlog_file in os.listdir():
        with open(backlog_file, 'r') as file:
            for line in file:
                backlog.append(line.rstrip("\n"))
    return backlog


def formated_time():
    """Helper function to reformat time to the specific format from below."""
    ct = time.localtime()
    return "{0:04d}-{1:02d}-{2:02d}T{3:02d}:{4:02d}:{5:02d}Z".format(*ct)  # modified to fit the correct format


def translate_backend_log_level(log_level: str):
    """
    Translate different logging levels from backend into actual logging levels.
    :param log_level: logging level from backend
    :return: translated logging level for logger
    """
    switcher = {
        'error': logging.ERROR,
        'warning': logging.WARNING,
        'info': logging.INFO,
        'debug': logging.DEBUG
    }
    return switcher.get(log_level, logging.INFO)


def translate_backend_state_name(state: str):
    """
    Translate state-machine state from backend into actual state name.
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
    return switcher.get(state, 'error') # default returns error


def translate_reset_cause(reset_cause: int):
    """
    Translate reset cause into readable string.
    :param reset_cause: from machine
    :return: translated reset cause string
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


def get_current_version():
    try:
        from OTA_VERSION import VERSION
    except ImportError:
        VERSION = '1.0.0'
    return VERSION


def read_log(num_errors: int = 3):
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
            if "ERROR" in line[
                          :42]:  # only look at the beginning of the line, otherwise the string can appear recursively
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