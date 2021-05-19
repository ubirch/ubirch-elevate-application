import utime as time

from network import LTE
from ubirch import ModemInterface

MAX_ERROR_RESP_PREFIX = len("+CME ERROR")

MODEM_DEBUG = True

class Modem(ModemInterface):
    """
    todo
    """
    def __init__(self, lte: LTE, error_handler = None, debug: bool = MODEM_DEBUG):
        """
        Initialize with error handler.
        :param debug: todo
        """
        self.lte = lte
        self._AT_session_active = False  # weather or not the lib currently opened an AT commands session
        self._AT_session_modem_suspended = False  # weather the modem was suspended for an AT session
        self.debug = debug
        self.error_handler = error_handler

    def prepare_AT_session(self) -> None:
        """
        Ensures all prerequisites to send AT commands to modem and saves the modems state
        for restoring it later.
        """
        if self._AT_session_active:
            return

        # if modem is connected, suspend it and remember that we did
        if self.lte.isconnected():
            self.lte.pppsuspend()
            self._AT_session_modem_suspended = True

        self._AT_session_active = True

    def finish_AT_session(self) -> None:
        """
        Restores the modem state after the library is finished sending AT commands.
        """
        if not self._AT_session_active:
            return

        # if modem was suspended for the session, restore it
        if self._AT_session_modem_suspended:
            self.lte.pppresume()
            self._AT_session_modem_suspended = False

        self._AT_session_active = False

    def check_sim_access(self) -> bool:
        """
        Checks Generic SIM Access.
        :return: if SIM access was successful
        """
        try:
            self.send_at_cmd("AT+CSIM=?", expected_result_prefix="OK", max_retries=10)
            return True
        except:
            return False

    def send_at_cmd(self, cmd: str, expected_result_prefix: str = None, max_retries: int = 1) -> str:
        """
        Sends an AT command to the modem. This method uses the `send_at_cmd` method of LTE. It additionally filters
        its output for unsolicited messages and potentially retries if AT command returned an error or invalid response.
        Throws an exception if all attempts fail.
        :param cmd: AT command to send to modem
        :param expected_result_prefix: the return value of LTE.send_at_cmd is
            parsed by this value, if None it is extracted from the command
        :param max_retries: the number of retries in case of an error or invalid response
        :return: AT response
        """
        at_prefix = "AT"
        if not cmd.startswith(at_prefix):
            raise Exception('use only for AT+ prefixed commands')

        if expected_result_prefix is None:
            if "=" in cmd:
                expected_result_prefix = cmd[len(at_prefix):].split('=', 1)[0]
            elif "?" in cmd:
                expected_result_prefix = cmd[len(at_prefix):].split('?', 1)[0]
            else:
                expected_result_prefix = cmd[len(at_prefix):]

        exc = Exception()
        for _ in range(max_retries):
            try:
                return self._send_at_cmd(cmd, expected_result_prefix)
            except Exception as e:
                exc = e
                time.sleep_ms(50)
        else:
            raise exc

    def _send_at_cmd(self, cmd: str, expected_result_prefix: str) -> str:
        """
        Sends AT command. This method uses the `send_at_cmd` method of
        LTE. It additionally filters its output for unsolicited messages.
        Throws an exception if the sent command returns an error or invalid response.
        :param cmd: command to send
        :param expected_result_prefix: the return value of LTE.send_at_cmd is parsed by this value
        :return: response message
        """
        if self.debug: print("++ " + cmd)
        result = [k for k in self.lte.send_at_cmd(cmd).split('\r\n') if len(k.strip()) > 0]
        if self.debug:
            print('-- ' + '\r\n-- '.join([r for r in result]))

        retval = None
        error = None

        # filter results
        skip_next_line = False
        for line_number, line in enumerate(result):
            if skip_next_line:
                skip_next_line = False
                continue
            elif "ERROR" in line[:MAX_ERROR_RESP_PREFIX + 1]:
                error = line
            elif line.startswith(expected_result_prefix):
                retval = line
                if line_number + 1 < len(result) and result[line_number + 1] == "OK":
                    skip_next_line = True
            else:
                # unsolicited
                print("WARNING: ignoring: {}".format(line))

        if retval is not None:
            return retval
        elif error is not None:
            raise Exception("command {} returned {}:\n{}".format(cmd, error, repr(result)))
        else:
            raise Exception("command {} returned no AT response:\n{}".format(cmd, repr(result)))

    def set_function_level(self, function_level: str) -> None:
        if self.debug: print("\tsetting function level: {}".format(function_level))
        self.send_at_cmd("AT+CFUN=" + function_level, expected_result_prefix="OK", max_retries=10)

    def get_function_level(self) -> str:
        result = self.send_at_cmd("AT+CFUN?", max_retries=10)
        if self.debug: print("CFUN = {}".format(result))
        return result.lstrip("+CFUN: ")

    def set_registration_level(self, registration_level: str) -> None:
        if self.debug: print("\tsetting network registration level: {}".format(registration_level))
        self.send_at_cmd("AT+CEREG=" + registration_level, expected_result_prefix="OK", max_retries=10)

    def get_registration_level(self) -> str:
        result = self.send_at_cmd("AT+CEREG?", max_retries=10)
        if self.debug: print("CEREG = {}".format(result))
        if "," in result:
            response = result.lstrip("+CEREG: ").split(',')
            return response[0]
        raise Exception("could not get registration level. Result was: {}".format(result))

    def reset(self) -> None:
        """
        Performs a hardware reset on the cellular modem. This function can take more than 5 seconds to return
        as it waits for the modem to shutdown, reboot and become responsive again.
        Throws an exception if reset fails.
        """
        function_level = "1"
        cereg_level = "1" # TODO keep for now, until first real deployment

        if self.debug: print("\twaiting for reset to finish")
        self.lte.reset()
        self.lte.init()

        self.set_function_level(function_level)
        if not self.get_function_level() == function_level:
            raise Exception("could not set modem function level")

        self.set_registration_level(cereg_level)
        if not self.get_registration_level() == cereg_level:
            raise Exception("could not set modem registration level")

        if self.debug: print("\twaiting for SIM to be responsive")
        if not self.check_sim_access():
            raise Exception("SIM does not seem to respond after reset")

    def get_imsi(self) -> str:
        """
        Get the international mobile subscriber identity (IMSI) of the SIM card
        """
        IMSI_LEN = 15
        get_imsi_cmd = "AT+CIMI"

        if self.debug: print("\n>> getting IMSI")
        result = self.send_at_cmd(get_imsi_cmd, expected_result_prefix="", max_retries=10)
        if len(result) != IMSI_LEN:
            raise Exception("received invalid response: {}".format(result))
        int(result)  # throws ValueError if IMSI has invalid syntax for integer with base 10
        return result

    def get_signal_quality(self) -> (int, int):
        """
        Get the signal quality of the LTE connection
        :return: rssi, ber
            ++ AT+CSQ
            -- +CSQ: 24,99
            -- OK
        """
        CSQ_RESULT_LEN = len("+CSQ: 0,0")
        csq_cmd = "AT+CSQ"

        if self.debug: print("\n>> getting signal quality")
        result = self.send_at_cmd(csq_cmd)

        if len(result) < CSQ_RESULT_LEN or len(result) > (CSQ_RESULT_LEN + 2):
            raise Exception("received invalid response: {}".format(result))

        response = result.lstrip("+CSQ: ").split(',')
        return int(response[0]), int(response[1])

    def get_network_stats(self) -> str:
        """
        Get the network status
        :return:
            ++ AT+COPS?
            -- +COPS: 0,2,"26201",9
            -- OK
        """
        COPS_RESPONSE_LEN = len("+COPS: 0,2, 26201 ,9")
        cops_cmd = "AT+COPS?"

        if self.debug: print("\n>> getting network status")
        result = self.send_at_cmd(cops_cmd)

        if len(result) < COPS_RESPONSE_LEN:
            raise Exception("received invalid response: {}".format(result))

        return str(result.lstrip("+COPS: "))
