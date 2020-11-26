"""
| Elevate Sim class, which inherits the SimProtocol from ubirch_sim and
| extends the functionality for the elevate usecase
|
"""
from ubirch import SimProtocol
from network import LTE


class ElevateSim(SimProtocol):

    def __init__(self, lte: LTE, at_debug: bool = False, channel: int = None):
        super(ElevateSim, self).__init__(lte, at_debug, channel)
        print("ElevateSim initialized")

    def __del__(self):
        super(ElevateSim, self).__del__()
        print("ElevateSim deleted")

    def get_signal_quality(self, debug_print=False):
        """
        Get the signal quality of the LTE connection
        :param debug_print: print output for debug purposes
        :return: rssi, ber
        """
        csq_cmd = "AT+CSQ"

        if self.DEBUG: print("\n>> getting signal quality")
        self._prepare_AT_session()
        try:
            # select SS entry
            data, code = self._send_at_cmd(csq_cmd)
        finally:
            self._finish_AT_session()

        if code == 'OK':
            response = data[5:].split(',')
            if debug_print: print(">> " + repr(data))
            return int(response[0]), int(response[1])

        raise Exception(code)

    def get_network_stats(self, debug_print=False):
        """
        Get the network status
        :param debug_print: print output for debug purposes
        :return:
        """
        csq_cmd = "AT+COPS?"

        if self.DEBUG: print("\n>> getting network status")
        self._prepare_AT_session()
        try:
            # select SS entry
            data, code = self._send_at_cmd(csq_cmd)
        finally:
            self._finish_AT_session()

        if code == 'OK':
            response = data.split(',')
            if debug_print: print(">> " + repr(data))
            return repr(data)

        raise Exception(code)


