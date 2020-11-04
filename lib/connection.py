import sys
import utime as time

import machine
from network import LTE


class Connection:

    def connect(self): # todo rename 'ensure connection'
        raise NotImplementedError

    def isconnected(self) -> bool:
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError


class NB_IoT(Connection):

    def __init__(self, lte: LTE, apn: str, band: int or None, attachtimeout: int, connecttimeout: int):
        self.lte = lte
        self.apn = apn
        self.band = band
        self.attachtimeout = attachtimeout
        self.connecttimeout = connecttimeout

        self.lte.lte_callback(LTE.EVENT_COVERAGE_LOSS, self.cb_handler)

    def attach(self):
        if self.lte.isattached():
            return

        sys.stdout.write("\tattaching to the NB-IoT network")
        # since we disable unsolicited CEREG messages in modem.py, as they interfere with AT communication with the SIM via CSIM commands,
        # we are required to use an attach method that does not require cereg messages, for pycom that is legacyattach=false
        self.lte.attach(apn=self.apn, band=self.band, legacyattach=False)
        i = 0
        while not self.lte.isattached() and i < self.attachtimeout:
            i += 1
            time.sleep(1.0)
            # sys.stdout.write(".")
            print('.', end='')
        if not self.lte.isattached():
            raise OSError("!! unable to attach to NB-IoT network.")

        print("\n\t\tattached: {} s".format(i))

    def connect(self):
        if self.lte.isconnected():
            return

        if not self.lte.isattached(): self.attach()

        sys.stdout.write("\tconnecting to the NB-IoT network")
        self.lte.connect()  # start a data session and obtain an IP address
        i = 0
        while not self.lte.isconnected() and i < self.connecttimeout:
            i += 1
            time.sleep(1.0)
            # sys.stdout.write(".")
            print('.', end='')
        if not self.lte.isconnected():
            raise OSError("!! unable to connect to NB-IoT network.")

        print("\n\t\tconnected: {} s".format(i))
        # print('-- IP address: ' + str(lte.ifconfig()))

    def isconnected(self) -> bool:
        return self.lte.isconnected()

    def disconnect(self):
        if self.lte.isconnected():
            self.lte.disconnect()

    def setattachtimeout(self, attachtimeout: int):
        self.attachtimeout = attachtimeout

    def setconnecttimeout(self, connecttimeout: int):
        self.connecttimeout = connecttimeout

    def cb_handler(self, arg):
        print("CB: LTE Coverage lost")
        self.lte.deinit()


class WIFI(Connection):

    def __init__(self, networks: dict):
        from network import WLAN
        self.wlan = WLAN(mode=WLAN.STA)
        self.networks = networks

    def connect(self):
        if self.wlan.isconnected():
            return

        for _ in range(4):
            nets = self.wlan.scan()
            print("\tsearching for wifi networks...")
            for net in nets:
                if net.ssid in self.networks:
                    ssid = net.ssid
                    password = self.networks[ssid]
                    print('\twifi network ' + ssid + ' found, connecting ...')
                    self.wlan.connect(ssid, auth=(net.sec, password), timeout=5000)
                    while not self.wlan.isconnected():
                        machine.idle()  # save power while waiting
                    print('\twifi network connected')
                    print('\tIP address: {}\n'.format(self.wlan.ifconfig()))
                    return
            print("!! no usable networks found, trying again in 30s")
            print("!! available networks:")
            print("!! " + repr([net.ssid for net in nets]))
            machine.idle()
            time.sleep(30)

        raise OSError("!! unable to connect to WIFI network.")

    def isconnected(self) -> bool:
        return self.wlan.isconnected()

    def disconnect(self):
        if self.wlan.isconnected():
            self.wlan.disconnect()


connectionInstance = None


def get_connection(lte: LTE, cfg: dict) -> Connection:
    global connectionInstance
    if connectionInstance is not None:
        return connectionInstance
    if cfg['connection'] == "wifi":
        connectionInstance = WIFI(cfg['networks'])
        return connectionInstance
    elif cfg['connection'] == "nbiot":
        connectionInstance = NB_IoT(lte, cfg['apn'], cfg['band'], cfg['nbiot_attach_timeout'],
                                    cfg['nbiot_connect_timeout'])
        return connectionInstance
    else:
        raise Exception(
            "Connection type {} not supported. Supported types: 'wifi' and 'nbiot'".format(cfg['connection']))


