"""
TODO
"""
from uuid import UUID

import ujson as json

import lib.urequests as requests


def _send_request(url: str, data: bytes, headers: dict) -> (int, bytes):
    """
    Send a http patch request to the backend.
    :param url: the backend service URL
    :param data: the data to send to the backend
    :param headers: the headers for the request
    :return: the backend response status code, the backend response content (body)
    """
    r = requests.patch(url=url, data=data, headers=headers)
    return r.status_code, r.content


def _get_request(url: str, headers: dict) -> (int, bytes):
    """
    Send a http get request to the backend.
    :param url: the backend service URL
    :param headers: the headers for the request
    :return: the backend response status code, the backend response content (body)
    """
    r = requests.get(url=url, headers=headers)
    return r.status_code, r.content


class ElevateAPI:
    """elevate API accessor methods."""

    def __init__(self, cfg: dict):
        self.debug = True
        # cfg['debug']
        self.data_url = cfg['elevateDataUrl'] + cfg['elevateDeviceId']
        self._elevate_headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'X-App-Token': cfg['elevateAppToken'],
            'X-Device-Token': cfg['elevateDeviceToken']
        }

    def send_data(self, uuid: UUID, message: bytes) -> (int, bytes):
        """
        TODO
        Send a JSON data message to the ubirch data service. Requires encoding before sending.
        :param uuid: the sender's UUID
        :param auth: the ubirch backend auth token (ubirchAuthToken)
        :param message: the encoded JSON message to send to the data service
        :return: the server response status code, the server response content (body)
        """
        if self.debug:
            print("** sending data message to " + self.data_url)
        return _send_request(url=self.data_url + "?reduceHeaders=1",
                             data=message,
                             headers=self._elevate_headers)

    def get_state(self, uuid: UUID, message: bytes) -> (int, str, str):
        """
        TODO
        :return:
        """
        log_level = ""
        state = ""
        if self.debug:
            print("** getting the current state from " + self.data_url)

        r, c = _get_request(url=self.data_url + "?include=properties.firmwareLogLevel,properties.firmwareState",
                            headers=self._elevate_headers)
        if r == 200:
            state_info = json.loads(c)
            # print("dump", json.dumps(state_info))
            if 'properties' in state_info:
                props = state_info['properties']
                # print(props)
                if 'firmwareLogLevel' in props:
                    log_level = props['firmwareLogLevel']
                if 'firmwareState' in props:
                    state = props['firmwareState']
        return r, log_level, state

    """
    {
        "_id":"KBkgm6qDZE2i94c2Q",
        "properties":{
            "equipmentInfoId":"BsLsSecZhYzizCYhw",
            "ownerId":"fiN4c4HNdmSBT2JzQ",
            "productId":"8XmwxWa6PwvpT9jEx",
            "createdAt":{
                "$date":1598644767333
            },
            "name":"Testsensor 2",
            "firmwareLogLevel":"warning",
            "firmwareState":"blinking",
            "variables":{
                "isWorking":{
                    "value":true,
                    "updatedAt":{
                        "$date":1600115661477
                    }
                }
            }
        },
        "related":{}
    }
    """

    def generate_data_package(self, x, y, z):
        """
        TODO
        """
        print("TODO")
