"""
TODO
"""
from uuid import UUID

import urequests as requests


def _send_request(url: str, data: bytes, headers: dict) -> (int, bytes):
    """
    Send a http post request to the backend.
    :param url: the backend service URL
    :param data: the data to send to the backend
    :param headers: the headers for the request
    :return: the backend response status code, the backend response content (body)
    """
    r = requests.post(url=url, data=data, headers=headers)
    return r.status_code, r.content


class ElevateAPI:
    """elevate API accessor methods."""

    def __init__(self, cfg: dict):
        self.debug = True
        # cfg['debug']
        self.data_url = cfg['elevate_data_url']
        self._elevate_headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'X-App-Token': cfg['elevate_api_token']
        }

    def send_data(self, uuid: UUID, message: bytes) -> (int, bytes):
        """
        TODO
        Send a JSON data message to the ubirch data service. Requires encoding before sending.
        :param uuid: the sender's UUID
        :param auth: the ubirch backend auth token (password)
        :param message: the encoded JSON message to send to the data service
        :return: the server response status code, the server response content (body)
        """
        if self.debug:
            print("** sending data message to " + self.data_url)
        return _send_request(url=self.data_url,
                             data=message,
                             headers=self._elevate_headers)

    def generate_data_package(self, x, y, z):
        """
        TODO
        """
        print("TODO")
