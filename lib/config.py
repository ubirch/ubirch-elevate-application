import uos

import ujson as json

NIOMON_SERVICE = "http://unsafe.niomon.{}.ubirch.com"
DATA_SERVICE = "https://data.{}.ubirch.com/v1"
BOOTSTRAP_SERVICE = "https://api.console.{}.ubirch.com/ubirch-web-ui/api/v1/devices/bootstrap"


def load_config(sd_card_mounted: bool = False) -> dict:
    """
    Load available configurations. First set default configuration (see "default_config.json"),
    then overwrite defaults with configuration from user config file ("config.json")
    the config file should be placed in the same directory as this file
    {
        "connection": "<'wifi' or 'nbiot'>",
        "apn": "<APN for NB IoT connection",
        "band": <LTE frequency band (integer) or 'null' to scan all bands>,
        "nbiot_attach_timeout": <int in seconds, timeout after which the nb-iot attach is aborted and board reset>,
        "nbiot_connect_timeout": <int in seconds, timeout after which the nb-iot connect is aborted and board reset>,
        "nbiot_extended_attach_timeout": <int in seconds, extended attach timeout, used when not coming from sleep (after power-on, errors)>,
        "nbiot_extended_connect_timeout": <int in seconds, extended connect timeout, used when not coming from sleep (after power-on, errors)>,
        "watchdog_timeout": <int in seconds, if execution takes longer than this in total, the board is reset>,
        "watchdog_extended_timeout": <int in seconds, extended watchdog timeout, used when not coming from sleep (after power-on, errors)>,
        "networks": {
          "<WIFI SSID>": "<WIFI PASSWORD>"
        },
        "board": "<'pysense' or 'pytrack'>",
        "ubirchAuthToken": "<auth token for the ubirch backend>",
        "keyService": "<URL of key registration service>",
        "niomon": "<URL of authentication service>",
        "data": "<URL of data service>",
        "verify": "<URL of verification service>",
        "bootstrap": "<URL of bootstrap service>",
        "interval": <measure interval in seconds>,
        "debug": <true or false>
    }
    :param sd_card_mounted: flag for sd card usage instead of internal flash
    :return: a dict with the available configurations
    """

    # load default config
    default_config = "default_config.json"
    with open(default_config, 'r') as c:
        cfg = json.load(c)

    # overwrite default config with user config if there is one
    user_config = "config.json"
    if user_config in uos.listdir():
        with open(user_config, 'r') as c:
            user_cfg = json.load(c)
            cfg.update(user_cfg)

    # overwrite existing config with config from sd card if there is one
    sd_config = 'config.txt'
    if sd_card_mounted and sd_config in uos.listdir('/sd'):
        with open('/sd/' + sd_config, 'r') as c:
            api_config = json.load(c)
            cfg.update(api_config)

    # ensure that the ubirch backend auth token is set
    if cfg['ubirchAuthToken'] is None:
        raise Exception("missing auth token")

    # set default values for unset service URLs
    if 'niomon' not in cfg:
        cfg['niomon'] = NIOMON_SERVICE.format(cfg['env'])

    # now make sure the env key has the actual environment value that is used in the URL
    if "https" in cfg['niomon']:
        cfg['env'] = cfg['niomon'].split(".")[1] # https endpoint
    else:
        cfg['env'] = cfg['niomon'].split(".")[2] # http endpoint

    if cfg['env'] not in ["dev", "demo", "prod"]:
        raise Exception("invalid ubirch backend environment \"{}\"".format(cfg['env']))

    # and set remaining URLs
    if 'data' not in cfg:
        cfg['data'] = DATA_SERVICE.format(cfg['env'])
    else:
        cfg['data'] = cfg['data'].rstrip("/msgPack")

    if 'bootstrap' not in cfg:
        cfg['bootstrap'] = BOOTSTRAP_SERVICE.format(cfg['env'])

    return cfg
