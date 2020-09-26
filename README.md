# elevate-delta elevator application

The elevator application has the purpose to recognize if an elevator is moving 
and to provide this information to [accessibility.cloud](https://accessibility.cloud).

The project is a cooperation between [Sozialhelden](https://sozialhelden.de) and [Ubirch](https://ubirch.de).

Further information will follow soon.
  
## Configuration

you need to provide a `config.json` file, which includes:

```json
{
  "elevateDataUrl": "https://wheelmap.pro/hardware-sensors/",
  "elevateDeviceId": "<ENTER TOKEN HERE>",     // get this ID from the end of the URL
  "elevateDeviceToken": "<ENTER TOKEN HERE>",  // get this from "deice specific firmware access token
  "elevateAppToken": "<ENTER TOKEN HERE>", // should stay the same
  "ubirchAuthToken": "<ENTER TOKEN HERE>", // get this from ubirch console
  "env": "prod",
  "connection":"nbiot",
  "nbiot_extended_attach_timeout": 900,
  "nbiot_extended_connect_timeout": 60,
  "debug": true
}
```