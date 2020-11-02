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
  "elevateDeviceToken": "<ENTER TOKEN HERE>",  // get this from device specific firmware access token
  "elevateAppToken": "<ENTER TOKEN HERE>", // should stay the same
  "ubirchAuthToken": "<ENTER TOKEN HERE>", // get this from ubirch console
  "env": "prod",
  "connection":"nbiot",
  "nbiot_extended_attach_timeout": 900,
  "nbiot_extended_connect_timeout": 60,
  "debug": true
}
```
## Programming the device

1. Clone this repository in a directory of your choice or download and extract a [release zip file](https://github.com/ubirch/ubirch-testkit/releases).
      ```
      $ git clone https://github.com/ubirch/ubirch-elevate-application.git
      ```

1. Add the project directory to your IDE's working directory:
    - Atom: `File` -> `Add Project Folder`
    - VS Code: `File` -> `Open Folder`

You should now see the code files (.py) in your IDE's folder view. We can now upload the code to the board:
1. Connect your assembled Pycom device with the inserted SIM card to your computer via USB. Make sure that the Pymakr console is connected. (Checkmark on the "Pymakr Console" Button.) If not, click the "Pymakr Console" button to establish a connection. In the Pymakr console in your IDE, you should see the following output:
      ```
      Connecting to /dev/ttyACM0...
      
      >>> 
      ```
1.  Press [CTRL+C] a few times, until you see:
    ```bash
    >>>
    ```
1.  Press the Pymakr `UPLOAD` button. This transfers the example code into the GPy's internal flash.The code will be uploaded to the board and should start to print information in the Pymakr console while it is running. You can ignore error messages regarding missing configuration or auth tokens, as we will set this up later. You now have a GPy running the Testkit code, the next step is to register your SIM with the UBIRCH backend and upload the configuration to the Testkit.
