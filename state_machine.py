## from:
## https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code

import time
import machine
import ubinascii
from sensor import MovementSensor
from config import *
import pycom


LED_OFF = 0x000000

#standard brightness: 1% (low-power)
LED_WHITE     = 0x030303
LED_GREEN     = 0x000600
LED_YELLOW    = 0x060600
LED_ORANGE    = 0x060200
LED_RED       = 0x060000
LED_PURPLE    = 0x030006
LED_BLUE      = 0x000006
LED_TURQUOISE = 0x010605
LED_PINK      = 0x060002

#full brightness (for errors etc)
LED_WHITE_BRIGHT     = 0xffffff
LED_GREEN_BRIGHT     = 0x00ff00
LED_YELLOW_BRIGHT    = 0xffff00
LED_ORANGE_BRIGHT    = 0xffa500
LED_RED_BRIGHT       = 0xff0000
LED_PURPLE_BRIGHT    = 0x800080
LED_BLUE_BRIGHT      = 0x0000ff
LED_TURQUOISE_BRIGHT = 0x40E0D0
LED_PINK_BRIGHT      = 0xFF1493


DROP_DURATION = 10
################################################################################
# State Machine

class StateMachine(object):

    def __init__(self):
        self.state = None
        self.states = {}
        self.sensor = MovementSensor()

        print("\n\n\n\n\n[Core] Initializing magic... ✨ ")
        print("[Core] Hello, I am " , ubinascii.hexlify(machine.unique_id()))

        # Serial.println("!");
        # Particle.function("locate", locate);
        # sensorDriver.init();
        # networkDriver.init();
        # battery.init();
        #
        # Particle.variable("curTimeoutMs", intervalForInactivityEventMs);

    def add_state(self, state):
        self.states[state.name] = state

    def go_to_state(self, state_name):
        if self.state:
            print('Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        print('Entering %s' % (self.state.name))
        self.state.enter(self)

    def update(self):
        if self.state:
            print('Updating %s' % (self.state.name))
            self.state.update(self)

    # When pausing, don't exit the state
    def pause(self):
        self.state = self.states['paused']
        print('Pausing')
        self.state.enter(self)

    # When resuming, don't re-enter the state
    def resume_state(self, state_name):
        if self.state:
            print('Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        print('Resuming %s' % (self.state.name))

    def reset_state_machine(self):
        """As indicated, reset the machines system's variables."""
        print('Resetting the machine')

################################################################################
# States


# Abstract parent state class.

class State(object):

    def __init__(self):
        pass

    @property
    def name(self):
        return ''

    def enter(self, machine):
        pass

    def exit(self, machine):
        pass

    def update(self, machine):
        # if switch.fell:
        #     machine.paused_state = machine.state.name
        #     machine.pause()
        #     return False
        return True


#Connecting State to connect to network

class ConnectingState(State):

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'connecting'

    def enter(self, machine):
        State.enter(self, machine)
        set_led(LED_TURQUOISE)

    def exit(self, machine):
        State.exit(self, machine)

    def connect(self):
        # connect to network maybe
        return True

    def update(self, machine):
        # here I want to wait for 500 ms
            machine.go_to_state('sendingVersionDiagnostics')

# Sending Version Diagnostics State
class SendingVersionDiagnosticsState(State):

    def __init__(self):
        super().__init__()
        self.version_wait_time = 0

    @property
    def name(self):
        return 'sendingVersionDiagnostics'

    def enter(self, machine):
        State.enter(self, machine)
        now = time.ticks_ms()
        self.version_wait_time = now + DROP_DURATION
        set_led(LED_BLUE)

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.version_wait_time:
                machine.go_to_state('sendingCellularDiagnostics')


# Sending Cellular Diagnostics State
class SendingCellularDiagnosticsState(State):

    def __init__(self):
        super().__init__()
        self.cellular_wait_time = 0

    @property
    def name(self):
        return 'sendingCellularDiagnostics'

    def enter(self, machine):
        State.enter(self, machine)
        now = time.ticks_ms()
        self.cellular_wait_time = now + DROP_DURATION
        set_led(LED_PURPLE)

    def exit(self, machine):
        State.exit(self, machine)
        machine.shower_count = 0

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.cellular_wait_time:
                machine.go_to_state('waitingForOvershoot')


# wait here for acceleration to overshoot the threshold
class WaitingForOvershootState(State):

    def __init__(self):
        super().__init__()
        global g_trigger

    @property
    def name(self):
        return 'waitingForOvershoot'

    def enter(self, machine):
        State.enter(self, machine)
        set_heartbeat()

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            if machine.sensor.trigger:
                speed_max, speed_min = machine.sensor.calc_speed()
                i = 0
                while i < 3:
                    if speed_max[i] > g_THRESHOLD:
                        set_led(LED_GREEN)
                        machine.go_to_state('measuringPaused')
                    if speed_min[i] < -g_THRESHOLD:
                        set_led(LED_RED)
                        machine.go_to_state('measuringPaused')
                    i += 1


# Send the data away
class MeasuringPausedState(State):

    def __init__(self):
        super().__init__()
        self.cellular_wait_time = 0

    @property
    def name(self):
        return 'measuringPaused'

    def enter(self, machine):
        State.enter(self, machine)
        now = time.ticks_ms()
        self.cellular_wait_time = now + 1000
        print(machine.sensor.speed_max, machine.sensor.speed_min)
        machine.sensor.speed_max[0] = 0.0
        machine.sensor.speed_max[1] = 0.0
        machine.sensor.speed_max[2] = 0.0
        machine.sensor.speed_min[0] = 0.0
        machine.sensor.speed_min[1] = 0.0
        machine.sensor.speed_min[2] = 0.0

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.cellular_wait_time:
                set_led(LED_OFF)
                machine.go_to_state('waitingForOvershoot')

# Reset the LEDs and audio, start the servo raising the ball
# When the switch is released, stop the ball and move to waiting

class ErrorState(State):

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'error'

    def enter(self, machine):
        State.enter(self, machine)
        strip.fill(0)
        strip.brightness = 1.0
        strip.show()
        if audio.playing:
            audio.stop()
        servo.throttle = RAISE_THROTTLE

    def exit(self, machine):
        State.exit(self, machine)
        servo.throttle = 0.0

    def update(self, machine):
        if State.update(self, machine):
            if switch.rose:
                machine.go_to_state('waiting')

################################################################################

def set_led(led_color):
    pycom.heartbeat(False)  # disable blue heartbeat blink
    pycom.rgbled(led_color)

def set_heartbeat():
    pycom.heartbeat(True)
