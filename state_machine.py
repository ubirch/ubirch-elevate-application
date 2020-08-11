## from:
## https://learn.adafruit.com/circuitpython-101-state-machines?view=all#code

import time
import machine
import ubinascii
from sensor import MovementSensor
from config import *
import pycom

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
            #print('Exiting %s' % (self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        #print('Entering %s' % (self.state.name))
        self.state.enter(self)

    def update(self):
        if self.state:
            #print('Updating %s' % (self.state.name))
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
        # self.firework_color = random_color()
        # self.burst_count = 0
        # self.shower_count = 0
        # self.firework_step_time = time.ticks_ms() + 0.05
        # strip.fill(0)
        # strip.show()




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

    def exit(self, machine):
        State.exit(self, machine)
        machine.shower_count = 0

    def update(self, machine):
        if State.update(self, machine):
            now = time.ticks_ms()
            if now >= self.cellular_wait_time:
                machine.go_to_state('waitingForOvershoot')


# Show a shower of sparks following an explosion

class WaitingForOvershootState(State):

    def __init__(self):
        super().__init__()
        global g_trigger

    @property
    def name(self):
        return 'waitingForOvershoot'

    def enter(self, machine):
        State.enter(self, machine)

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if State.update(self, machine):
            if machine.sensor.trigger:
                speed_max, speed_min = machine.sensor.calc_speed()
                i = 0
                while i < 3:
                    if speed_max[i] > g_THRESHOLD:
                        # pycom.rgbled(0x007f00)
                        machine.go_to_state('idle')
                    if speed_min[i] < -g_THRESHOLD:
                        # pycom.rgbled(0x7f0000)
                        machine.go_to_state('idle')
                    i += 1


# Do nothing, wait to be reset

class IdleState(State):

    def __init__(self):
        super().__init__()
        #self.cellular_wait_time = 0

    @property
    def name(self):
        return 'idle'

    def enter(self, machine):
        State.enter(self, machine)
        #now = time.ticks_ms()
        #self.cellular_wait_time = now + 1000
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
            #now = time.ticks_ms()
            #if now >= self.cellular_wait_time:
            # pycom.rgbled(0x000000)
            machine.go_to_state('waitingForOvershoot')




# Reset the LEDs and audio, start the servo raising the ball
# When the switch is released, stop the ball and move to waiting

class RaisingState(State):

    def __init__(self):
        super().__init__()

    @property
    def name(self):
        return 'raising'

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


# Pause, resuming whem the switch is pressed again.
# Reset if the switch has been held for a second.

class PausedState(State):

    def __init__(self):
        super().__init__()
        self.switch_pressed_at = 0
        self.paused_servo = 0

    @property
    def name(self):
        return 'paused'

    def enter(self, machine):
        State.enter(self, machine)
        self.switch_pressed_at = time.ticks_ms()
        if audio.playing:
            audio.pause()
        self.paused_servo = servo.throttle
        servo.throttle = 0.0

    def exit(self, machine):
        State.exit(self, machine)

    def update(self, machine):
        if switch.fell:
            if audio.paused:
                audio.resume()
            servo.throttle = self.paused_servo
            self.paused_servo = 0.0
            machine.resume_state(machine.paused_state)
        elif not switch.value:
            if time.ticks_ms() - self.switch_pressed_at > 1.0:
                machine.go_to_state('raising')

################################################################################
