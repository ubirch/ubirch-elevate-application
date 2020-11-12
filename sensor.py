import pyboard
import utime as time


class MovementSensor(object):
    def __init__(self):
        self.accel = 0.0
        self.accel_filtered = 0.0
        self.accel_smooth = 0.0
        self.accel_filtered_smooth = 0.0
        self.speed = 0.0
        self.speed_smooth = 0.0
        self.speed_filtered = 0.0
        self.speed_filtered_smooth = 0.0
        self.speed_min = 0.0
        self.speed_max = 0.0
        self.accel_max = 0.0
        self.accel_min = 0.0
        self.altitude = 0.0
        self.temperature = 0.0

        self.active = False
        self.overshoot = False
        self.direction = '🔸'
        self.last_print_ms = 0
        self.last_start_ms = 0

        # initialise the accelerometer
        self.pysense = pyboard.Pysense()

        # taken from LIS2HH12.py
        #   ARG => threshold
        #   3 => max 8G; resolution: 125   micro G
        #   2 => max 4G; resolution: 62.5  micro G
        #   0 => max 2G; resolution: 31.25 micro G
        self.pysense.accelerometer.set_full_scale(0)

        # taken from LIS2HH12.py
        #   ARG => duration
        #   0 => POWER DOWN
        #   1 => 10  Hz; resolution: 800 milli seconds; max duration: 204000 ms
        #   2 => 50  Hz; resolution: 160 milli seconds; max duration: 40800  ms
        #   3 => 100 Hz; resolution: 80  milli seconds; max duration: 20400  ms
        #   4 => 200 Hz; resolution: 40  milli seconds; max duration: 10200  ms
        #   5 => 400 Hz; resolution: 20  milli seconds; max duration: 5100   ms
        #   6 => 500 Hz; resolution: 10  milli seconds; max duration: 2550   ms
        self.pysense.accelerometer.set_odr(3)

    def start(self):
        self.last_start_ms = time.ticks_ms()
        self.speed_max = self.speed_min = 0
        self.overshoot = False
        self.direction = '🔸'

        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.setup_fifo()
        self.active = True
        self.pysense.accelerometer.enable_fifo_interrupt(self.accelerometer_interrupt_cb)

    def stop(self):
        self.active = False
        self.pysense.accelerometer.enable_fifo_interrupt(handler=None)

    def poll_sensors(self):
        self.altitude = self.pysense.altimeter.altitude()
        self.temperature = self.pysense.humidity.temperature()

    # The accelerometer interrupt callback is triggered, when the fifo of the accelerometer is full.
    def accelerometer_interrupt_cb(self, pin):
        self.pysense.accelerometer.enable_fifo_interrupt(handler=None)
        if not self.active:
            return
        for _ in range(32):
            accel_xyz_tuple = self.pysense.accelerometer.acceleration()
            self.process_next_sample(accel_xyz_tuple)
        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.enable_fifo_interrupt(self.accelerometer_interrupt_cb)


    def print_status_table(self):
        print("┌---------------------------------------┬---------------------------------------------------------┬---------┬--------┬--------┐")
        print("| acceleration                          |  speed (estimated)                                      | time    | alt    | temp   |")
        print("| raw      smooth   w/o DC    smooth    |  raw        w/o DC     smooth     min        max        |         |        |        |")
        print("├---------------------------------------┼---------------------------------------------------------┼---------┼--------┼--------┤")
        print("|                                       |                                                         |         |        |        |")
        print("└---------------------------------------┴---------------------------------------------------------┴---------┴--------┴--------┘")

    def print_status(self):
        now = time.ticks_ms()
        if (now - self.last_print_ms < 250):
            return
        self.last_print_ms = now
        print("\r", end="")
        print("\033[s\033[2A│ %+.3fg  %+.3fg  %+.3fg  %+.3fg    │  %+.3fm/s  %+.3fm/s  %+.3fm/s  %+.3fm/s  %+.3fm/s  │ %6.1fs | %5.1fm | %3.1f°C \033[u" % (
            self.accel,
            self.accel_smooth,
            self.accel_filtered,
            self.accel_filtered_smooth,
            self.speed,
            self.speed_filtered,
            self.speed_filtered_smooth,
            self.speed_min,
            self.speed_max,
            0.001 * (time.ticks_ms() - self.last_start_ms),
            self.altitude,
            self.temperature,
        ), end="")

    def process_next_sample(self, accel_xyz_tuple):
        if self.overshoot:
            return
        acceleration_filter1_alpha = 0.04
        acceleration_filter2_alpha = 0.5
        speed_filter1_alpha = 0.03
        speed_filter2_alpha = 0.2
        threshold = 0.2

        # Calculate length of 3D acceleration vector and remove gravity (1.0g)
        self.accel = (accel_xyz_tuple[0] ** 2 + accel_xyz_tuple[1] ** 2 + accel_xyz_tuple[2] ** 2) ** (1./2.) - 1.0

        # Remove jitter from acceleration signal.
        self.accel_smooth = acceleration_filter1_alpha * self.accel + (1 - acceleration_filter1_alpha) * self.accel_smooth

        # Auto-calibrate: Filter out bias first using a DC bias filter.
        self.accel_filtered = self.accel - self.accel_smooth

        self.accel_filtered_smooth = acceleration_filter2_alpha * self.accel_filtered + (1 - acceleration_filter2_alpha) * self.accel_filtered_smooth

        # Accumulate past acceleration values (without gravity) to calculate speed.
        self.speed = self.speed + self.accel_filtered_smooth

        # Average signal to remove high-frequency noise. Without this, a sudden movement like a
        # train passing nearby or an entering passenger could cause an overshoot event.
        self.speed_smooth = speed_filter1_alpha * self.speed + (1 - speed_filter1_alpha) * self.speed_smooth

        # The signal still has a DC bias. Remove it.
        self.speed_filtered = self.speed - self.speed_smooth

        # Another low-pass filter on the result to remove jitter.
        self.speed_filtered_smooth = speed_filter2_alpha * self.speed_filtered + (1 - speed_filter2_alpha) * self.speed_filtered_smooth

        # Discard the first few samples after a restart to ignore overshoot event from artifacts
        if (time.ticks_ms() - self.last_start_ms > 3000):
            self.speed_max = max(self.speed_max, self.speed_filtered_smooth)
            self.speed_min = min(self.speed_min, self.speed_filtered_smooth)
            self.accel_max = max(self.accel_max, self.accel_filtered_smooth)
            self.accel_min = min(self.accel_min, self.accel_filtered_smooth)

            if abs(self.speed_filtered_smooth) > threshold:
                self.overshoot = True
                if self.speed_filtered_smooth > 0:
                    self.direction = '🔺'
                if self.speed_filtered_smooth < 0:
                    self.direction = '🔻'

