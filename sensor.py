import pyboard
import utime as time
import uos as os
from helpers import mount_sd

from sensor_config import *

# TODO, simplify the filtering and data

log_in_file = "log_raw.dat.10"
log_out_file = "log_filtered.dat.10"

def make_new_logfile(filename):
    mount_sd()
    while filename in os.listdir('/sd'):
        num = filename.split(".")
        print("OLD name ={}.{} num ={}".format(num[0], num[1], num[2]))
        number = int(num[2]) + 1
        log_file_new = "{}.{}.{}".format(num[0], num[1], number)
        print("NEW = {}".format(log_file_new))
        filename = log_file_new
    else:
        print("NO FILE FOUND")

    return filename

class MovementSensor(object):

    def __init__(self):
        self.accel_xyz = []
        self.accel_smooth = []
        self.accel_filtered = []
        self.accel_filtered_smooth = []
        self.speed = []
        self.speed_smooth = []
        self.speed_filtered = []
        self.speed_filtered_smooth = []
        self.speed_max = 0.0
        self.speed_min = 0.0
        self.altitude = 0.0
        self.temperature = 0.0
        self.trigger = False        # todo rename this variable to indicate, what happens
        self.overshoot = False
        self.last_print_ms = 0
        self.last_start_ms = 0

        self.log_in_file = make_new_logfile(log_in_file)
        self.log_out_file = make_new_logfile(log_out_file)

        self.globals_init()

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

        # set highpass filter
        # self.pysense.accelerometer.set_high_pass(1)
        # reset highpass filter
        self.pysense.accelerometer.reset_high_pass()

        # enable activity interrupt
        print("start")
        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.setup_fifo()
        self.pysense.accelerometer.enable_fifo_interrupt(self.accelerometer_interrupt_cb)
        print("int enabled")

    def poll_sensors(self):
        self.altitude = self.pysense.altimeter.altitude()
        self.temperature = self.pysense.humidity.temperature()

    # The accelerometer interrupt callback is triggered, when the fifo of the accelerometer is full.
    # It collects all data from the accelerometer and sets the trigger for the calculation.
    def accelerometer_interrupt_cb(self, pin):
        self.pysense.accelerometer.enable_fifo_interrupt(handler=None)
        # print("*")
        try:
            accel_tuple = self.pysense.accelerometer.acceleration()
            accel_list = list(accel_tuple)
        except Exception as e:
            print("ERROR      can't read data:", e)

        # get data
        for i in range(32):
            try:
                accel_tuple = self.pysense.accelerometer.acceleration()
                accel_list.extend(accel_tuple)
            except Exception as e:
                print("ERROR      can't read data:", e)
        i = 0
        while i < 32:
            j = 0
            while j < 3:
                self.accel_xyz[i][j] = accel_list[i * 3 + j]
                j += 1
            i += 1

        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.enable_fifo_interrupt(self.accelerometer_interrupt_cb)
        self.trigger = True
        # print("*",end="")
        # print("X: {} Y: {} Z: {}".format(self.accel_xyz[0][0], self.accel_xyz[0][1], self.accel_xyz[0][2]))

    def globals_init(self):
        for i in range(32):
            self.accel_xyz.append([])
            self.accel_smooth.append([])
            self.accel_filtered.append([])
            self.accel_filtered_smooth.append([])
            self.speed.append([])
            self.speed_smooth.append([])
            self.speed_filtered.append([])
            self.speed_filtered_smooth.append([])
            for j in range(3):
                self.accel_xyz[i].append(0.0)
                self.accel_smooth[i].append(0.0)
                self.accel_filtered[i].append(0.0)
                self.accel_filtered_smooth[i].append(0.0)
                self.speed[i].append(0.0)
                self.speed_smooth[i].append(0.0)
                self.speed_filtered[i].append(0.0)
                self.speed_filtered_smooth[i].append(0.0)

    # calculate the speed from the given acceleration values
    def calc_speed(self):
        timestamp = time.time()
        timestamp_ms = time.ticks_ms()
        z_buffer_accel_raw = []
        z_buffer_accel_raw.append("{},{},".format(timestamp, timestamp_ms))
        z_buffer_accel_filtered = []
        z_buffer_accel_filtered.append("{},{},".format(timestamp, timestamp_ms))

        self.trigger = False
        ACCELERATION_FILTER1_ALPHA = 0.0137 /3
        ACCELERATION_FILTER2_ALPHA = 0.0137 *3
        SPEED_FILTER1_ALPHA = 0.0137 /3
        SPEED_FILTER2_ALPHA = 0.0137 *3

        # run through the ring
        i = 0

        while i < 32:
            j = 0
            while j < 3:
                # Remove jitter from acceleration signal.
                self.accel_smooth[i][j] = ACCELERATION_FILTER1_ALPHA * self.accel_xyz[i][j] \
                                          + (1 - ACCELERATION_FILTER1_ALPHA) * self.accel_smooth[i -1][j]
                # Auto-calibrate: Filter out bias first using a DC bias filter.
                self.accel_filtered[i][j] = self.accel_xyz[i][j] - self.accel_smooth[i][j]

                self.accel_filtered_smooth[i][j] = ACCELERATION_FILTER2_ALPHA * self.accel_filtered[i][j] \
                                                   + (1 - ACCELERATION_FILTER2_ALPHA) * self.accel_filtered_smooth[i -1][j]

                # Accumulate past acceleration values (without gravity) to calculate speed.
                self.speed[i][j] = self.speed[i -1][j] + self.accel_filtered_smooth[i][j]

                # Average signal to remove high-frequency noise. Without this, a sudden movement like a
                # train passing nearby or an entering passenger could cause an overshoot event.
                self.speed_smooth[i][j] = SPEED_FILTER1_ALPHA * self.speed[i][j] \
                                          + (1 - SPEED_FILTER1_ALPHA) * self.speed_smooth[i -1][j]

                # The signal still has a DC bias. Remove it.
                self.speed_filtered[i][j] = self.speed[i][j] - self.speed_smooth[i][j]

                # Another low-pass filter on the result to remove jitter.
                self.speed_filtered_smooth[i][j] = SPEED_FILTER2_ALPHA * self.speed_filtered[i][j] \
                                                   + (1 - SPEED_FILTER2_ALPHA) * self.speed_filtered_smooth[i -1][j]

                j += 1

            z_buffer_accel_raw.append("{:.6f},".format(self.accel_xyz[i][2]))
            z_buffer_accel_filtered.append("{:.6f},".format(self.speed_filtered_smooth[i][2]))

            i += 1

        self.speed_min = min(min(self.speed_filtered_smooth))
        self.speed_max = max(max(self.speed_filtered_smooth))


        z_buffer_accel_raw.append("\n")
        with open('/sd/' + self.log_in_file, 'a') as file:
            for raw_data in z_buffer_accel_raw:
                file.write(raw_data)
        z_buffer_accel_raw.clear()

        z_buffer_accel_filtered.append("\n")
        with open('/sd/' + self.log_out_file, 'a') as file:
            for filtered_data in z_buffer_accel_filtered:
                file.write(filtered_data)
        z_buffer_accel_filtered.clear()

        # print(self.speed_min, self.speed_max)
        return # self.speed_max, self.speed_min
