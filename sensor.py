import pyboard
from pyboard.LIS2HH12 import FULL_SCALE_2G, ODR_100_HZ
import _thread

from sensor_config import *

_thread.stack_size(8192)

FIFO_VALUES = 32
FIFO_AXIS = 3

# TODO, simplify the filtering and data

class MovementSensor(object):
    """
    Movement Sensor Class with accelerometer as input and filtering of raw data in a thread.
    """

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
        self.overshoot = False

        self.init_filters()

        # initialise the accelerometer
        self.pysense = pyboard.Pysense()

        self.pysense.accelerometer.set_full_scale(FULL_SCALE_2G) # => max 2G; resolution: 31.25 micro G

        self.pysense.accelerometer.set_odr(ODR_100_HZ) # => 100 Hz; resolution: 80  milli seconds; max duration: 20400  ms

        # enable activity interrupt # CHECK: should say fifo instead of activity?
        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.setup_fifo()
        self.pysense.accelerometer.enable_fifo_interrupt(self.accelerometer_interrupt_cb)

        # The Filtering is happening in a thread, concurrent to the rest of the code.
        self.threadLock = _thread.allocate_lock()
        _thread.start_new_thread(self.sensor_filtering_thread, ())

    def poll_sensors(self):
        """
        Poll temperature and altitude values and store them in class attributes.
        """
        self.altitude = self.pysense.altimeter.altitude()
        self.temperature = self.pysense.humidity.temperature()

    def accelerometer_interrupt_cb(self, pin):
        """
        Accelerometer Interrupt Callback function.
        It is triggered, when the fifo of the accelerometer is full.
        It collects all data from the accelerometer and releases the threadLock for data filtering.
        """
        self.pysense.accelerometer.enable_fifo_interrupt(handler=None) # disable the fifo interrupt handler
        # get data
        for i in range(FIFO_VALUES):
            try:
                self.accel_xyz[i] = self.pysense.accelerometer.acceleration()
            except Exception as e:
                print("ERROR      can't read data:", e)

        self.pysense.accelerometer.restart_fifo()
        self.pysense.accelerometer.enable_fifo_interrupt(self.accelerometer_interrupt_cb)

        # release the threadLock, so that the filtering thread can process the data.
        if self.threadLock.locked():
            self.threadLock.release()

    def init_filters(self):
        """
        Initialize the filter variables for processing.
        """
        for i in range(FIFO_VALUES):
            self.accel_xyz.append([])
            self.accel_smooth.append([])
            self.accel_filtered.append([])
            self.accel_filtered_smooth.append([])
            self.speed.append([])
            self.speed_smooth.append([])
            self.speed_filtered.append([])
            self.speed_filtered_smooth.append([])
            for j in range(FIFO_AXIS):
                self.accel_xyz[i].append(0.0)
                self.accel_smooth[i].append(0.0)
                self.accel_filtered[i].append(0.0)
                self.accel_filtered_smooth[i].append(0.0)
                self.speed[i].append(0.0)
                self.speed_smooth[i].append(0.0)
                self.speed_filtered[i].append(0.0)
                self.speed_filtered_smooth[i].append(0.0)

    def perform_filters(self):
        """
        Perform the filtering functions from the given raw acceleration values.
        """
        ACCELERATION_FILTER1_ALPHA = 0.0137 /3
        ACCELERATION_FILTER2_ALPHA = 0.0137 *3
        SPEED_FILTER1_ALPHA = 0.0137 /3
        SPEED_FILTER2_ALPHA = 0.0137 *3

        for i in range(len(self.accel_smooth)):
            for j in range(len(self.accel_smooth[i])):
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
        return

    def movement(self):
        """
        Calculate th maximum absolute speed value from current sensor values,
        over all axis.
        :return: (indirect) Set the overshoot flag
        """
        self.overshoot = False
        self.speed_min = min(min(self.speed_filtered_smooth))
        self.speed_max = max(max(self.speed_filtered_smooth))

        if self.speed_max > g_THRESHOLD:
            self.overshoot = True
        if abs(self.speed_min) > g_THRESHOLD:
            self.overshoot = True
        return

    def sensor_filtering_thread(self):
        """
        Filter the raw sensor data in a thread.
        The while True loop is necessary to keep this thread alive.
        The thread will wait for the treadLock and then process the data.
        """
        while True:
            self.threadLock.acquire() # wait here, until threadLock is released.
            self.perform_filters()
            self.movement()

