#
# This file implements a mock-accelerometer
# Intended to be used with raw logs from elevate-research/rawData/
# The data contained in the logs dictates that the emulated "fifo"
# has a size of 32 values
# The time in which the "fifo" gets filled is also provided in the data log file
# Using this time, a sample rate is calculated for each batch of 32 values
# The first batch of values will be written to the fifo with a rate of 75 values/s
#

import time
import _thread

#
# Initialisation parameters:
#   data_file: The file to read from
#   loop_data: If set to True, the data read from the file (data_file) will be looped (Default: True)
#   fifo_full_cb: The function to call when the fifo is "full" (can be changed using enable_fifo_interrupt()) (Default: None)
#   fifo_full_cb_pin: The value for the bin parameter to be passed to the callback (Default: 0)
#
class MockAccelerometer:
  """ mocks a pycom-pysense-accelerometer """
  def __init__(self, data_file : str, loop_data : bool = True, fifo_full_cb = None, fifo_full_cb_pin = 0):
    self.data_file = data_file
    self.fd = None
    self.fifo_full_cb = fifo_full_cb
    self.fifo_full_cb_pin = fifo_full_cb_pin
    self.fifo_lock = None
    self.stopped = True
    self.loop_count = 0

    self.value_lines = []   # stores parsed values from the data file
    self.loop = loop_data   # controls whether the data from data_file should be looped or not
    self.batch_time = 32/75 # time until the current batch of values should be written to the fifo
                            # (75 values/second -> 0.426... seconds/batch)
    self.fifo = []          # stores the current batch of values (32) ("FIFO")
    self.fifo_index = 0     # stores the current fifo index

    # create the fifo lock
    self.fifo_lock = _thread.allocate_lock()

    # clear the fifo
    self.restart_fifo()

    # parse input data
    self.parse_data()

    return

  def open_data_file(self):
    """ open the data file """
    self.fd = open(self.data_file, "r")

  def close_data_file(self):
    """ close the data file """
    self.fd.close()

  def acceleration(self):
    """ returns the current acceleration values """
    return tuple(self.fifo.copy())

  def enable_fifo_interrupt(self, cb):
    """ set the function to be called when the fifo is full """
    self.fifo_full_cb = cb

  def restart_fifo(self):
    """ clear the fifo """
    with self.fifo_lock:
      self.fifo = [0] * 32

  def fifo_write(self, value):
    """ writes a value to the fifo """
    with self.fifo_lock:
      # write data and increase the index
      self.fifo[self.fifo_index] = value
      self.fifo_index += 1

      # check if the fifo is "full"
      if self.fifo_index == 32:
        self.fifo_index = 0

        # check if a callback is registered
        if self.fifo_full_cb != None:
          self.fifo_full_cb(self.fifo_full_cb_pin)

  def parse_data(self):
    """ reads the data file and stores its values """
    # open the file
    self.open_data_file()

    # read the file
    # each line represents one 32 value batch
    # the values are prepended by two timestamps:
    #   the time in seconds
    #   the time in milliseconds
    # these timestamps will be ignored since a constant sampling rate of 75Hz is assumed
    for line in self.fd.readlines():
      # remove the trailing newline
      line.rstrip("\n")

      # split the line into a list and convert each value into a float
      # cut of the first two values (timestamps) and add the list to value_lines
      self.value_lines.append(list(map(lambda x: float(x), line.split(",")[2:-1])))

    # close the file
    self.close_data_file()

  def _loop(self, _):
    """ read data from the file and feed it into the fifo """
    while True:
      for values in self.value_lines:
        # write values to the fifo
        for value in values:
          # write the value
          self.fifo_write(value)

          # wait for 1/75 seconds - some compensation for runtime
          time.sleep((990 / 75) / 1000)

        # check for self.stopped after every batch of values
        if self.stopped == True:
          _thread.exit()

      # check if the file (-> the values) should be looped
      if self.loop != True:
        break

      self.loop_count += 1

    _thread.exit()

  def start(self):
    """ start the mock sensor """
    # check if the mock sensor is currently running
    if self.stopped == True:
      self.stopped = False

      _thread.start_new_thread(self._loop, (0, ))
    else:
      # ignore the start-call
      return

  def stop(self):
    """ stop the mock sensor """
    self.stopped = True
