# https://pypi.org/project/micropython-logging/

import os
import sys
from . import Handler


def try_remove(fn: str) -> None:
    """Try to remove a file if it existst."""
    try:
        os.remove(fn)
    except OSError:
        print("LOG error")
        pass


def get_filesize(fn: str) -> int:
    """Return size of a file."""
    return os.stat(fn)[6]


class RotatingFileHandler(Handler):
    """A rotating file handler like RotatingFileHandler.
    # TODO this should be added to the code to avoid log file overflow.

    Compatible with CPythons `logging.handlers.RotatingFileHandler` class.
    """

    def __init__(self, filename, maxBytes=0, backupCount=0):
        super().__init__()
        self.filename = filename
        self.maxBytes = maxBytes
        self.backupCount = backupCount

        try:
            self._counter = get_filesize(self.filename)
        except OSError:
            print("LOG error")
            self._counter = 0

    def emit(self, record):
        """Write to file."""
        msg = self.formatter.format(record)
        s_len = len(msg)

        if self.maxBytes and self.backupCount and self._counter + s_len > self.maxBytes:
            # remove the last backup file if it is there
            try_remove(self.filename + ".{0}".format(self.backupCount))

            for i in range(self.backupCount - 1, 0, -1):
                if i < self.backupCount:
                    try:
                        os.rename(
                            self.filename + ".{0}".format(i),
                            self.filename + ".{0}".format(i + 1),
                        )
                    except OSError:
                        print("LOG error")
                        pass

            os.rename(self.filename, self.filename + ".1")
            self._counter = 0

        with open(self.filename, "a") as f:
            print("file opened")
            f.write(msg + "\n")

        self._counter += s_len
