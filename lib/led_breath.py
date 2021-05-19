import utime as time
import pycom
import math


class LedBreath(object):
    """
    Class LedBreath lets the RGB LED from the PyCom module breath
    according to the time ticks.
    If the breathing does not work for a while, it means the controller is not running
    """

    def __init__(self):
        self.period = 5000.0
        self.color = 0xFF00FF
        self.brightness = 0xFF
        # storage for backup values, used for blinking method
        self.period_back = self.period
        self.color_back = self.color
        self.brightness_back = self.brightness

    def update(self):
        """
        Update the breathing, means to calculate the new intensity value of the light
        and set the RGB LED accordingly.
        The breathing follows a sinewave with a period of self.period
        and the time is controlled by time.ticks_ms().
        """
        # calculate the intensity
        _intensity = self.brightness / 512.0 * (math.sin(time.ticks_ms() / self.period * 2 * math.pi) + 1)
        if _intensity < 0.1:
            _intensity = 0.1
        # split the color into the RGB components
        _red = self.color >> 16 & 0xFF
        _green = self.color >> 8 & 0xFF
        _blue = self.color & 0xFF
        # combine the intensity and the colors into the new light value
        _light = (int(_intensity * _red) << 16) + \
                 (int(_intensity * _green) << 8) + \
                 (int(_intensity * _blue))
        # set the RGBLED to the new value
        pycom.rgbled(_light)

    def set_color(self, color):
        """
        set_color is used to set the color of the RGB LED and update it directly.
        This color will stay until it is changed.
        :param color: is the color value in R,G,B
        """
        self.color = color & 0xFFFFFF
        self.update()

    def set_brightness(self, brightness):
        """
        set_brightness is used to set the brightness of the RGB LED light at maximum.
        The default value is 0xFF
        :param brightness: is the brightness at maximum of the breathing interval
        """
        self.brightness = brightness & 0xFF
        self.update()

    def set_period(self, period_ms):
        """
        set_period is used to set the period [ms] of the RGB LED light breathing.
        The default value is 5000 ms
        :param period_ms: is the period [ms] of the breathing interval
        """
        self.period = period_ms
        self.update()

    def set_blinking(self):
        """
        set_blinking is used to set the LED breathing to a fast a bright mode.
        This Method shall be used to make the sensor visible and recognizable.
        """
        # first backup the old values
        self.period_back = self.period
        self.color_back = self.color
        self.brightness_back = self.brightness
        # now set a bright red color with 1000 ms period
        self.period = 1000
        self.color = 0xFF8010
        self.brightness = 0xFF
        self.update()

    def reset_blinking(self):
        """
        reset_blinking is used to set the LED breathing to a fast a bright mode.
        This Method shall be used to reset the .
        """
        # get back the backuped values
        self.period = self.period_back
        self.color = self.color_back
        self.brightness = self.brightness_back
        self.update()