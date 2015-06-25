""" @package arduino_serial_interface
Handle communication with RC transmitter through Arduino interface.

This package is developed for use with the motion capture system in the Faculty
of Engineering at the University of Auckland.

@author Jeremie Xavier Joseph Bannwarth
@version 0.1
@date 06/2015
"""

import math
import time
import serial
import struct

class ardu_rc_serial_interface:
    """ Send PPM signal settings to Arduino via serial to control RC transmitter.

    The analog value for each channel is encoded as the spacing (in ms) between
    subsequent pulses.

    Communication is performed over serial by sending comma delimited ASCII
    strings. The data is sent using the following format:

        s<channel1>,<channel2>,<...>,<channelN-1>,<channelN><linereturn>

    where <channelK> is an ASCII string representing the spacing length (in ms)
    encoding channel K.

    Eg. s675,890,500,780<linereturn> for a 4 channel transmitter (N = 4).

    For more information on PPM signals:
    https://en.wikipedia.org/wiki/Pulse-position_modulation
    """
    def __init__(self,
                 number_channels = 8,
                 min_space_width = 670,
                 default_space_width = 1080,
                 max_space_width = 1465,
                 serial_port = "/dev/ttyACM0",
                 baud_rate = 57600):
        """ The constructor.
        """
        # Object variables
        ## Minimum possible spacing between subsequent pulses (in ms).
        self.min_space_width = min_space_width
        ## Default spacing between subsequent pulses (in ms).
        self.default_space_width = default_space_width
        ## Maximum possible spacing between subsequent pulses (in ms).
        self.max_space_width = max_space_width
        ## Number of channels supported by the RC transmitter.
        self.number_channels = number_channels
        ## Pulse spacings for the transmitter channels (in ms).
        self.channels = [default_space_width]*number_channels

        ## Serial connection object.
        self.ser = serial.Serial(serial_port, baud_rate)

    def send_data(self):
        """ Send channel values over serial.
        @param self The object pointer.
        """
        str_to_print = 's' + ','.join(map(str, self.channels))
        print str_to_print
        self.ser.write(str_to_print)

    def update_channel(self,
                       channel_number,
                       channel_value):
        """ Update the period of a single channel.

        Also check that the value is in the allowable range.
        @param self The object pointer.
        @param channel_number The channel identifier number.
        @param channel_value The new spacing period (in ms) to assign to the channel.
        """
        if ( (channel_number >= 0) and ( channel_number < self.number_channels) ):
            # Limit values to correct range
            if (channel_value < self.min_space_width):
                self.channels[channel_number] = self.min_space_width
            elif (channel_value > self.max_space_width):
                self.channels[channel_number] = self.max_space_width
        else:
            self.channels[channel_number] = channel_value

    def update_channels(self,
                        channel_values = []):
        """ Update the values of all channels.
        @param self The object pointer.
        @param channel_values The new spacing periods (in ms) to assign to the channels.
        """
        for channel_number, channel_value in enumerate(channel_values):
            self.update_channel(channel_number, channel_value)

    def print_channels():
        """ Output current channel values to screen for debugging.
        """
        str_to_print = 's' + ','.join(map(str, l))
        print str_to_print

    def close():
        """ Close serial connection.
        """
        self.ser.close()
