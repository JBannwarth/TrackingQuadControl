""" @package arduino_serial_interface
Documentation.

More details.

@author Jeremie Xavier Joseph Bannwarth
"""

import math
import time
import serial
import struct

class ardu_rc_serial_interface:
    def __init__(self,
                 number_channels = 8,
                 min_space_width = 670,
                 default_space_width = 1080,
                 max_space_width = 1465,
                 serial_port = "/dev/ttyACM0",
                 baud_rate = 57600):
        # Object variables
        self.min_space_width = min_space_width
        self.default_space_width = default_space_width
        self.max_space_width = max_space_width
        self.number_channels = number_channels
        self.channels = [default_space_width]*number_channels

        # Serial setup
        self.ser = serial.Serial(serial_port, baud_rate)

    def send_data(self):
        str_to_print = 's' + ','.join(map(str, self.channels))
        print str_to_print
        self.ser.write(str_to_print)

    def update_channel(self,
                       channel_number,
                       channel_value):
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
        for channel_number, channel_value in enumerate(channel_values):
            self.update_channel(channel_number, channel_value)

    def print_channels(self):
        str_to_print = 's' + ','.join(map(str, l))
        print str_to_print

    def close():
        self.ser.close()


# ser = serial.Serial("/dev/ttyACM0",57600)
# print ser.name
# arduInterface = ardu_rc_serial_interface()
# val = 700
# l = [0]*8
# while True:
#     val = val + 1
#     if (val > 1500):
#         val = 700
#     l = [val]*8
#     arduInterface.update_channels(l)
#     arduInterface.send_data()
#     # arduInterface.print_channels()
#     # strToPrint = 's' + ','.join(map(str, l))
#     # print strToPrint
#     # ser.write(strToPrint)
#     # del strToPrint
#     time.sleep(0.0225)
#     # ser.flush()
#
# arduInterface.close()
