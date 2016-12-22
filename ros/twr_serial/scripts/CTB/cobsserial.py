#!/usr/bin/env python
# -*- encoding: utf-8 -*-

#
# (C) 2015, Ciellt AB/Stefan Petersen (spe@ciellt.se)
#

import serial

class CobsSerial(serial.Serial):
    """Extends original serial.Serial with read_cobs()
    """

    def __init__(self, *args, **kwds):
        serial.Serial.__init__(self, *args, **kwds)

    def read_cobs(self):
        """Read until \0 (COBS packet terminator) is found"""

        char = 0xff
        tmp = []
        while char != '\0':
            char = self.read(1)
            # End Of Something
            if len(char) == 0:
                break
            tmp.append(char)

        # Remove last element, since that is the terminator
        tmp.pop()

        return ''.join(tmp)

    def send_binary(self, str):
        """Send binary string."""
        for d in str:
            self.write(d)
