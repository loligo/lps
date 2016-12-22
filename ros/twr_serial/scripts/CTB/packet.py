#!/usr/bin/env python
# -*- encoding: utf-8 -*-

#
# (C) 2015, Ciellt AB/Stefan Petersen (spe@ciellt.se)
#

import cobs
import crc

class Packet(object):
    _type = 0x00

    def __init__(self, to_addr, from_addr, data = None):
        self._packet = [self._type]
        self._packet.extend(self._ntoh16(to_addr))
        self._packet.extend(self._ntoh16(from_addr))
        if data:
            self._packet.extend(self._ntoh16(len(data)))
            self._packet.extend(data)
        else:
            self._packet.extend([0, 0])

    def _ntoh16(self, data):
        return  [ data & 0xff , (data >> 8) & 0xff ]

    def _ntoh32(self, data):
        return [data & 0xff ,  (data >> 8) & 0xff, \
            (data >> 16) & 0xff , (data >> 24) & 0xff ]

    def packet(self):
        return self._packet

    def packet_as_str(self):
        return "".join(map(chr, self._packet))

    def packet_crc_cobs(self):
        mycrc = crc.crc(self.packet_as_str())
        data = cobs.encode(self.packet_as_str() + \
                           chr(mycrc & 0xff) + chr((mycrc >> 8) & 0xff))
        data = data + '\x00'
        return data

    def _checksum_ok(self, in_bytes):
        calculated_crc = crc.crc(in_bytes[:-2])
        received_crc = ((ord(in_bytes[-1]) << 8) | ord(in_bytes[-2]))
        return calculated_crc == received_crc

    def _decode(self, in_bytes):
        decoded = cobs.decode(in_bytes)
        if self._checksum_ok(decoded):
            return map(ord, decoded[:-2])
        else:
            return None

    def decode(self, in_bytes):
        return self._decode(in_bytes)

if __name__ == '__main__':

    mp = Packet(0x763c, 0x0000, [0x01, 0x02, 0x03])
    reference_packet = [0x01, 0x03, 0x3c, 0x76, 0x01, 0x02, 0x03, 0x06, \
                        0x01, 0x02, 0x03, 0x71, 0x9b, 0x00]

    packet1 = ''.join(map(chr, reference_packet))
    packet2 = mp.packet_crc_cobs()

    if packet1 == packet2:
        print("OK");
    else:
        print("Fail")
