#!/usr/bin/env python
# license removed for brevity
import rospy
from lps.msg import LPSRange
import CTB
import time
import random

class Get_Range(CTB.Packet):
    _type = 0x90

    def __init__(self, to_addr, from_addr):
        CTB.Packet.__init__(self, to_addr, from_addr)

    def decode(self, in_bytes):
        packet = self._decode(in_bytes)
        response = {}
        response['type'] = packet[0]
        response['to'] = (packet[2] * 256 + packet[1])
        response['from'] = (packet[4] * 256 + packet[3])
        response['anchorid'] = (packet[8] * 256 + packet[7])
        response['distmm'] = (packet[12] * 0x1000000 + packet[11] * 0x10000 +
                                packet[10] * 256 + packet[9])
        response['power'] = ( packet[14] * 256 + packet[13])

        return response


def talker():
    s = CTB.CobsSerial('/dev/ttyUSB0', 115200)
    time.sleep(2.0)

    pub = rospy.Publisher('lps/ranges', LPSRange, queue_size=10)
    rospy.init_node('serialtalker', anonymous=True)

    rate = rospy.Rate(1) # 10hz
    get_range_packet = Get_Range(0x1000, 0x0000)

    s.send_binary('\x00')
    while not rospy.is_shutdown():
        try:
            answer = get_range_packet.decode(s.read_cobs())
        except:
            s.send_binary('\x00')
            time.sleep(0.01)
            continue
        rospy.loginfo(answer)
        if answer['anchorid'] == 0xffff : continue
        rospy.loginfo(answer)
        msg = LPSRange(answer['from'],answer['anchorid'],answer['distmm'],-answer['power']/10.0)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
