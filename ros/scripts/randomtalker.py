#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from lps.msg import LPSRange
import random

import CTB

def talker():
    s = CTB.CobsSerial('/dev/ttyUSB0', 115200)

    pub = rospy.Publisher('lps/rangedemo', LPSRange, queue_size=10)
    rospy.init_node('randomtalker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = LPSRange(random.randint(0, 3),random.randint(0, 9),random.randint(500,10000),83)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
