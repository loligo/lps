#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import serial
from std_msgs.msg import String

class JsonSerial:
    'Class instance for publishing json packets received on serial port'

    def __init__(self):
        self.s_devname = '/dev/ttyUSB0'
        self.s = 0

    def readline(self,timeout):
        self.s.timeout=timeout
        try:
            l=self.s.readline().rstrip('\n')
            rospy.logdebug(b'readline(' + self.s_devname + b'): ')
        except serial.SerialException as e:
            rospy.logdebug(b'readline(' + self.s.name + b'): exception')
            return -10,''

        return 0,l
        

    def run(self):
        rospy.init_node('jsonserial', anonymous=True, log_level=rospy.INFO)
        pub = rospy.Publisher('uwbjson', String, queue_size=100)

        self.s_devname = rospy.get_param('serial_device','/dev/ttyUSB0')

        # Force reset
        self.s = serial.Serial(self.s_devname)
        self.s.setDTR(False)
        rospy.sleep(1.)
        self.s.flushInput()
        self.s.setDTR(True)

        # Reopen serial
        self.s = serial.Serial(self.s_devname,baudrate=115200,timeout=1)
        rospy.loginfo('Serial port opened:' + self.s.name)
        self.s.flush()

        count = 0
        # Eternal loop
        while not rospy.is_shutdown():
            retcode,jsonblob = self.readline(5)
            if retcode != 0: continue

            #msg = String()
            #msg.header.stamp=rospy.Time.now()
            #msg.header.seq=count
            #msg.d = jsonblob
            count+=1
            pub.publish(jsonblob)
            

if __name__ == '__main__':
    
    try:
        s = JsonSerial()
        s.run()
    except rospy.ROSInterruptException:
        pass              
