#!/usr/bin/env python
# license removed for brevity
import rospy
import json
import time
import struct
import serial
from std_msgs.msg import UInt16
from std_msgs.msg import String

class JsonSerial:
    'Class instance for publishing json packets received on serial port'

    def __init__(self):
        self.s_devname = '/dev/ttyUSB0'
        self.s = 0
        self.id = -1

    def init_device(self):
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
        
    def callback_setAntennaDelay(self, d):
        self.slave_antenna_delay = d.data
        s=b'a'+struct.pack('>H',self.slave_antenna_delay).encode('hex').upper()
        rospy.loginfo(b'Sending new antenna delay: ' + s)
        self.s.write(s + b'\r');
        rospy.loginfo(rospy.get_caller_id() + " New antenna delay %d", self.slave_antenna_delay)
        
    def readline(self,timeout):
        self.s.timeout=timeout
        try:
            l=self.s.readline().rstrip('\n')
        except serial.SerialException as e:
            rospy.logdebug(b'readline(' + self.s.name + b'): exception')
            return -10,''

        return 0,l
        
    
    def run(self):
        rospy.init_node('jsonserial', anonymous=True, log_level=rospy.INFO)
        pub = rospy.Publisher('uwbjson', String, queue_size=100)
        rospy.Subscriber("antenna_dly", UInt16, self.callback_setAntennaDelay)

        self.s_devname = rospy.get_param('~serial_device','/dev/ttyUSB0')
        self.init_device()
        
        # Eternal loop
        started_clockref = False
        while not rospy.is_shutdown():
            # We don't know who's on the other end yet?
            if self.id < 0 : self.s.write(b'?\r')
            if self.id == 0 and started_clockref==False: self.s.write(b'm0064\r')
            
            retcode,jsonblob = self.readline(5)
            if retcode != 0: continue
            if len(jsonblob) == 0: continue
            if jsonblob[0] == "#": continue

            # Verify that this is json
            try:
                d = json.loads(jsonblob)
            except ValueError:
                continue

            if self.id < 0:
                #extract out id
                try:
                    if d['l_id'] > -1:
                        self.id=int(d['l_id'],16);
                        rospy.loginfo("My id: %d", self.id)
                except AttributeError:
                    print(d)

            # Publish message as is
            pub.publish(jsonblob)
            

if __name__ == '__main__':
    try:
        s = JsonSerial()
        s.run()
    except rospy.ROSInterruptException:
        pass              
