#!/usr/bin/env python
# license removed for brevity
import rospy
import json
import time
import struct
import serial
from std_msgs.msg import UInt32
from std_msgs.msg import String

class JsonSerial:
    'Class instance for publishing json packets received on serial port'

    def __init__(self):
        self.s_devname = '/dev/ttyUSB0'
        self.s = 0
        self.id = -1

    def init_device(self):
        # Try to open the given device name
        try:
            self.s = serial.Serial(self.s_devname)
        except serial.serialutil.SerialException:
            rospy.logwarn('Could not open ' + self.s_devname)
            return False
        
        # Force reset of arduino
        self.s.setDTR(False)
        rospy.sleep(1.)
        self.s.flushInput()
        self.s.setDTR(True)

        # Reopen serial
        self.s = serial.Serial(self.s_devname,baudrate=115200,timeout=1)
        rospy.loginfo('Serial port opened:' + self.s.name)
        self.s.flush()
        return True
        
    # Sets the antenna delay, takes 32bit word
    # - top 16 bits is our address if it matches,
    # - lower 16 bits is the new antenna delay
    def callback_setAntennaDelay(self, d):
        if (((d.data & 0xffff0000) >> 16) != self.id): return;
        self.slave_antenna_delay = d.data
        s=b'a'+struct.pack('>H',self.slave_antenna_delay).encode('hex').upper()
        rospy.loginfo(b'Sending new antenna delay: ' + s)
        self.s.write(s + b'\r');
        rospy.loginfo(rospy.get_caller_id() + " New antenna delay %d", self.slave_antenna_delay)

        
    # Initiate TWR, takes 32bit word
    # - top 16 bits is from address,
    # - lower 16 bits is destination address
    def callback_starttwrwith(self, d):
        if (((d.data & 0xffff0000) >> 16) != self.id): return;
        s=b't'+struct.pack('>H', (d.data&0xffff)).encode('hex').upper()
        self.s.write(s + b'\r');
        rospy.loginfo("Init TWR ranging: 0x%X -> 0x%X", ((d.data & 0xffff0000) >> 16), (d.data & 0xffff))
        
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
        rospy.Subscriber("antenna_dly", UInt32, self.callback_setAntennaDelay)
        rospy.Subscriber("start_twr_between", UInt32, self.callback_starttwrwith)

        # Open our serial port
        self.s_devname = rospy.get_param('~serial_device','/dev/ttyUSB0')
        okinit = self.init_device()
        if (okinit == False): exit(0);
        
        # Once opened ok enter an eternal loop
        while not rospy.is_shutdown():
            # We don't know who's on the other end yet?
            if self.id < 0 : self.s.write(b'?\r')
            
            retcode,jsonblob = self.readline(5)
            if retcode != 0: continue        # Ignore timeout or error
            if len(jsonblob) == 0: continue  # Ignore empty packet
            if jsonblob[0] == "#": continue  # Ignore debug / human readable output

            # Verify that this is interpretable json
            try:
                d = json.loads(jsonblob)
            except ValueError:
                continue

            # Do we know our own id?
            if self.id < 0:
                try:
                    if d['id'] > -1:
                        self.id=int(d['id'],16);
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
