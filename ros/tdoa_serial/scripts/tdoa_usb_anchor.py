#!/usr/bin/env python
# license removed for brevity
import rospy
import json
import time
import struct
import serial
import numpy as np
from std_msgs.msg import UInt32
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import FluidPressure
from lps_messages.msg import LPSSyncedJson

import LocalClock

class TdoaSerialAnchor:
    'Class driver for the LPS Tdoa Anchor on a USB UART'

    def __init__(self):
        self.s_devname = '/dev/ttyUSB0'
        self.s = 0
        self.id = -1
        self.lc = LocalClock.LocalClock(self.id)

    def init_device(self):
        # Try to open the given device name
        try:
            self.s = serial.Serial(self.s_devname)
        except serial.serialutil.SerialException:
            rospy.logwarn('Could not open ' + self.s_devname)
            return False
        
        # Force reset of LPS
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

    # Extracting data from the hex coded packet
    #
    def extract_seq_id(self,d):
        o = 2*2
        return int(d[o:(o+2)],16)

    def extract_packet_type(self,d):
        o = 9*2
        return int(d[o:(o+2)],16)

    def extract_source_address(self,d):
        o = 7*2
        return int(d[(o+2):(o+4)],16)*256 + int(d[o:(o+2)],16)

    def extract_dest_address(self,d):
        o = 5*2
        return int(d[(o+2):(o+4)],16)*256 + int(d[o:(o+2)],16)
    
    def extractts(self,d,offset):
        ts=np.uint64(0)
        o = offset*2
        for i in range(0,5):
            start=o+i*2
            end=start+2
            ts = ts + (int(d[start:end],16) << i*8)
        return ts

    # Header     ts       tp q0       q1       q2       q3       gx   gy   gz   mx   my   mz   pressure temp     crc
    # 0          5        9  10       14       18       22       26   28   30   32   34   36   38       42
    # 4188e51011 ffff0500 30 d8d94314 ffa4baf5 fffffffe a95b11fe 2b71 a636 14fd 3bff aaff 7aff c5880100 d70a0000 6b6a
    def extractImuData(self,source_address,d):
        msg = Imu()
        msg.header.stamp=rospy.Time.now()
        msg.header.frame_id='imu' + format(source_address,'x')
        #quarternion
        quat_sens=1073741824.0
        q=[]
        o = 10*2
        for j in range(0,4):
            v = np.uint32(0)
            for i in range(0,4):
                start=o+i*2
                end=start+2
                v = v + (int(d[start:end],16) << i*8)
            o+=4*2
            v = np.int32(v)
            q.append(float(v)/quat_sens) #
        msg.orientation.w=q[0]
        msg.orientation.x=q[1]
        msg.orientation.y=q[2]
        msg.orientation.z=q[3]
        for i in xrange(len(msg.orientation_covariance)):
            msg.orientation_covariance[i]=0.0
        # Gyros
        gyro_sens=16.375
        g=[]
        o = 26*2
        for j in range(0,3):
            v = np.uint16(0)
            for i in range(0,2):
                start=o+i*2
                end=start+2
                v = v + (int(d[start:end],16) << i*8)
            o+=2*2
            v = np.int16(v)
            g.append(float(v)/gyro_sens)
        msg.angular_velocity.x=g[0]
        msg.angular_velocity.y=g[1]
        msg.angular_velocity.z=g[2]
        for i in xrange(len(msg.angular_velocity_covariance)):
            msg.angular_velocity_covariance[i]=0.0
        
        # Accelerometers
        for i in xrange(len(msg.linear_acceleration_covariance)):
            msg.linear_acceleration_covariance[i]=-1.0

        return msg
        

    def extractPressure(self,source_address,d):
        msg = FluidPressure()
        msg.header.stamp=rospy.Time.now()
        msg.header.frame_id='imu' + str(source_address)
        # Barometer
        o = 38*2
        v = np.uint32(0)
        for i in range(0,4):
            start=o+i*2
            end=start+2
            v = v + (int(d[start:end],16) << i*8)
        o+=4*2
        msg.fluid_pressure = np.int32(v)*1.0
        msg.variance = 0
        return msg

    def extractTemperature(self,source_address,d):
        msg = Temperature()
        msg.header.stamp=rospy.Time.now()
        msg.header.frame_id='imu' + str(source_address)
        # Barometer
        o = 42*2
        v = np.uint32(0)
        for i in range(0,4):
            start=o+i*2
            end=start+2
            v = v + (int(d[start:end],16) << i*8)
        o+=4*2
        msg.temperature = np.int32(v)/100.0
        msg.variance = 0
        return msg
    
    ######
    
    def updateClockRef(self,d):
        try:
            master_clock = self.extractts(d['d'].replace(' ',''),10)
        except KeyError:
            return

        local_clock = int(d['ts'],16)
        self.lc.addDataPt(master_clock, local_clock)
        self.lc.update()

        
    def run(self):
        rospy.init_node('tdoaserialanchor', anonymous=True, log_level=rospy.INFO)
        pub = rospy.Publisher('synced_uwb_json', LPSSyncedJson, queue_size=100)
        imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        temp_pub = rospy.Publisher('/imu/temperature', Temperature, queue_size=10)
        pressure_pub = rospy.Publisher('/imu/pressure', FluidPressure, queue_size=10)
        rospy.Subscriber("antenna_dly", UInt32, self.callback_setAntennaDelay)
        rospy.Subscriber("start_twr_between", UInt32, self.callback_starttwrwith)

        # Open our serial port
        self.s_devname = rospy.get_param('~serial_device','/dev/ttyUSB2')
        okinit = self.init_device()
        if (okinit == False): exit(0);
        
        # Once opened ok enter an eternal loop
        clock_sync_initiated = False
        twr_throttle = 10
        last_twr_time = rospy.Time(0).secs
        while not rospy.is_shutdown():
            # We don't know who's on the other end yet?
            if self.id < 0 : self.s.write(b'?\r')
            if clock_sync_initiated==False and self.id==0:
                rospy.loginfo("Initiating clock sync on anchor 0")
                clock_sync_initiated = True
                self.s.write(b'm0064\r')

            # If we don't have an estimation of our tof offset
            # order a twr with master
            twr_throttle -= 1
            if self.id != 0 and self.lc.tof_offset==0 and twr_throttle<1:
                rospy.loginfo("Order TWR with master - initial")
                self.s.write(b't0000\r')
                twr_throttle = 10

            if self.id != 0 and self.lc.tof_offset!=0 and rospy.Time(0).secs - last_twr_time > 10:
                rospy.loginfo("Order TWR with master - regular")
                self.s.write(b't0000\r')
                
            retcode,jsonblob = self.readline(1)
            if retcode == -10:
                rospy.loginfo("timeout")
                self.s.write(b'?\r')
                continue;
            if retcode != 0: continue        # Ignore timeout or error
            if len(jsonblob) == 0: continue  # Ignore empty packet
            rospy.loginfo("packet: '%s'", jsonblob)

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
                        self.id = int(d['id'],16)
                        self.lc.id = self.id
                        rospy.loginfo("My id: %d", self.id)
                except AttributeError:
                    print(d)

            try:
                ts = int(d['ts'],16)
                rssi = int(d['rssi'],10)
                if len(d['d']) > 1024: continue;
            except KeyError:
                print(d)
                continue
            
            # Update local clock filter
            try:
                packet_type = self.extract_packet_type(d['d'])
            except KeyError:
                return

            seq_id = self.extract_seq_id(d['d'])
            source_address = self.extract_source_address(d['d'])
            dest_address   = self.extract_dest_address(d['d'])
            
            if packet_type == 0x20 and source_address == 0x0000:
                self.updateClockRef(d)
                continue            
            if packet_type == 0x30 and len(d['d']) > 93:
                msg = self.extractImuData(source_address,d['d'])
                imu_pub.publish(msg)
                msg = self.extractPressure(source_address,d['d'])
                pressure_pub.publish(msg)
                msg = self.extractTemperature(source_address,d['d'])
                temp_pub.publish(msg)
            
            ts_adj=self.lc.masterat(ts)

                
            # Check for twr messages that can give us tof to master
            # to futher adjust our timings
            try:
                tof = int(d['tof'],16)
                tof_mm = int(d['tof_mm'],10)
                if source_address == 0x0000 and dest_address == self.id and tof_mm < 300000:
                    self.lc.tof_offset = tof/1000.0
                    rospy.loginfo("TOF Offset updated: 0x%X (%.0fmm)", tof, tof_mm)
                if source_address == self.id and dest_address == 0x0000 and tof_mm < 300000:
                    self.lc.tof_offset = tof/1000.0
                    rospy.loginfo("TOF Offset updated: 0x%X (%.0fmm)", tof, tof_mm)
            except KeyError:
                tof = 0
                
            # Publish message as is
            msg = LPSSyncedJson()
            msg.header.stamp=rospy.Time.now()
            msg.listener_id = self.id
            msg.seq_id = seq_id
            msg.source_addr = source_address
            msg.dest_addr = dest_address
            msg.ts = ts
            msg.ts_adj = int(ts_adj)
            msg.rssi = rssi
            msg.twr = tof*1.0/499.2e6/128.0*299702547.0
            msg.data = str(d['d'])
            try:
                pub.publish(msg)
            except rospy.exceptions.ROSSerializationException:
                print(msg)
                

if __name__ == '__main__':
    try:
        s = TdoaSerialAnchor()
        s.run()
    except rospy.ROSInterruptException:
        pass              
