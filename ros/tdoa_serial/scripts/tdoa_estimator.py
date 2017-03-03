#!/usr/bin/env python
# license removed for brevity
import json
import time
import struct
import serial
import math
import numpy as np
import scipy
from scipy import optimize
from scipy import constants

import rospy
import tf
from std_msgs.msg import UInt32
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import FluidPressure
from lps_messages.msg import LPSSyncedJson

class TdoaEstimator:
    'Tdoa Estimator'

    def __init__(self):
        self.current_vector = []
        self.last_complete_vector = []
        self.current_seq_id = -1;
        self.tf_listener = 0
        self.tf_broadcaster = 0

    def tsdiff(self,a,b):
        d=a-b
        if d> 0xFF00000000: d-=0xFFFFFFFFFF
        if d<-0xFF00000000: d+=0xFFFFFFFFFF
        if d> 0xFF000000:   d-=0xFFFFFFFF
        if d<-0xFF000000:   d+=0xFFFFFFFF
        return d
        

    def process_vector(self, v):
        if len(v) < 4: return
        v.sort(lambda x,y: x.listener_id-y.listener_id)

        c = scipy.constants.c
        
        # The first anchor in the list becomes a0
        a0=v[0]
        ts0=a0.ts_adj
        t0=ts0/(499.2e6*128.0)
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/anchor' + str(a0.listener_id), v[0].header.stamp)
            a0_trans=np.array(trans)
            rospy.logdebug("%d[%x]: sq=%3d d=%.3f len=%d",a0.listener_id,a0.source_addr,a0.seq_id,0,len(v))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # The second anchor in the list becomes a1
        a1=v[1]
        if (a1.ts_adj==0): return;
        
        ts1=self.tsdiff(a1.ts_adj,ts0)
        t1=ts1/(499.2e6*128.0)
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/anchor' + str(a1.listener_id), v[1].header.stamp)
            a1_trans=np.array(trans)-a0_trans
            rospy.logdebug("%d[%x]: d=%.3f",a1.listener_id,a1.source_addr,t1*c)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Remaining anchors form the equations
        Amatrix = np.zeros((len(v)-2,3))
        Bmatrix = np.zeros((len(v)-2,1))
        r=0
        for i in range(2,len(v)):
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/map', '/anchor' + str(v[i].listener_id), v[i].header.stamp)
                trans=np.array(trans)-a0_trans
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            ts = self.tsdiff(v[i].ts_adj,ts0)
            if ts==0: ts=1
            t = ts/(499.2e6*128.0)
            d = c*t
            divisor = 1.0/(c*t)
            A=2*trans[0]*divisor - 2*a1_trans[0]/(t1*c)
            B=2*trans[1]*divisor - 2*a1_trans[1]/(t1*c)
            C=2*trans[2]*divisor - 2*a1_trans[2]/(t1*c)
            D=c*t - t1*c - (trans[0]*trans[0]+trans[1]*trans[1]+trans[2]*trans[2])*divisor + (a1_trans[0]*a1_trans[0]+a1_trans[1]*a1_trans[1]+a1_trans[2]*a1_trans[2])/(t1*c)

            rospy.logdebug("%d[%x]: d=%.3f",v[i].listener_id,v[i].source_addr,t*c)
            Amatrix[r,0] = A;
            Amatrix[r,1] = B;
            Amatrix[r,2] = C;
            Bmatrix[r,0] = -D;
            r+=1
        (x,residuals,rank,s) = np.linalg.lstsq(Amatrix, Bmatrix)
        x=np.transpose(x[:,0])+a0_trans
        if rank < 2: return
        rospy.loginfo("x[%x]=(%.3f,%.3f,%.3f)",a0.source_addr,x[0], x[1], x[2])
        if math.isnan(x[0]) or math.isnan(x[1]): return
        self.tf_broadcaster.sendTransform((x[0], x[1], x[2]),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     'imu' + format(v[i].source_addr,'x'),
                     "map")
        
                
            
    def callback_lpssyncedjson(self, d):
        #Todo: Handle multiple tags
        if d.source_addr < 0x1000: return
        if d.seq_id != self.current_seq_id:
            self.current_seq_id = d.seq_id
            self.last_complete_vector = self.current_vector
            self.current_vector = []
            self.process_vector(self.last_complete_vector)
        self.current_vector.append(d)
        
        
    def run(self):
        rospy.init_node('tdoaserialanchor', anonymous=True, log_level=rospy.DEBUG)
        pub = rospy.Publisher('synced_uwb_json', LPSSyncedJson, queue_size=100)
        pressure_pub = rospy.Publisher('/imu/pressure', FluidPressure, queue_size=10)
        rospy.Subscriber("synced_uwb_json", LPSSyncedJson, self.callback_lpssyncedjson)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # enter an eternal loop
        rospy.spin()
            

if __name__ == '__main__':
    try:
        s = TdoaEstimator()
        s.run()
    except rospy.ROSInterruptException:
        pass              
