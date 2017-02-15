#!/usr/bin/env python
# license removed for brevity
import json
import time
import struct
import serial
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
        
    def process_vector(self, v):
        print("#-----------------#")
        t0=-1
        for i in v:
            if i.listener_id==0: t0=i.ts_adj
        if t0 < 0: return
        rospy.loginfo("  t0: %d", t0)

        Amatrix = np.zeros((len(v),3))
        Bmatrix = np.zeros((len(v),1))
        r=0
        for i in v:
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/map', '/anchor' + str(i.listener_id), i.header.stamp)
                #print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            ts = i.ts_adj-t0
            t = (i.ts_adj-t0)/(499.2e6*128.0)
            c = scipy.constants.c
            d = 0
            if i.ts_adj == t0:
                A=0
                B=0
                C=0
                D=0
            else:
                d = c*t
                divisor = 1.0/(c*t)
                A=2*trans[0]*divisor
                B=2*trans[1]*divisor
                C=2*trans[2]*divisor
                D=t*c-(trans[0]*trans[0]+trans[1]*trans[1]+trans[2]*trans[2])*divisor
            print(i.listener_id,i.source_addr,i.seq_id,ts,d)
            Amatrix[r,0] = A;
            Amatrix[r,1] = B;
            Amatrix[r,2] = C;
            Bmatrix[r,0] = -D;
            r+=1
        #print (Amatrix,Bmatrix)
        x = np.linalg.lstsq(Amatrix, Bmatrix)[0]
        print (x)
        self.tf_broadcaster.sendTransform((x[0], x[1], x[2]),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     'imu' + format(i.source_addr,'x'),
                     "map")
        
                
            
    def callback_lpssyncedjson(self, d):
        if d.seq_id != self.current_seq_id:
            self.current_seq_id = d.seq_id
            self.last_complete_vector = self.current_vector
            self.current_vector = []
            self.process_vector(self.last_complete_vector)
        self.current_vector.append(d)
        
        
    def run(self):
        rospy.init_node('tdoaserialanchor', anonymous=True, log_level=rospy.INFO)
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
