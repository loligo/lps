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
from lps_messages.msg import LPSRange

class LinearDelayAutocalibrator:
    'Used to calibrate the antenna delay by placing all anchors'
    ' in a single straight line'

    def __init__(self):
        self.twr = []
        self.start_ant_delay = 0x4000

    def getDistanceEstimate(self,i,j):
        a = np.average(self.twr[i][j])
        b = np.average(self.twr[j][i])
        rospy.logdebug(" distEst 0x%x <-> 0x%x = %.1f (mm) (%f,%f)", i, j, 0.5*(a+b), a, b)
        return 0.5*(a+b)
        
    def solve_for_delays(self, nanchors):
        neq = nanchors*(nanchors-1)/2 + nanchors
        Amatrix = np.zeros((neq,nanchors*2))
        Bmatrix = np.zeros((neq,1))

        r=0
        # Lock down A0's x to 0
        Amatrix[r,0] = 1
        Bmatrix[r,0] = 0
        r+=1
        # Lock down other available anchors
        for i in range(1,nanchors):
            try:
                t = self.tf_listener.getLatestCommonTime("/map", '/anchor' + str(i))
                (trans,rot) = self.tf_listener.lookupTransform('/map', '/anchor' + str(i), t)
                Amatrix[r,0] = -1
                Amatrix[r,i*2] = 1
                Bmatrix[r,0] = trans[0]*1000 # Recalculate into mm
                r+=1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                continue

        # Twr + delay equations
        for i in range(1,nanchors):
            for j in range(0,i):
                d = self.getDistanceEstimate(i,j)
                if (math.isnan(d)): continue
                Amatrix[r,2*i] = 1   # The higher id anchor's x
                Amatrix[r,2*j] = -1  # The lower  id anchor's x
                Amatrix[r,2*i+1] = 1 # delay
                Amatrix[r,2*j+1] = 1 # delay
                Bmatrix[r,0] = d
                r+=1

        (x2,residuals,rank,s) = np.linalg.lstsq(Amatrix[1::,1::], Bmatrix[1::])
        x= np.insert(x2, 0, 0, axis=0)
        return (x,(rank>=(2*nanchors-1)))
                
            
    def callback_twrcomplete(self, d):
        rospy.logdebug(" twr result 0x%x <-> 0x%x = %d (mm)", d.anchor_id, d.tag_id, d.dist_mm)
        self.twr[d.anchor_id][d.tag_id].append(d.dist_mm)
        
        
    def run(self):
        rospy.init_node('tdoaserialanchor', anonymous=True, log_level=rospy.DEBUG)
        twr_pub = rospy.Publisher("start_twr_between", UInt32, queue_size=100)
        power_pub = rospy.Publisher("tx_power", UInt32, queue_size=1)
        antenna_dly_pub = rospy.Publisher("antenna_dly", UInt32, queue_size=1)
        self.tf_listener = tf.TransformListener()

        maxnum_anchors = rospy.get_param('number_of_anchors_to_calibrate',4)
        for i in range(0,maxnum_anchors):
            self.twr.append([])
            for j in range(0,maxnum_anchors):
                self.twr[i].append([])

        rospy.loginfo("Waiting for anchors to start up...")
        rospy.sleep(5)
        
        rospy.loginfo("Setting base txpower to 0xAF...")
        power_pub.publish(0xFFFF00AF);
        rospy.sleep(0.5)

        rospy.loginfo("Temporarily resetting antenna delay to 0x%x...", self.start_ant_delay)
        rospy.set_param('anchor_store_eeprom',False)
        rospy.sleep(0.5)
        antenna_dly_pub.publish(0xFFFF0000 | self.start_ant_delay);
        rospy.sleep(2)

        # Start listening for twr messages
        rospy.Subscriber('twr', LPSRange, self.callback_twrcomplete)
        
        num_total_twr = rospy.get_param('calibration_twr_runs',50)
        rospy.loginfo("Performing twr between %d anchors %d times...", maxnum_anchors, num_total_twr)
        for k in range(0,num_total_twr):
            for i in range(1,maxnum_anchors):
                for j in range(0,i):
                    rospy.sleep(0.05)
                    rospy.logdebug("twr order %d -> %d", i,j)
                    twr_pub.publish((i<<16)|j)

        # Solve the equationsystem
        (x,solvedOk) = self.solve_for_delays(maxnum_anchors)
        mm2tof = (499.2e6*128.0)/299702547.0/1000.0
        
        rospy.loginfo("Results from the solver:")
        ok_to_set = solvedOk
        for i in range(0,len(x)/2):
            rospy.loginfo(" id:%2d, x:%7.1fmm, delay adj:%4.0fmm or %4.0f in dw units", i, x[2*i], x[2*i+1], x[2*i+1]*mm2tof)
            if math.isnan(x[2*i+1]): ok_to_set = False

            
        if ok_to_set:
            rospy.set_param('anchor_store_eeprom',True)
            rospy.sleep(0.5)
            rospy.loginfo("Antenna delay estimation successful, writing new delays to anchors... ")
            for i in range(0,len(x)/2):
                newdly = int(self.start_ant_delay + (x[2*i+1]*mm2tof))
                antenna_dly_pub.publish((i << 16)| newdly);
                rospy.loginfo(" id:%2d, new delay:0x%.4X", i, newdly)
            rospy.loginfo("All done :)")
        else:
            rospy.logwarn("Antenna delay estimation failed. ")
            rospy.logwarn("Check that you've provided enough coordinates for anchors in the launch file.")
                
        return 
            

if __name__ == '__main__':
    try:
        s = LinearDelayAutocalibrator()
        s.run()
    except rospy.ROSInterruptException:
        pass              
