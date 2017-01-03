#!/usr/bin/env python
# license removed for brevity
import sys, argparse
import json
import rospy
import time
from std_msgs.msg import String
import numpy as np
import math
import matplotlib.pyplot as plt        

import LocalClock
        
class Plotter:
    'Class instance for analysing uwb timings'

    def __init__(self):
        self.data = []
        rospy.Subscriber('uwbjson', String, self.uwbjson_callback)
        self.lc = [LocalClock.LocalClock(i) for i in range(8)]

    def datalen(self,id=-1):
        return len(self.data)

    def updateClockRef(self,d):
        try:
            master_clock = self.extractts(d['d'].replace(' ',''),10)
        except KeyError:
            return

        print(d)
        local_clock = int(d['ts'],16)

        id = int(d['id'],16)
        if id > len(self.lc): return
        
        self.lc[id].addDataPt(master_clock, local_clock)
        self.lc[id].update()

        
    def adjustClock(self,d):
        id = int(d['id'],16)
        if id > len(self.lc): return
        return self.lc[id].masterat(int(d['ts'],16))
        
    def uwbjson_callback(self,d):
        if len(self.data) > 7505: return
        try:
            a = json.loads(d.data)
        except ValueError:
            return;

        try:
            packet_type = self.extract_packet_type(a['d'])
        except KeyError:
            return

        if packet_type == 0x20:
            self.updateClockRef(a)
        else:
            a['adj_ts']=self.adjustClock(a)
            print a

        self.data.append(a)

    def extractts(self,d,offset):
        ts=np.uint64(0)
        o = offset*2
        for i in range(0,5):
            start=o+i*2
            end=start+2
            ts = ts + (int(d[start:end],16) << i*8)
        #rospy.loginfo("ts: %X", ts)    
        return ts

    def extract_packet_type(self,d):
        o = 9*2
        return int(d[o:(o+2)],16)
    
    def getTsDataForId(self,id):
        remote_ts = []
        local_ts = [] 
        for d in self.data:
            try:
                if self.extract_packet_type(d['d']) != 0x20:
                    continue
            except KeyError:
                continue
            
            if d['id']==id:
                try:
                    master_clock = self.extractts(d['d'].replace(' ',''),10)
                except KeyError:
                    continue;
                print(d)
                local_clock = int(d['ts'],16)
                remote_ts.append(master_clock)
                local_ts.append(local_clock)
                self.lc.addDataPt(master_clock, local_clock)
                self.lc.update()

        return remote_ts,local_ts
        

def main():
    global _toa_filter_limit, _errorplot, _truedist
    parser = argparse.ArgumentParser()
    #parser.add_argument('files', metavar='file', nargs='+')
    #parser.add_argument('-l',dest='labels')
    #parser.add_argument('-v', dest='verbose', action='store_true')
    #parser.add_argument('-d',metavar='distance in mm',type=int,dest='truedist',help='true measured distance in mm')
    #parser.add_argument('-e', dest='plot_errors', action='store_true', help='Plot errors instead of absolute values')
    #parser.add_argument('--twr_limit', metavar='n*stddev', type=float)
    args = parser.parse_args()
    
    fig = plt.figure(num=1)
    plt.clf()

    rospy.init_node('analyse_timings', anonymous=True)
    rospy.loginfo("analyse_timings init")

    plotter=Plotter()
    
    while not rospy.is_shutdown() and plotter.datalen()<500:
        rospy.sleep(1)

    m,l=plotter.getTsDataForId('2')
    
    plt.subplot(3, 1, 1)
    line_ledgends_A=[]
    line_average, = plt.plot(range(len(l)),m-np.min(m),'.', label='listener 1', linewidth=1.0)
    line_ledgends_A.append(line_average)
    plt.legend(handles=line_ledgends_A,loc=4)
    plt.grid(b=True, which='both')
    plt.xlabel('local clock')
    plt.ylabel('master clock')

    plt.subplot(3, 1, 2)
    line_ledgends_A=[]
    
    line_average, = plt.semilogy(range(len(plotter.lc.x_pred_err)),np.abs(plotter.lc.x_pred_err),'.', label='error', linewidth=1.0)
    line_ledgends_A.append(line_average)

    plt.legend(handles=line_ledgends_A,loc=1)
    plt.grid(b=True, which='both')
    plt.xlabel('local clock')
    plt.ylabel('prediction error(mm)')

    plt.subplot(3, 1, 3)
    line_ledgends_A=[]
    
    line_average, = plt.plot(range(len(plotter.lc.x_v)),np.abs(plotter.lc.x_v),'.', label='v', linewidth=1.0)
    line_ledgends_A.append(line_average)

    plt.legend(handles=line_ledgends_A,loc=1)
    plt.grid(b=True, which='both')
    #plt.ylim([-7e-6,-5e-6])
    plt.xlabel('local clock')
    plt.ylabel('relative drift speed')
    
    plt.show()
    exit(0)
    
if __name__ == "__main__":
    main()

            
