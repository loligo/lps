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

class LocalClock:
    'Class for tracking the master clock'

    def __init__(self):
        self.tof_offset = 0
        # Kalman filter
        self.P = np.zeros((2,2))
        self.P[0,0] = 1e14
        self.P[1,1] = 1e-8
        self.x = np.zeros((2,1)) # Master offset and offset change speed
        # Measurements
        self.m = []
        self.l = []
        self.have_init = False
        self.x_pred_err=[]
        self.x_v = []
        
    def getErrorEstimate():
        return self.x_pred_err
    
    def addDataPt(self, master_clock, local_clock):
        self.m.append(master_clock)
        self.l.append(local_clock)

    def masterat(self, local_ts):
        # Predict master ts for this local_ts
        return 0
        
    def update(self):
        # Define fixed matrixes
        F = np.eye(2)
        H = np.zeros((1,2))
        H[0,0]=1
        H[0,1]=0
        
        R=1e12
        Q = np.eye(2)
        Q[0,0] = 1e3*1e3
        Q[1,1] = 2e-6*2e-6

        k=0
        lts0 = 0
        lts1 = 0
        mts0 = 0
        mts1 = 0
        for lts,mts in reversed(zip(self.l,self.m)):
            if k==0:
                lts1 = lts
                mts1 = mts
            if k==1:
                lts0 = lts
                mts0 = mts
                k+=1
                break
            k+=1

        if k<2: return

        if (lts1<lts0):
            rospy.loginfo("overflow lts1");
            lts1+=0xFFFFFFFFFF;
            
        dt = lts1-lts0
        rospy.loginfo("t@ %f dt: %f (mdt: %f)",lts0,dt,mts1-mts0);
        rospy.loginfo("lts0: %f lts1: %f",lts0,lts1);
        rospy.loginfo("mts0: %f mts1: %f",mts0,mts1);
        F[0,1] = dt
        if self.have_init == False:
            self.have_init = True
            self.x[0,0] = mts1
            self.x[1,0] = (mts1-mts0)/dt
            rospy.loginfo("Initial x=[%f,%e]",self.x[0,0], self.x[1,0]-1.0);
            return

        # PREDICT x, P
        pred_x = np.mat(F) * np.mat(self.x);
        pred_P = np.mat(F) * np.mat(self.P) * np.mat(F.transpose()) + Q;
        rospy.loginfo("new prediction xpred=[%f,%e]",pred_x[0,0], pred_x[1,0]-1.0);
        rospy.loginfo("ppred=[%e,%e;%e,%e]",pred_P[0,0], pred_P[0,1], pred_P[1,1], pred_P[1,1]);
        
        # Translate measurement / local clock to match current time domain ???
        pred_z = np.mat(H)*np.mat(pred_x);
        need_to_correct_offset = False

        if (mts1<mts0):
            rospy.loginfo("overflow mts1");
            mts1+=0xFFFFFFFFFF;
            need_to_correct_offset = True
            
        z=np.zeros((1,1))
        z[0,0]=mts1
        y = z - pred_z;
        self.x_pred_err.append(y[0,0]/65535.0/1000000*3e8*1000)
        S = np.mat(H)*np.mat(pred_P)* np.mat(H.transpose()) + np.ones((1,1))*R;
        
        Sinv = 1.0/S[0,0];
        esquare = np.mat(y.transpose())*np.mat(Sinv)*np.mat(y);

        # Gate?!
        K = np.mat(pred_P)* np.mat(H.transpose())*np.mat(Sinv)
        print esquare
        if esquare < 1e-6:
            self.x = pred_x + K*y;
            self.P = np.mat(np.eye(2) - K*H)*np.mat(pred_P);
        else:
            self.x = pred_x
            self.P = pred_P

        if need_to_correct_offset:
            self.x[0,0]-=0xFFFFFFFFFF

        self.x_v.append(self.x[1,0]-1.0)
        rospy.loginfo("new estimate x=[%f,%e] err=%e",self.x[0,0], self.x[1,0]-1.0, y[0,0]);
        

        
class Plotter:
    'Class instance for analysing uwb timings'

    def __init__(self):
        self.data = []
        rospy.Subscriber('uwbjson', String, self.uwbjson_callback)
        self.lc = LocalClock()

    def datalen(self,id=-1):
        return len(self.data)
        
    def uwbjson_callback(self,d):
        if len(self.data) > 505: return
        try:
            a = json.loads(d.data)
        except ValueError:
            return;
        self.data.append(a)

    def extractts_old(self,d,offset):
        ts=np.uint64(0)
        for i in range(0,5): ts = ts + (int(d[offset+i],16) << i*8)
        return ts

    def extractts(self,d,offset):
        ts=np.uint64(0)
        o = offset*2
        for i in range(0,5):
            start=o+i*2
            end=start+2
            ts = ts + (int(d[start:end],16) << i*8)
        #rospy.loginfo("ts: %X", ts)    
        return ts
        
    def getTsDataForId(self,id):
        remote_ts = []
        local_ts = [] 
        for d in self.data:
            if d['l_id']==id:
                try:
                    master_clock = self.extractts(d['data'].replace(' ',''),10)
                except KeyError:
                    continue;
                print(d)
                local_clock = int(d['dw_ts'],16)
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
    
    while not rospy.is_shutdown() and plotter.datalen()<100:
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

            
