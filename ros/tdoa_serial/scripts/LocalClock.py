#!/usr/bin/env python
import json
import rospy
import time
import numpy as np

class LocalClock:
    'Class for tracking the master clock'

    def __init__(self, did):
        self.id = did
        self.tof_offset = 0
        # Kalman filter
        self.P = np.zeros((2,2))
        self.x = np.zeros((2,1)) # Master offset and offset change speed
        # Measurements
        self.m = []
        self.l = []
        self.have_init = False
        self.missed_update = 0
        self.x_pred_err=[]
        self.x_v = []
        
    def getErrorEstimate():
        return self.x_pred_err
    
    def addDataPt(self, master_clock, local_clock):
        self.m.append(master_clock)
        self.l.append(local_clock)
        while len(self.m) > 100: self.m.pop(0)
        while len(self.l) > 100: self.l.pop(0)
        
    def masterat(self, local_ts):
        if self.id == 0: return local_ts
        # Predict master ts for this local_ts
        if self.have_init == False: return 0;
        pred_x = self.predict(local_ts)
        return int(pred_x[0,0]) - self.tof_offset

    def predict(self, local_ts):
        if len(self.l) < 1: return -1
        dt = local_ts - self.l[-1]
        if (dt<0): dt+=0xFFFFFFFFFF;
        F = np.eye(2)
        F[0,1] = dt
        pred_x = np.mat(F) * np.mat(self.x);
        return pred_x
    
    def update(self):
        # Define fixed matrixes
        F = np.eye(2)
        H = np.zeros((1,2))
        H[0,0]=1
        H[0,1]=0
        
        R=(200/4.7)
        R=R*R
        Q = np.eye(2)
        Q[0,0] = R
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

        if (lts1<lts0): lts1+=0xFFFFFFFFFF;
            
        dt = lts1-lts0
        #rospy.loginfo("t@ %f dt: %f (mdt: %f)",lts0,dt,mts1-mts0);
        #rospy.loginfo("lts0: %f lts1: %f",lts0,lts1);
        #rospy.loginfo("mts0: %f mts1: %f",mts0,mts1);
        F[0,1] = dt
        if self.have_init == False or self.missed_update>5:
            self.have_init = True
            self.x[0,0] = mts1
            self.x[1,0] = (mts1-mts0)/dt
            self.P = np.zeros((2,2))
            self.P[0,0] = R
            self.P[1,1] = 1e-4
            rospy.loginfo("Initial x=[%f,%e]",self.x[0,0], self.x[1,0]-1.0);
            self.missed_update=0
            return

        # PREDICT x, P
        pred_x = np.mat(F) * np.mat(self.x);
        pred_P = np.mat(F) * np.mat(self.P) * np.mat(F.transpose()) + Q;
        #rospy.loginfo("new prediction xpred=[%f,%e]",pred_x[0,0], pred_x[1,0]-1.0);
        #rospy.loginfo("ppred=[%e,%e;%e,%e]",pred_P[0,0], pred_P[0,1], pred_P[1,1], pred_P[1,1]);
        
        # Translate measurement / local clock to match current time domain ???
        pred_z = np.mat(H)*np.mat(pred_x);
        need_to_correct_offset = False

        if (mts1<mts0):
            mts1+=0xFFFFFFFFFF;
            need_to_correct_offset = True
            
        z=np.zeros((1,1))
        z[0,0]=mts1
        y = z - pred_z;
        self.x_pred_err.append(y[0,0]/65535.0/1000000*3e8*1000)
        S = np.mat(H)*np.mat(pred_P)* np.mat(H.transpose()) + np.ones((1,1))*R;
        
        Sinv = 1.0/S[0,0];
        esquare = np.mat(y.transpose())*np.mat(Sinv)*np.mat(y);

        # Gate, difficult to tune...
        K = np.mat(pred_P)* np.mat(H.transpose())*np.mat(Sinv)
        #print esquare
        if esquare < 1e-5:
            self.missed_update=0
            self.x = pred_x + K*y;
            self.P = np.mat(np.eye(2) - K*H)*np.mat(pred_P);
        else:
            self.missed_update+=1
            self.x = pred_x
            self.P = pred_P

        if need_to_correct_offset:
            self.x[0,0]-=0xFFFFFFFFFF
            
        self.x_v.append(self.x[1,0]-1.0)
        #rospy.loginfo("new estimate x=[%f,%e] err=%e",self.x[0,0], self.x[1,0]-1.0, y[0,0]);
