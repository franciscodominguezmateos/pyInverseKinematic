'''
Created on Feb 5, 2016
@author: Francisco Dominguez
'''
import numpy as np
import numpy.linalg as la
from math import *
import visual as vs

class Arm():
    def __init__(self,l0=10,l1=10,l2=10,l3=10):
        self.l0=l0
        self.l1=l1
        self.l2=l2
        self.l3=l3
        self.q=np.matrix([[0],[0],[0]])
        self.v0=vs.arrow(por=(0,0,0),axis=(l0,0,0))
        self.v1=vs.arrow()
        self.v2=vs.arrow()
        self.v3=vs.arrow()
        self.update(self.q)
    def update(self,q):
        q1=q[0]
        q2=q[1]
        q3=q[2]
        l0=self.l0
        l1=self.l1
        l2=self.l2
        l3=self.l3
        r0_O1x=l0
        r0_O1y=0
        r0_12x=l1*cos(q1)
        r0_12y=l1*sin(q1)
        self.v1.pos=(r0_O1x,r0_O1y,0)
        self.v1.axis=(r0_12x,r0_12y,0)

        r0_O2x=r0_O1x+r0_12x
        r0_O2y=r0_O1y+r0_12y
        r0_23x=l2*cos(q1+q2)
        r0_23y=l2*sin(q1+q2)
        self.v2.pos=(r0_O2x,r0_O2y,0)
        self.v2.axis=(r0_23x,r0_23y,0)

        r0_O3x=r0_O2x+r0_23x
        r0_O3y=r0_O2y+r0_23y
        r0_3Fx=l3*cos(q1+q2+q3)
        r0_3Fy=l3*sin(q1+q2+q3)
        self.v3.pos=(r0_O3x,r0_O3y,0)
        self.v3.axis=(r0_3Fx,r0_3Fy,0)
    def r(self,q):
        q1=q[0]
        q2=q[1]
        q3=q[2]
        l0=self.l0
        l1=self.l1
        l2=self.l2
        l3=self.l3
        r0_O1x=l0
        r0_O1y=0
        r0_12x=l1*cos(q1)
        r0_12y=l1*sin(q1)
        r0_O2x=r0_O1x+r0_12x
        r0_O2y=r0_O1y+r0_12y
        r0_23x=l2*cos(q1+q2)
        r0_23y=l2*sin(q1+q2)
        r0_O3x=r0_O2x+r0_23x
        r0_O3y=r0_O2y+r0_23x
        r0_3Fx=l3*cos(q1+q2+q3)
        r0_3Fy=l3*sin(q1+q2+q3)
        rx=r0_O1x+r0_12x+r0_23x+r0_3Fx
        ry=r0_O1y+r0_12y+r0_23y+r0_3Fy
        rz=0
        return np.matrix([[rx],[ry],[rz]])
    def J(self,q):
        q1=q[0]
        q2=q[1]
        q3=q[2]
        l0=self.l0
        l1=self.l1
        l2=self.l2
        l3=self.l3
        drx_q1=-l1*sin(q1)-l2*sin(q1+q2)-l3*sin(q1+q2+q3)
        drx_q2=           -l2*sin(q1+q2)-l3*sin(q1+q2+q3)
        drx_q3=                         -l3*sin(q1+q2+q3)
        dry_q1= l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3)
        dry_q2=            l2*cos(q1+q2)+l3*cos(q1+q2+q3)
        dry_q3=                          l3*cos(q1+q2+q3)
        return np.matrix([[drx_q1,drx_q2,drx_q3],
                          [dry_q1,dry_q2,dry_q3],
                          [0,0,0]])
    def setGoal(self,r):
        self.rGoal=r
    #invere kinematics
    def ikStep(self,qi):
        rg=self.rGoal
        ri=self.r(qi)
        J=self.J(qi)
        Jp=la.pinv(J)
        dr=(rg-ri)
        dq=Jp*dr
        alpha=1
        qi1=qi+alpha*dq
        return qi1
    #inverse diffrentialkinematics
    def idkStep(self,qi,dr):
        J=self.J(qi)
        Jp=la.pinv(J)
        dq=Jp*dr
        qi1=qi+dq
        return qi1
    #inverse constrained kinematics
    def ikStepConstrained(self,qi,q1):
        J=self.J(qi)
        Jp=la.pinv(J)
        I=np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        N=I-Jp*J
        #dr=rg-ri
        #dq=Jp*dr+N*q0
        Jq1=np.matrix([1,0,0])
        dq=N*la.pinv(Jq1*N)*q1
        qi1=qi+dq
        return qi1
    
vs.scene.up=((1,0,0))
a=Arm()
q0=np.matrix([[0],[0],[0]])
rg=np.matrix([15,25,0]).T
a.setGoal(rg)
qi=q0
ri=a.r(qi)
print ri,la.norm(rg-ri)
qc=np.matrix([[0],[0],[pi/2]])
t=0
while True:
    while la.norm(rg-ri)>0.05:
        vs.rate(1)
        qi=a.ikStep(qi)
        ri=a.r(qi)
        a.update(qi)
        print ri,la.norm(rg-ri)
    while True:
        vs.rate(20)
        st=sin(t)
        qc=np.matrix([[st],[0],[0]])
        qi=a.ikStepConstrained(qi,st/20)
        a.update(qi)
        t+=0.1
        #print st
print qi


        
    


