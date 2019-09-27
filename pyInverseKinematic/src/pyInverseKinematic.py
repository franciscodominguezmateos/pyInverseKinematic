'''
Created on Feb 5, 2016
@author: Francisco Dominguez
'''
import numpy as np
import numpy.linalg as la
from math import *
import vpython as vs
from vpython import vector

class Arm():
    def __init__(self,l0=10,l1=10,l2=10,l3=10):
        self.l0=l0
        self.l1=l1
        self.l2=l2
        self.l3=l3
        self.q=np.matrix([[0],[0],[0]])
        self.v0=vs.arrow(pos=vector(0,0,0),axis=vector(l0,0,0))
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
        self.v1.pos=vector(r0_O1x,r0_O1y,0)
        self.v1.axis=vector(r0_12x,r0_12y,0)

        r0_O2x=r0_O1x+r0_12x
        r0_O2y=r0_O1y+r0_12y
        r0_23x=l2*cos(q1+q2)
        r0_23y=l2*sin(q1+q2)
        self.v2.pos=vector(r0_O2x,r0_O2y,0)
        self.v2.axis=vector(r0_23x,r0_23y,0)

        r0_O3x=r0_O2x+r0_23x
        r0_O3y=r0_O2y+r0_23y
        r0_3Fx=l3*cos(q1+q2+q3)
        r0_3Fy=l3*sin(q1+q2+q3)
        self.v3.pos=vector(r0_O3x,r0_O3y,0)
        self.v3.axis=vector(r0_3Fx,r0_3Fy,0)
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
        alpha=0.05
        qi1=qi+alpha*dq
        return qi1
    #inverse diffrentialkinematics
    def idkStep(self,qi,dr):
        J=self.J(qi)
        Jp=la.pinv(J)
        dq=Jp*dr
        qi1=qi+dq
        return qi1
    #inverse constrained differential kinematics
    def idkStopStepConstrained(self,qi,q1):
        J=self.J(qi)
        Jp=la.pinv(J)
        I=np.matrix([[1,0,0],
                     [0,1,0],
                     [0,0,1]])
        N=I-Jp*J
        dq=N*q1
        qi1=qi+dq
        return qi1
    def idkStepConstrained(self,qi,dr,q1):
        J=self.J(qi)
        Jp=la.pinv(J)
        I=np.matrix([[1,0,0],
                     [0,1,0],
                     [0,0,1]])
        N=I-Jp*J
        dq=Jp*dr+N*q1
        qi1=qi+dq
        return qi1
    
vs.scene.up=vector(1,0,0)
a=Arm()
PI=np.pi
q0=np.matrix([[PI*0.25],[PI*0.25],[0.5]])
rg=np.matrix([10,20,0]).T
a.setGoal(rg)
qi=q0
ri=a.r(qi)
print (ri,la.norm(rg-ri))
qc=np.matrix([[0],[0],[pi/2]])
t=0
while True:
    while la.norm(rg-ri)>0.15:
        vs.rate(10)
        qi=a.ikStep(qi)
        ri=a.r(qi)
        a.update(qi)
        print (ri,la.norm(rg-ri))
#     while True:
#         vs.rate(10)
#         drt=sin(t/2)/20.0
#         qct=sin(t*5)/10.0
#         dqc=np.matrix([[qct],[0],[0]])
#         dr=np.matrix([[drt],[0],[0]])
#         qi=a.idkStepConstrained(qi,dr,dqc)
#         a.update(qi)
#         t+=0.1
        #print st
    while True:
        vs.rate(10)
        st=sin(t*5)/2
        qc=np.matrix([[st],[0],[0]])
        qi=a.idkStopStepConstrained(qi,qc)
        a.update(qi)
        t+=0.1
        #print (st)
print (qi)


        
    


