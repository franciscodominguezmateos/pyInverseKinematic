'''
Created on 16 Jun 2018

@author: francisco
from: https://www.youtube.com/watch?v=wybp1_htA7k
Simulate a Stewart/Grugh robot platform
'''
import numpy as np
from visual import *

# End effector position
p=np.matrix([0,4,0]).T
# End effector orientation
R=np.matrix([[1,0,0],[0,1,0],[0,0,1]])

# robot definition
# this example is a 2D two prismatic articulation robot
# units are m
class Arm():
    def __init__(self):
        self.a1=np.matrix([-1,0,0]).T
        self.a2=np.matrix([ 1,0,0]).T
        self.b1=np.matrix([-0.5,0,0]).T
        self.b2=np.matrix([ 0.5,0,0]).T       
        self.p1=arrow(pos=self.a1)
        self.p2=arrow(pos=self.a2) 
    def parallelIK(self,p,R):
        a1=self.a1
        a2=self.a2
        b1=self.b1
        b2=self.b2
        s1=p+R*b1-a1
        s2=p+R*b2-a2
        return s1,s2

if __name__ == '__main__':
    scene.up=((0,1,0))
    a=Arm()
    t=0
    box(height=0.1,length=10,depth=10)
    b=box(pos=p,height=0.1)
    while true:
        rate(10)
        p=np.matrix([sin(t)*2,4,cos(t)*2*0]).T
        b.pos=p
        a.p1.axis=a.parallelIK(p,R)[0]
        a.p2.axis=a.parallelIK(p,R)[1]        
        t+=0.1
    print a.parallelIK(p,R)[0]
    print a.parallelIK(p,R)[1]
    