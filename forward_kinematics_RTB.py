import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

#Create Model

RR_arm = rtb.DHRobot([
    rtb.RevoluteDH(0,1,0,0,qlim=[-np.pi/2,np.pi/2]),
    rtb.RevoluteDH(0,1,0,0,qlim=[-np.pi/2,np.pi/2])
], name="2R")

RR_arm.plot(q=[90/180*np.pi,90/180*np.pi],block=True)

