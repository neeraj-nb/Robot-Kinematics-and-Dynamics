import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation
import numpy as np
import velocity_kinematics as vel_kin

fig = plt.figure()
ax = plt.axes(projection='3d')

DHtable = np.array([
            [-90,0,0.67183,45],
            [0,0.4318,0.13970,45],
            [90,0.0203,0,45],
            [-90,0,0.4318,45],
            [90,0,0,45],
            [0,0,0.0565,45]
        ])
jointlimits = [[0,360],[0,360],[0,360],[0,360],[0,360],[0,360]]

position = vel_kin.singularity(DHtable,jointlimits,10)

linex = position[:,0]
liney = position[:,1]
linez = position[:,2]

ax.scatter(linex,liney,linez,color='b',s=0.1,alpha=0.3)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.view_init(elev=90,azim=-90)
plt.show()