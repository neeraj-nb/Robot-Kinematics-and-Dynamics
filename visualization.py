import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation
import numpy as np
import forward_kinematics as fw_kin

DHtable = np.array([
            [0,1,0,45],
            [0,1,0,45]
        ])
fig = plt.figure()
ax = plt.axes(projection='3d')

position = fw_kin.generate_joint_positions(DHtable)
print(position)
linex = position[:,0]
liney = position[:,1]
linez = position[:,2]

ax.plot(linex,liney,linez,'red')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.view_init(elev=90,azim=-90)
plt.show()