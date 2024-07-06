import numpy as np
import forward_kinematics as frw_kin
import itertools
import multiprocessing
from functools import partial

from tqdm import tqdm

def jacobian(DHtable):
    Jacobian = np.zeros((6,6))
    T_0_6 = frw_kin.generate_combinedTmatrix_DHtable(DHtable)
    p_end = T_0_6[:3,3]

    T_0_i = np.identity(4)
    for i in range(DHtable.shape[0]):
        if i==0:
            T_0_i = T_0_i
        else:
            T = frw_kin.generate_Tmatrix_from_DHparameter(DHtable[i-1,0],DHtable[i-1,1],DHtable[i-1,2],DHtable[i-1,3])
            T_0_i = np.dot(T_0_i,T)
        z_i = T_0_i[:3,2]
        p_i = T_0_i[:3,3]

        r = p_end - p_i
        Jacobian[0:3,i] = np.cross(z_i,r)
        Jacobian[3:6,i] = z_i
    
    return Jacobian

def _singularity_worker(DHtable,progress,point):
    DHtable[0:6,3] = np.array(list(point))
    j = jacobian(DHtable)
    if np.linalg.det(j)==0:
        orientation, position, perspective, scale = frw_kin.extract_Tmatrix(frw_kin.generate_combinedTmatrix_DHtable(DHtable))
    if progress:
        progress.update()
    return position
    
def singularity(DHtable, jointlimits,progress=False,parallel=False):
    n = 3
    joint_space = [np.linspace(joint[0],joint[1],n) for joint in jointlimits]
    singularity = []
    simulation_space = list(itertools.product(joint_space[0],joint_space[1],joint_space[2],joint_space[3],joint_space[4],joint_space[5]))
    if progress:
        compute = n**6
        pbar = tqdm(total=compute)
    if parallel:
        pool = multiprocessing.Pool(processes=4)
        if not progress:
            pbar = False
        target = partial(_singularity_worker,DHtable,pbar)
        singularity = pool.map(target,simulation_space)
    else:
        for point in simulation_space:
            DHtable[0:6,3] = np.array(list(point))
            j = jacobian(DHtable)
            if np.linalg.det(j)==0:
                orientation, position, perspective, scale = frw_kin.extract_Tmatrix(frw_kin.generate_combinedTmatrix_DHtable(DHtable))
                singularity.append(position)
            if progress:
                pbar.update()
    singularity = np.array(singularity)

    return singularity