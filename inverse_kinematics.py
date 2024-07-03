import numpy as np
import sympy as sym
import forward_kinematics as forw_kin


def kinematic_decoupling(Tmatrix, DHtable):
    R, O, P, S = forw_kin.extract_Tmatrix(Tmatrix)
    R_decoupled = np.identity(4)
    R_decoupled[:3,:3] = R[:3,:3]
    O_decoupled = np.identity(4)
    O_decoupled[:3,3] = O[:3,0]
    d = DHtable[-1,2]
    wrist_position = O - d * R[2,:3].reshape((3,1))

    R_0_3 = forw_kin.generate_combinedTmatrix_DHtable(DHtable[:3])

    # inverse position problem
    a3 = DHtable[2,1]
    a2 = DHtable[1,1]
    d1 = DHtable[0,2]
    # theta_1_1 = np.arctan2(wrist_position[1],wrist_position[0])
    # theta_1_2 = np.arctan2(wrist_position[1],wrist_position[0]) + np.pi
    # r_sqr = wrist_position[0]**2 + wrist_position[1]**2 # OK
    # s_sqr = (wrist_position[2]-d1)**2 # OK
    # D = (s_sqr + r_sqr - a2**2 - a3**2)/(2 * a2 * a3)
    # theta_3_1 = np.arctan2(np.sqrt(1-D**2),D)
    # theta_3_2 = np.arctan2(-np.sqrt(1-D**2),D)
    # theta_2_1 = np.arctan2(np.sqrt(s_sqr),np.sqrt(r_sqr)) - np.arctan2(a3 * np.sin(theta_3_1),a2 + a3 * np.cos(theta_3_1))
    # theta_2_2 = np.arctan2(np.sqrt(s_sqr),np.sqrt(r_sqr)) - np.arctan2(a3 * np.sin(theta_3_2),a2 + a3 * np.cos(theta_3_2))

    # position_joint_sol = np.array([[theta_1_1,theta_2_1,theta_3_1],[theta_1_2,theta_2_2,theta_3_2]])

    # inverse orientation problem
    # s1_1 = np.sin(theta_1_1)
    # c1_1 = np.cos(theta_1_1)
    # s23_1 = np.sin(theta_2_1+theta_3_1)
    # c23_1 = np.cos(theta_2_1+theta_3_1)

    # s1_2 = np.sin(theta_1_2)
    # c1_2 = np.cos(theta_1_2)
    # s23_2 = np.sin(theta_2_2+theta_3_2)
    # c23_2 = np.cos(theta_2_1+theta_3_2)

    # l1 = s1_1 * R[0,2] - c1_1 * R[1,2]
    # l2 = s1_2 * R[0,2] - c1_2 * R[1,2]
    # theta_5_1 = np.arctan2(np.sqrt(1-(l1)**2),l1)
    # theta_5_2 = np.arctan2(-np.sqrt(1-(l1)**2),l1)
    # theta_5_3 = np.arctan2(np.sqrt(1-(l2)**2),l2)
    # theta_5_3 = np.arctan2(-np.sqrt(1-(l2)**2),l2)

    # theta_4_1 = np.arctan2(-c1_1*s23_1*R[0,2] - s1_1*s23_1*R[1,2])

    return R_0_3


def netwon_raphson_rf_method(Tmatrix, DHtable: sym.Matrix, mt, me, ig):
    joint_space = DHtable.free_symbols
    epoh = 0
    e = me
    while epoh < mt and e >= me:
        DHtable.subs(ig)
        T_i = forw_kin.generate_combinedTmatrix_DHtable(np.array(DHtable).astype(np.float64))
    return joint_space
