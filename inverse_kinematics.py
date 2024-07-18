import numpy as np
import sympy as sym
import forward_kinematics as forw_kin
import velocity_kinematics as vel_kin


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


def netwon_raphson_rf_method(Tmatrix, DHtable, mt, me, ig):
    epoh = 0
    e = me
    while epoh < mt and e >= me:
        DHtable[:,3] = ig[0,0:6]
        T_i = forw_kin.generate_combinedTmatrix_DHtable(DHtable)
        T_delta = Tmatrix - T_i
        delta = np.zeros((6,1))
        delta[0:3,0] = T_delta[0:3,3]
        delta_alpha = np.rad2deg(np.arctan2(T_delta[2,1],T_delta[2,2]))
        delta_beta = np.rad2deg(np.arctan2(-T_delta[2,0],np.sqrt((T_delta[2,1]**2 +T_delta[2,2]**2))))
        delta_gama = np.rad2deg(np.arctan2(T_delta[1,0],T_delta[0,0]))
        delta[3:6,0:1] = np.array([[delta_alpha],[delta_beta],[delta_gama],])
        jacobian_inv = np.linalg.pinv(vel_kin.jacobian(DHtable))
        delta_theta = np.dot(jacobian_inv,delta)
        print(delta)
        ig = ig + delta_theta
        epoh = epoh + 1
        e = np.linalg.norm(delta[0:3])
    return ig
