import numpy as np

# create Transformation matrix
def create_Tmatrix(r_matrix, translation, prespective=np.array([0,0,0]), scale=1):
    Tmatrix = np.zeros((4,4))
    Tmatrix[:3,:3] = r_matrix
    Tmatrix[:3,3] = translation[:,0]
    Tmatrix[3,:3] = prespective
    Tmatrix[3,3] = scale
    return Tmatrix

# extract information from Transformation matrix
def extract_Tmatrix(Tmatrix):
    r_matrix = Tmatrix[:3,:3]
    translation = Tmatrix[:3,3].reshape(3,1)
    perspective = Tmatrix[3,:3]
    scale = Tmatrix[3,3]
    return (r_matrix, translation, perspective, scale)

# calculate inverse of Transformation matrix
def T_inverse(Tmatrix):
    r_matrix, translation, perspective, scale = extract_Tmatrix(Tmatrix)
    r_inverse = np.transpose(r_matrix)
    trans_inverse = np.dot(r_inverse,-1*translation).reshape(3,1)
    T_inverse = create_Tmatrix(r_inverse,trans_inverse,perspective,scale)
    return T_inverse

b = np.array([[2],[1],[0],[1]])
a = np.array([[5],[7.8],[7],[1]])

# DH parameter

def generate_Tmatrix_from_DHparameter(alpha,a, d,theta):
    general_transformation_matrix = np.array([
        [np.cos(np.deg2rad(theta)),-np.sin(np.deg2rad(theta)),0,a],
        [np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)),np.cos(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)),-np.sin(np.deg2rad(alpha)),-np.sin(np.deg2rad(alpha))*d],
        [np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)),np.cos(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)),np.cos(np.deg2rad(alpha)),np.cos(np.deg2rad(alpha))*d],
        [0,0,0,1]
    ])
    return general_transformation_matrix

def generate_combinedTmatrix_DHtable(DHtable):
    combined_Tmatrix = 0
    for i in range(1,DHtable.shape[0]+1):
        Tmatrix_i = generate_Tmatrix_from_DHparameter(DHtable[-i,0],DHtable[-i,1],DHtable[-i,2],DHtable[-i,3])
        if isinstance(combined_Tmatrix,int):
            combined_Tmatrix=Tmatrix_i
        else:
            combined_Tmatrix = np.dot(Tmatrix_i,combined_Tmatrix)

    return combined_Tmatrix

# DHtable : alpha, a , d, theta

def generate_joint_positions(DHtable):
    positions = np.zeros((DHtable.shape[0],4))
    positions[0,:] = np.array([0,0,0,1])
    for i in range(0,DHtable.shape[0]-1):
        Tmatrix = generate_combinedTmatrix_DHtable(DHtable[:i+1])
        b_vector = np.array([[DHtable[i+1,1]],[0],[0],[1]])
        position = np.dot(Tmatrix,b_vector)
        positions[i+1,:4] = position.reshape(1,4)
    return positions