import numpy as np
from scipy.spatial.transform import Rotation
import math

def signQ(x): return 1 if x > 0 else -1

def FastCross(x, y):
    return np.cross(x, y)

def Quat(R):
    r = Rotation.from_matrix(R)
    quat = r.as_quat()  
    eta = quat[3]       
    eps = quat[:3]      
    return eta, eps

def FK_Quaternions(theta):
    L0 = 220; L1 = 737; L2 = 40; L3 = 500; L4 = 0.9; L5 = 419.8; L6 = 160; L7 = 200
    DH = np.array([
        [  L0,       0,     L1,       theta[0] ],
        [ -L2,   np.pi/2,    0,       theta[1] ],
        [  L3,       0,    -L4,       theta[2] ],
        [   0,  -np.pi/2,  -L5,       theta[3] ],
        [   0,   np.pi/2,    0,       theta[4] ],
        [   0,   np.pi/2,    0,       theta[5] ],
        [ -L7,       0,     L6,       0        ]
    ])

    T, P, R = [], [], []
    for a, alpha, d, t in DH:
        ca, sa, cal, sal = np.cos(t), np.sin(t), np.cos(alpha), np.sin(alpha)
        A = np.array([
            [ca, -sa, 0, a],
            [sa * cal, ca * cal, -sal, -sal * d],
            [sa * sal, ca * sal, cal, cal * d],
            [0, 0, 0, 1]
        ])
        T.append(A)
    
    T_all, P_all, R_all = [T[0]], [T[0][:3, 3]], [T[0][:3, :3]]
    for i in range(1, len(T)):
        T_next = T_all[-1] @ T[i]
        T_all.append(T_next)
        P_all.append(T_next[:3, 3])
        R_all.append(T_next[:3, :3])
    return T_all, P_all, R_all

def jacobi(theta):
    T, P, _ = FK_Quaternions(theta)
    J = np.zeros((6, 6))
    for i in range(6):
        z = T[i][:3, 2]
        J[:3, i] = FastCross(z, P[6] - P[i])
        J[3:, i] = z
    return J

def quaternion_ik(xyzrpy, theta):
    # Giải IK đầu vào: xyzrpy (m), the0: góc khớp khởi tạo
    comenset = [np.pi/2, 0, np.pi/4, np.pi/2, np.pi/2, 0]
    XYZ = xyzrpy[:3]
    yaw, pitch, roll = [math.radians(rad) for rad in xyzrpy[3:]]
    the0 = [math.radians(the) - comenset[index] for index, the in enumerate(theta)]

    # RPY to Rotation Matrix
    Rz = np.array([
        [np.cos(roll), -np.sin(roll), 0],
        [np.sin(roll), np.cos(roll), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(yaw), -np.sin(yaw)],
        [0, np.sin(yaw), np.cos(yaw)]
    ])
    # R_desired = Rx @ Ry @ Rz
    r = Rotation.from_euler('zyz', [roll, pitch, yaw])
    R_desired = r.as_matrix()
    etat, epst = Quat(R_desired)

    the = np.array(the0).copy()
    theta_max = np.deg2rad([90, 180, 80, 86, 90, 359.99])
    theta_min = np.deg2rad([-90, 0, -45, -90, -90, 0])
    KP, KO = np.eye(3) * 40, np.eye(3) * 10
    Gain = np.block([[KP, np.zeros((3,3))], [np.zeros((3,3)), KO]])
    alp, lamda, max_iter = 0.03, 0.01, 3000
    thresh1, thresh2 = 1e-4, 1e-4
    neP, neO = [], []
    for i in range(max_iter):
        J = jacobi(the)
        _, P, R = FK_Quaternions(the)
        eP = XYZ - P[6]
        eta, eps = Quat(R[6])
        eO = eta * epst - etat * eps - FastCross(epst, eps)
        
        twist      = np.concatenate((eP, eO))
        JT, JJ     = J.T, J.T @ J
        A          = JJ + (lamda**2) * np.eye(6)
        b          = JT @ (Gain @ twist)
        delta_theta = np.linalg.solve(A, b)

        the       += alp * delta_theta
        the        = np.clip(the, theta_min, theta_max)

        neP_val    = np.linalg.norm(eP)
        neO_val    = np.linalg.norm(eO)
        neP.append(neP_val)
        neO.append(neO_val)
        if neP[-1] < thresh1 and neO[-1] < thresh2:
            break
    
    the_real = [(the[index] + comenset[index]) for index in range(6)]

    if i == max_iter - 1:
        return False, None, None, None, None, None
    else:
        return True, the_real, neP, neO, P[6], R[6]

# Test
if __name__ == "__main__":
    xyzrpy = np.array([800, 0.9, 1100, 0, 90, 0])
    the0 = np.array([90, 88.95, 43.9, 86.38, 90.02, 180.85])
    _, theta, pos_err, ori_err, final_pos, final_rot = quaternion_ik(xyzrpy, the0)
    theta1 = [np.degrees(the) for the in theta]
    print("Góc khớp tính được (rad):", theta1)
    print("Vị trí cuối:", final_pos)
