import numpy as np
from scipy.spatial.transform import Rotation
import math

def signQ(x): return 1 if x > 0 else -1

def FastCross(x, y):
    return np.cross(x, y)

def Quat(R):
    def clamp(x):
        return max(x, 0)

    etat = 0.5 * np.sqrt(clamp(R[0, 0] + R[1, 1] + R[2, 2] + 1))

    epsilon = np.zeros(3)
    epsilon[0] = 0.5 * signQ(R[2, 1] - R[1, 2]) * np.sqrt(clamp(R[0, 0] - R[1, 1] - R[2, 2] + 1))
    epsilon[1] = 0.5 * signQ(R[0, 2] - R[2, 0]) * np.sqrt(clamp(R[1, 1] - R[2, 2] - R[0, 0] + 1))
    epsilon[2] = 0.5 * signQ(R[1, 0] - R[0, 1]) * np.sqrt(clamp(R[2, 2] - R[0, 0] - R[1, 1] + 1))

    return etat, epsilon

def FK_Quaternions(theta):
    L0 = 220; L1 = 737; L2 = 40; L3 = 500; L5 = 419.8; L6 = 160; L7 = 200
    DH = np.array([
        [  L0,       0,     L1,       theta[0] ],
        [ -L2,   np.pi/2,    0,       theta[1] ],
        [  L3,       0,      0,       theta[2] ],
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
    print(xyzrpy)
    print([math.degrees(the2) for the2 in the0])
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
    R0EE = Rotation.from_euler('zyz', [roll, pitch, yaw]).as_matrix()
    etat, epst = Quat(R0EE)

    the = np.array(the0).copy()
    theta_max = np.deg2rad([90, 180, 90, 90, 90, 359.99])
    theta_min = np.deg2rad([-90, 0, -45, -90, -90, 0])
    KP, KO = np.eye(3) * 30.5, np.eye(3) * 3.3
    Gain = np.block([[KP, np.zeros((3,3))], [np.zeros((3,3)), KO]])
    alp, lamda, max_iter = 0.02, 0.02, 3000
    thresh1, thresh2 = 1e-2, 1e-2
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
    the_deg = [math.degrees(the) for the in the_real]
    print(i, the_deg)

    if i == max_iter - 1:
        return False, None, None, None, None, None
    else:
        return True, the_real, neP, neO, P[6], R[6]

def IE(xyzrpy):
    X, Y, Z = xyzrpy[:3]
    roll, pitch, yaw = [math.radians(rad) for rad in xyzrpy[3:]]
    print(xyzrpy)

    L00 = 219
    L0 = 740.5
    L1 = 40
    L2 = 500
    L3 = 420
    L4 = 60
    L5 = 146
    L6 = 100

    # Rotation matrices
    Rz = np.array([[np.cos(roll), -np.sin(roll), 0],
                [np.sin(roll), np.cos(roll), 0],
                [0, 0, 1]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx = np.array([[1, 0, 0],
                [0, np.cos(yaw), -np.sin(yaw)],
                [0, np.sin(yaw), np.cos(yaw)]])

    R06 = Rz @ Ry @ Rx

    T07 = np.eye(4)
    T07[:3, :3] = R06
    T07[0, 3] = X - L00
    T07[1, 3] = Y
    T07[2, 3] = Z - L0

    P67 = np.array([[-L5], [0], [-L6], [1]])
    P06 = T07 @ P67

    X6, Y6, Z6 = P06[:3, 0]

    WX = X6 - L4 * R06[0, 2]
    WY = Y6 - L4 * R06[1, 2]
    WZ = Z6 - L4 * R06[2, 2]

    # Inverse Kinematics
    the1 = np.arctan2(WY, WX)
    T1 = np.degrees(the1)

    A = WX * np.cos(the1) + WY * np.sin(the1) + L1
    B = WZ
    s3 = (A**2 + B**2 - L3**2 - L2**2) / (2 * L3 * L2)
    c3_1 = np.sqrt(1 - s3**2)
    c3_2 = -np.sqrt(1 - s3**2)
    the3_1 = np.arctan2(np.round(s3, 5), np.round(c3_1, 5))
    the3_2 = np.arctan2(np.round(s3, 5), np.round(c3_2, 5))
    T3_1 = np.degrees(the3_1)
    T3_2 = np.degrees(the3_2)

    def compute_theta2(s3, c3, A, B, L2, L3):
        c2 = (A * (L3 * s3 + L2) - B * L3 * c3) / ((L3 * s3 + L2)**2 + (L3 * c3)**2)
        s2 = (B * (L3 * s3 + L2) + A * L3 * c3) / ((L3 * s3 + L2)**2 + (L3 * c3)**2)
        return np.arctan2(np.round(s2, 5), np.round(c2, 5))

    the2_1 = compute_theta2(s3, c3_1, A, B, L2, L3)
    the2_2 = compute_theta2(s3, c3_2, A, B, L2, L3)
    T2_1 = np.degrees(the2_1)
    T2_2 = np.degrees(the2_2)

    def R03(theta1, theta2, theta3):
        return np.array([
            [np.cos(theta2 + theta3) * np.cos(theta1), -np.sin(theta2 + theta3) * np.cos(theta1), np.sin(theta1)], 
            [np.cos(theta2 + theta3) * np.sin(theta1), -np.sin(theta2 + theta3) * np.sin(theta1), -np.cos(theta1)], 
            [np.sin(theta2 + theta3), np.cos(theta2 + theta3), 0]
        ])

    R03_1 = R03(the1, the2_1, the3_1)
    R03_2 = R03(the1, the2_2, the3_2)
    R36_1 = np.round(R03_1.T @ R06, 10)
    R36_2 = np.round(R03_2.T @ R06, 10)

    def compute_theta456(R36):
        theta5 = np.arctan2(np.sqrt(R36[1,0]**2 + R36[1,1]**2), -R36[1,2])
        print(theta5)
        if np.abs(np.sin(theta5)) > 1e-3:
            theta4 = np.arctan2(R36[2,2], -R36[0,2])
            theta6 = np.arctan2(R36[1,1], R36[1,0])
        else:
            theta4 = 0
            theta6 = np.arctan2(-R36[0,1], R36[0,0])
        return [theta4, theta5, theta6]

    # the4_1 = np.arctan2(R36_1[2, 2], -R36_1[0, 2])
    # the4_2 = np.arctan2(R36_2[2, 2], -R36_2[0, 2])

    # the5_1 = np.arctan2(np.sqrt(R36_1[1, 0]**2 + R36_1[1, 1]**2), -R36_1[1, 2])
    # the5_2 = np.arctan2(-np.sqrt(R36_1[1, 0]**2 + R36_1[1, 1]**2), -R36_1[1, 2])
    # the5_3 = np.arctan2(np.sqrt(R36_2[1, 0]**2 + R36_2[1, 1]**2), -R36_2[1, 2])
    # the5_4 = np.arctan2(-np.sqrt(R36_2[1, 0]**2 + R36_2[1, 1]**2), -R36_2[1, 2])

    # the6_1 = np.arctan2(R36_1[1, 1], -R36_1[1, 0])
    # the6_2 = np.arctan2(R36_2[1, 1], -R36_2[1, 0])


    [the4_1, the5_1, the6_1] = compute_theta456(R36_1)

    print(np.degrees(the4_1), np.degrees(the5_1), np.degrees(the6_1) )

    comenset = [90, 0, 45, 90, 90, 179.99]
    angles = np.array([T1, T2_1, T3_1, np.degrees(the4_1), np.degrees(the5_1), np.degrees(the6_1)])
    theta_max = [90, 180, 90, 86, 90, 180]
    theta_min = [-90, 0, -45, -90, -90, -180]
    print(angles)
    if np.isnan(angles).any():
        return False, None

    # Kiểm tra nằm trong khoảng
    if np.all((angles >= theta_min) & (angles <= theta_max)):
        result = angles + comenset
        result_rad = np.radians(result)
        return True, result_rad.tolist()
    else:
        return False, None

def ForwardKinematics(joint_deg, N=6):
    L0 = 220; L1 = 737; L2 = 40; L3 = 500; L4 = 0.9; L5 = 419.8; L6 = 160; L7 = 200
    joint = np.radians(joint_deg)
    DH = np.array([
        [  L0,       0,     L1,       joint[0] ],
        [ -L2,   np.pi/2,    0,       joint[1] ],
        [  L3,       0,      0,       joint[2] ],
        [   0,  -np.pi/2,  -L5,       joint[3] ],
        [   0,   np.pi/2,    0,       joint[4] ],
        [   0,   np.pi/2,    0,       joint[5] ],
        [ -L7,       0,     L6,       0        ]
    ])

    A = np.zeros((N+1, 4, 4))
    for i in range(N+1):
        a, alpha, d, theta = DH[i]
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        A[i] = np.array([
            [ct, -st, 0, a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa,  ca,  ca*d],
            [0, 0, 0, 1]
        ])

    T = A[0]
    for i in range(1, N+1):
        T = T @ A[i]
    position = T[:3, 3]         
    rotation = T[:3, :3]

    r = Rotation.from_matrix(rotation)
    rpy = r.as_euler('zyz', degrees=True)
    yaw, pitch, roll = rpy[0], rpy[1], rpy[2] 

    return np.array([*position, roll, pitch, yaw])


# Test
if __name__ == "__main__":
    xyzrpy = np.array([800.87, 100, 1000, 0, 90, -178])
    the0 = np.array([24.534000000000002, 87.71, 4.591000000000005, 84.703, -25.063999999999997, 85.808])
    _, theta, pos_err, ori_err, final_pos, final_rot = quaternion_ik(xyzrpy, the0)
    theta1 = [np.degrees(the) for the in theta]
    print("Góc khớp tính được (rad):", theta1)
    print("Vị trí cuối:", final_pos)
