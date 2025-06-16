import numpy as np

def ik_solver(x, y, z, roll, pitch, yaw):
    # Độ dài các đoạn tay (mm)
    l1, l2, l3, l4, l5, l6 = 385.9, 40, 500, 0.9, 417.37, 60

    # Giới hạn các khớp (đơn vị: rad)
    theta_max = np.deg2rad([90, 180, 80, 86, 90, 359.99])
    theta_min = np.deg2rad([-90, 0, -45, -90, -90, 0])

    # Tính ma trận quay từ roll, pitch, yaw
    r11 = np.cos(pitch) * np.cos(yaw)
    r12 = np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.cos(roll) * np.sin(yaw)
    r13 = np.sin(roll) * np.sin(yaw) + np.cos(roll) * np.cos(yaw) * np.sin(pitch)
    r21 = np.cos(pitch) * np.sin(yaw)
    r22 = np.cos(roll) * np.cos(yaw) + np.sin(pitch) * np.sin(roll) * np.sin(yaw)
    r23 = np.cos(roll) * np.sin(pitch) * np.sin(yaw) - np.cos(yaw) * np.sin(roll)
    r31 = -np.sin(pitch)
    r32 = np.cos(pitch) * np.sin(roll)
    r33 = np.cos(pitch) * np.cos(roll)

    Rw6 = np.array([
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ])

    # Vị trí cổ tay (wrist center)
    Px = x - l6 * r13
    Py = y - l6 * r23
    Pz = z - l6 * r33

    # Tính theta1
    Alpha = np.arctan2(Px, Py)
    c1_0 = l4 / np.sqrt(Px**2 + Py**2)
    c1_0 = np.clip(c1_0, -1, 1)
    try:
        theta1_0 = np.arctan2(np.sqrt(1 - c1_0**2), c1_0) - Alpha
        theta1_1 = np.arctan2(-np.sqrt(1 - c1_0**2), c1_0) - Alpha
    except ValueError:
        return []  # Không có nghiệm do sqrt(âm)

    b = Pz - l1
    raw_solutions = []

    for theta1 in [theta1_0, theta1_1]:
        a = Px * np.cos(theta1) + Py * np.sin(theta1) + l2
        s3 = (a**2 + b**2 - l3**2 - l5**2) / (2 * l3 * l5)
        s3 = np.clip(s3, -1, 1)

        try:
            for sign3 in [1, -1]:
                theta3 = np.arctan2(s3, sign3 * np.sqrt(1 - s3**2))
                c = l3 + l5 * s3
                beta = np.arctan2(b, a)
                c2 = c / np.sqrt(a**2 + b**2)
                c2 = np.clip(c2, -1, 1)

                for sign2 in [1, -1]:
                    theta2 = sign2 * np.arccos(c2) + beta

                    # Rw3 transpose
                    Rw3_T = np.array([
                        [np.cos(theta2 + theta3) * np.cos(theta1), np.cos(theta2 + theta3) * np.sin(theta1), np.sin(theta2 + theta3)],
                        [-np.sin(theta2 + theta3) * np.cos(theta1), -np.sin(theta2 + theta3) * np.sin(theta1), np.cos(theta2 + theta3)],
                        [np.sin(theta1), -np.cos(theta1), 0]
                    ])

                    R36 = Rw3_T @ Rw6

                    for sign5 in [1, -1]:
                        theta4 = np.arctan2(-R36[2, 2], R36[0, 2])
                        s5 = np.sqrt(1 - R36[1, 2]**2)
                        theta5 = np.arctan2(sign5 * s5, -R36[1, 2])
                        theta6 = np.arctan2(-R36[1, 1], R36[1, 0])

                        solution = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
                        raw_solutions.append(solution)
        except ValueError:
            continue  # Skip nếu math domain error

    # Lọc nghiệm hợp lệ
    valid_solutions = []
    for sol in raw_solutions:
        if np.all(sol >= theta_min) and np.all(sol <= theta_max):
            print(sol)
            if all(np.linalg.norm(sol - np.array(v)) > 1e-3 for v in valid_solutions):
                valid_solutions.append(sol)

    return np.degrees(valid_solutions)


# Example usage
if __name__ == "__main__":
    xyzrpy = [850.28, -278.0, 970.17, 0.0, np.pi / 2, np.pi]
    solutions = ik_solver(*xyzrpy)
    for i, sol in enumerate(solutions, 1):
        print(f"Solution {i}: {np.round(sol, 2)} degrees")
