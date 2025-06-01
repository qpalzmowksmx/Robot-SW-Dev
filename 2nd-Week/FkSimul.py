import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 행렬값
def dh_matrix(theta, d, a, alpha):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

# Forward Kinematics 시뮬레이션
def forward_kinematics(dh_params):
    T = np.identity(4)
    positions = [T[:3, 3]]  # origin
    
    for theta, d, a, alpha in dh_params:
        T = T @ dh_matrix(theta, d, a, alpha)
        positions.append(T[:3, 3])  # save position of each joint
    
    return np.array(positions)

# 로봇팔의 위치를 3D로 시각화
def plot_robot(positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]
    ax.plot(xs, ys, zs, '-o', linewidth=2, markersize=6)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Forward Kinematics - Robot Arm')

    ax.set_box_aspect([1, 1, 1])
    ax.grid(True)
    plt.show()

# 메인 함수
def main():
    # 예시: 간단한 3자유도 로봇팔 DH 파라미터 (radian)
    dh_params = [
        (np.radians(45), 0.5, 1.0, np.radians(0)),
        (np.radians(30), 0.0, 1.0, np.radians(0)),
        (np.radians(-30), 0.0, 0.5, np.radians(0))
    ]

    positions = forward_kinematics(dh_params)
    print("각 관절 위치 (X, Y, Z):")
    for i, pos in enumerate(positions):
        print(f"Joint {i}: {pos}")

    plot_robot(positions)

if __name__ == "__main__":
    main()
