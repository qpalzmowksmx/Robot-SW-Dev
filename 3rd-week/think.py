import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R

# --- Forward Kinematics Core Functions ---
def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                   np.cos(alpha),                  d],
        [0,              0,                                  0,                             1]
    ])

def forward_kinematics(theta_list, a, d, alpha):
    T = np.eye(4)
    positions = []
    for i in range(len(theta_list)):
        T_i = dh_transform(theta_list[i], d[i], a[i], alpha[i])
        T = T @ T_i
        positions.append(T[:3, 3])
    return T, positions

def rotation_matrix_to_euler_angles(R_mat):
    r = R.from_matrix(R_mat)
    return r.as_euler('zyx', degrees=True)

def is_in_obstacle(point, obstacle_center, obstacle_radius):
    return np.linalg.norm(point - obstacle_center) < obstacle_radius

def jacobian(theta_list, a, d, alpha):
    num_joints = len(theta_list)
    J = np.zeros((6, num_joints))
    T_0i = [np.eye(4)] * (num_joints + 1)
    for i in range(num_joints):
        T_i = dh_transform(theta_list[i], d[i], a[i], alpha[i])
        T_0i[i+1] = T_0i[i] @ T_i
    p_n = T_0i[-1][:3, 3]
    for i in range(num_joints):
        z_i = T_0i[i][:3, 2]
        p_i = T_0i[i][:3, 3]
        J[:3, i] = np.cross(z_i, (p_n - p_i))
        J[3:, i] = z_i
    return J

# --- Plotting ---
def plot_robot(ax, positions, T, obstacle_center, obstacle_radius, singularity_point):
    ax.cla()
    R_mat = T[:3, :3]
    orientation = rotation_matrix_to_euler_angles(R_mat)

    if positions:
        ax.plot([0, positions[0][0]], [0, positions[0][1]], [0, positions[0][2]], 'ro-')
        for i in range(len(positions) - 1):
            ax.plot([positions[i][0], positions[i + 1][0]], 
                    [positions[i][1], positions[i + 1][1]], 
                    [positions[i][2], positions[i + 1][2]], 'bo-')

        end_effector = positions[-1]
        ax.scatter(end_effector[0], end_effector[1], end_effector[2], color='green',
                   label=f'Pos: ({end_effector[0]:.2f}, {end_effector[1]:.2f}, {end_effector[2]:.2f})\n'
                         f'Ori: ({orientation[2]:.2f}, {orientation[0]:.2f}, {orientation[1]:.2f})')

        # End-effector z-axis
        z_axis_vector = R_mat[:, 2]
        arrow_length = 20  # cm
        arrow_tip = end_effector + arrow_length * z_axis_vector
        ax.plot([end_effector[0], arrow_tip[0]],
                [end_effector[1], arrow_tip[1]],
                [end_effector[2], arrow_tip[2]],
                color='red')

        ax.scatter(singularity_point[0], singularity_point[1], singularity_point[2],
                   color='purple', s=100, marker='x', label='Singularity Point')

        # # Singularity proximity check
        # if np.linalg.norm(end_effector - singularity_point) < 10:
        #     ax.set_title("⚠️ NEAR SINGULARITY!", color='orange')

        # draw singularity point
        ax.scatter(singularity_point[0], singularity_point[1], singularity_point[2],
           color='orange', marker='^', s=100, label='Singularity')

        # check for singularity proximity
        if np.linalg.norm(end_effector - singularity_point) < 10:  # 10cm 이내 경고
             ax.set_title("⚠️ NEAR SINGULARITY!", color='orange')



        ax.legend()

    # Obstacle
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = obstacle_radius * np.cos(u) * np.sin(v) + obstacle_center[0]
    y = obstacle_radius * np.sin(u) * np.sin(v) + obstacle_center[1]
    z = obstacle_radius * np.cos(v) + obstacle_center[2]
    ax.plot_surface(x, y, z, color='r', alpha=0.3)

    for pos in positions:
        if is_in_obstacle(pos, obstacle_center, obstacle_radius):
            ax.set_title("⚠️ COLLISION DETECTED!", color='red')
            break

    ax.set_xlim([-300, 300])
    ax.set_ylim([-300, 300])
    ax.set_zlim([0, 300])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.grid()
    plt.draw()

# --- Callback ---

def update(val):
    for i in range(6):
        theta_degrees[i] = sliders[i].val
    theta = np.radians(theta_degrees)

    T, positions = forward_kinematics(theta, a, d, alpha)
    J = jacobian(theta, a, d, alpha)

    try:
        cond = np.linalg.cond(J)
        if cond > 1000:
            ax.set_title("⚠️ NEAR SINGULARITY! Applying damping...", color='orange')

            # 예: 목표 end-effector 속도 벡터 (임의, 0.1씩)
            dx = np.array([0.1, 0.1, 0.1, 0, 0, 0])
            
            # DLS 적용
            joint_velocities = damped_least_squares(J, dx, damping=0.1)
            print("Adjusted joint velocities:", joint_velocities)

            # 여기선 실제로 조인트를 업데이트하진 않지만, 시뮬레이션에 활용 가능
        else:
            plot_robot(ax, positions, T, obstacle_center, obstacle_radius, singularity_point)
    except np.linalg.LinAlgError:
        ax.set_title("⚠️ SINGULARITY DETECTED!", color='red')

# def update(val):
#     for i in range(6):
#         theta_degrees[i] = sliders[i].val
#     theta = np.radians(theta_degrees)
#     T, positions = forward_kinematics(theta, a, d, alpha)
#     J = jacobian(theta, a, d, alpha)
#     try:
#         if np.linalg.cond(J) > 1e3:
#             near_singularity = True
#             # ax.set_title("!!! SINGULARITY DETECTED !!!", color='red')
#     except np.linalg.LinAlgError:
#         near_singularity = True
#         ax.set_title("⚠️ SINGULARITY DETECTED!", color='red')
#     else:
#         plot_robot(ax, positions, T, obstacle_center, obstacle_radius, singularity_point)

# --- Parameters ---

# --- Singularity Point 입력 ---
def input_singularity_point():
    print("Enter singularity point coordinates (in cm):")
    x = float(input("X: "))
    y = float(input("Y: "))
    z = float(input("Z (height): "))
    return np.array([x, y, z])


# 로봇팔에 대한거
def input_robot_parameters():
    a, d, alpha = [], [], []
    for i in range(6):
        print(f"\nFor joint {i+1}:")
        a_i = float(input(f"Enter a{i+1} (cm): "))
        d_i = float(input(f"Enter d{i+1} (cm): "))
        alpha_i = float(input(f"Enter alpha{i+1} (rad): "))
        a.append(a_i)
        d.append(d_i)
        alpha.append(np.radians(alpha_i))
    return np.array(a), np.array(d), np.array(alpha)

a, d, alpha = input_robot_parameters()
obstacle_center = np.array([0.3, 0, 0.8])
obstacle_radius = 0.1
singularity_point = input_singularity_point()   # 입력된 특이점 받아오기

# --- Visualization ---
theta_degrees = np.array([0, 0, 0, 0, 0, 0])
theta = np.radians(theta_degrees)
T, positions = forward_kinematics(theta, a, d, alpha)
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(211, projection='3d')
plot_robot(ax, positions, T, obstacle_center, obstacle_radius, singularity_point)

sliders = []
slider_ax = [plt.axes([0.3, 0.2 + i * 0.05, 0.4, 0.03]) for i in range(6)]
for i in range(6):
    slider = Slider(slider_ax[i], f'Theta {i+1}', -180, 180, valinit=theta_degrees[i])
    sliders.append(slider)
    slider.on_changed(update)

plt.tight_layout()
plt.show()
