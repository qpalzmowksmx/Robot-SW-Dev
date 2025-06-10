import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R 
# 여기에 한번만 임포트하면 됩니다.

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

# --- Use scipy's Rotation for Euler angles ---
def rotation_matrix_to_euler_angles(R_mat):
    r = R.from_matrix(R_mat)
    return r.as_euler('zyx', degrees=True)  # or any desired sequence


# --- Obstacle Checker ---
def is_in_obstacle(point, obstacle_center, obstacle_radius):
    return np.linalg.norm(point - obstacle_center) < obstacle_radius

# --- Jacobian Calculation ---
def jacobian(theta_list, a, d, alpha):
    num_joints = len(theta_list)
    J = np.zeros((6, num_joints))
    T_0i = [np.eye(4)] * (num_joints + 1)

    # Compute transform matrices T_i for each joint
    for i in range(num_joints):
        T_i = dh_transform(theta_list[i], d[i], a[i], alpha[i])
        T_0i[i+1] = T_0i[i] @ T_i

    # Compute Jacobian matrix
    z_prev = np.array([0, 0, 1])  # Initial z-axis direction (world frame)
    p_n = T_0i[-1][:3, 3]
    for i in range(num_joints):
        z_i = T_0i[i][:3, 2]
        p_i = T_0i[i][:3, 3]
        J[:3, i] = np.cross(z_i, (p_n - p_i))
        J[3:, i] = z_i

    return J

# --- Plotting ---
def plot_robot(ax, positions, T, obstacle_center, obstacle_radius):
    ax.cla()
    R = T[:3, :3]
    orientation = rotation_matrix_to_euler_angles(R)

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
        ax.legend()

    # Draw obstacle
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = obstacle_radius * np.cos(u) * np.sin(v) + obstacle_center[0]
    y = obstacle_radius * np.sin(u) * np.sin(v) + obstacle_center[1]
    z = obstacle_radius * np.cos(v) + obstacle_center[2]
    ax.plot_surface(x, y, z, color='r', alpha=0.3)

    # Collision check
    for pos in positions:
        if is_in_obstacle(pos, obstacle_center, obstacle_radius):
            ax.set_title("⚠️ COLLISION DETECTED!", color='red')
            break
    else:
        ax.set_title("3D Visualization of Robotic Arm")

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1.5])
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
    
    # Calculate Jacobian and check singularity
    J = jacobian(theta, a, d, alpha)
    try:
        if np.linalg.cond(J) > 1e3:
            ax.set_title("⚠️ SINGULARITY DETECTED!", color='red')
    except np.linalg.LinAlgError:
        ax.set_title("⚠️ SINGULARITY DETECTED!", color='red')
    else:
        plot_robot(ax, positions, T, obstacle_center, obstacle_radius)

# --- Input DH Parameters (Length of Robot Links) ---
def input_robot_parameters():
    a = []
    d = []
    alpha = []
    
    print("Enter the DH parameters for each joint (in meters and radians):\n")
    
    for i in range(6):
        print(f"\nFor joint {i+1}:")
        a_i = float(input(f"Enter a{i+1} (Link length in meters): "))
        d_i = float(input(f"Enter d{i+1} (Link offset in meters): "))
        alpha_i = float(input(f"Enter alpha{i+1} (Twist angle in radians): "))
        
        a.append(a_i)
        d.append(d_i)
        alpha.append(alpha_i)
    
    return np.array(a), np.array(d), np.array(alpha)

# --- Parameters from User Input ---
a, d, alpha = input_robot_parameters()

# Obstacle setting (center position and radius)
obstacle_center = np.array([0.3, 0, 0.8])
obstacle_radius = 0.1

# --- Visualization Setup ---
theta_degrees = np.array([0, 0, 0, 0, 0, 0])
theta = np.radians(theta_degrees)
T, positions = forward_kinematics(theta, a, d, alpha)
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(211, projection='3d')
plot_robot(ax, positions, T, obstacle_center, obstacle_radius)

sliders = []
slider_ax = [plt.axes([0.3, 0.2 + i * 0.05, 0.4, 0.03]) for i in range(6)]
for i in range(6):
    slider = Slider(slider_ax[i], f'Theta {i+1}', -180, 180, valinit=theta_degrees[i])
    sliders.append(slider)
    slider.on_changed(update)

plt.tight_layout()
plt.show()
