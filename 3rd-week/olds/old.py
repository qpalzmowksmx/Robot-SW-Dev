import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

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

def rotation_matrix_to_euler_angles(R):
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    return np.degrees(pitch), np.degrees(yaw), np.degrees(roll)

# --- Obstacle Checker ---
def is_in_obstacle(point, obstacle_center, obstacle_radius):
    return np.linalg.norm(point - obstacle_center) < obstacle_radius

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
    plot_robot(ax, positions, T, obstacle_center, obstacle_radius)

# --- Parameters ---
theta_degrees = np.array([0, 0, 0, 0, 0, 0])
a = np.array([0, -0.425, -0.392, 0, 0, 0])
d = np.array([0.089, 0, 0, 0.109, 0.095, 0.082])
alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])

# Obstacle setting (center position and radius)
obstacle_center = np.array([0.3, 0, 0.8])
obstacle_radius = 0.1

# --- Visualization Setup ---
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
