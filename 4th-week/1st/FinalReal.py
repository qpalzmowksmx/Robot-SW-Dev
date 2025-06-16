# 칼만 필터 연산에 부동소숫점 있다
# 잊지 말자,,,,
# float은 인식이 안된다 ㅠㅠㅠ
# FinalReal.py

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- 1. Load Data from CSV --- 귀찮으니까 git에 그대로 있는 상대경로
data = pd.read_csv('./IMU_GPS_sensor_data.csv')

time = data['time'].values
gps_x = data['gps_x'].values
gps_y = data['gps_y'].values
absolute_x = data['absolute_x'].values
absolute_y = data['absolute_y'].values
imu_ax = data['imu_acceleration_x'].values
imu_ay = data['imu_acceleration_y'].values

# --- 2. Kalman Filter Class ---
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.x = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64) # [pos_x, pos_y, vel_x, vel_y]
        self.P = np.eye(4) * 1000
        self.F = np.array([[1, 0, 1, 0],
                           [0, 1, 0, 1],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.Q = np.eye(4) * process_variance
        self.R = np.eye(2) * measurement_variance

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

# --- 3. Apply Kalman Filter to GPS Data ---
kf = KalmanFilter(process_variance=0.01, measurement_variance=5)
filtered_x, filtered_y = [], []

for gx, gy in zip(gps_x, gps_y):
    kf.predict()
    kf.update(np.array([gx, gy]))
    filtered_x.append(kf.x[0])
    filtered_y.append(kf.x[1])

# --- 4. Plot Kalman Filter Results ---
plt.figure(figsize=(12, 6))
plt.plot(gps_x, gps_y, 'b.', label='GPS 측정값')
plt.plot(filtered_x, filtered_y, 'r-', label='칼만 필터 추정값')
plt.xlabel('X 위치 (m)')
plt.ylabel('Y 위치 (m)')
plt.title('칼만 필터를 이용한 위치 추정')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- 5. Forward Kinematics & Jacobian Analysis for 2-Link Manipulator ---
def forward_kinematics(theta1, theta2, l1, l2):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

def jacobian(theta1, theta2, l1, l2):
    J = np.array([
        [-l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2), -l2 * np.sin(theta1 + theta2)],
        [l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2),  l2 * np.cos(theta1 + theta2)]
    ])
    return J

def singularity_check(J):
    return abs(np.linalg.det(J)) < 1e-6

def singularity_approach(J):
    return np.linalg.cond(J) > 1e3

def dls_pseudo_inverse(J, lambda_damping=0.1):
    Jt = J.T
    JJt = J @ Jt
    I = np.eye(JJt.shape[0])
    damped_inv = np.linalg.inv(JJt + (lambda_damping ** 2) * I)
    return Jt @ damped_inv

# --- 6. Evaluate for Range of Joint Angles ---
l1, l2 = 1.0, 1.0
theta1_range = np.linspace(-np.pi, np.pi, 100)
theta2_range = np.linspace(-np.pi, np.pi, 100)

X, Y = [], []
singular_pts = []
near_singular_pts = []

for t1 in theta1_range:
    for t2 in theta2_range:
        x, y = forward_kinematics(t1, t2, l1, l2)
        J = jacobian(t1, t2, l1, l2)

        if singularity_check(J):
            singular_pts.append((x, y))
        elif singularity_approach(J):
            near_singular_pts.append((x, y))
        else:
            X.append((x, y))

        J_pinv = dls_pseudo_inverse(J)  # just to confirm it works

# --- 7. Plot Singularity Analysis ---
plt.figure(figsize=(8, 8))
if X:
    x_all, y_all = zip(*X)
    plt.scatter(x_all, y_all, c='lightgray', s=5, label='Normal Configuration')

if near_singular_pts:
    x_near, y_near = zip(*near_singular_pts)
    plt.scatter(x_near, y_near, c='orange', s=20, label='Near Singularity')

if singular_pts:
    x_sing, y_sing = zip(*singular_pts)
    plt.scatter(x_sing, y_sing, c='red', s=40, marker='x', label='Singularity')

plt.title("2-Link Manipulator: Singularity Map & DLS-Aware Jacobian")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.show()
