import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- 1. Load Data ---
data = pd.read_csv('./IMU_GPS_sensor_data.csv/')  # 파일 경로 에 있는지 확인 까먹지 않기--상대경로 커찮

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
        self.x = np.array([0, 0, 0, 0])  # [pos_x, pos_y, vel_x, vel_y]
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
        self.P = np.dot(self.F @ self.P @ self.F.T) + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

# --- 3. Apply Kalman Filter ---
kf = KalmanFilter(process_variance=0.01, measurement_variance=5)
filtered_x, filtered_y = [], []

for gx, gy in zip(gps_x, gps_y):
    kf.predict()
    kf.update(np.array([gx, gy]))
    filtered_x.append(kf.x[0])
    filtered_y.append(kf.x[1])

# --- 4. Plot Results ---
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
