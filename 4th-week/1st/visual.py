import matplotlib.pyplot as plt

plt.figure(figsize=(12, 8))

# 실제 위치 (absolute_x, absolute_y)
plt.plot(absolute_x, absolute_y, label='Actual Position', color='blue')

# Kalman Filter 적용된 추정 위치 (estimated_x, estimated_y)
plt.plot(estimated_x, estimated_y, label='Estimated Position', color='red')

# GPS 측정값
plt.scatter(gps_x, gps_y, label='GPS Measurement', s=10, c='green')

plt.title('Kalman Filter 기반 위치 추정 (GPS + IMU 융합)')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid(True)
plt.show()
