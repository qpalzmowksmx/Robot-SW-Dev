# Kalman Filter 초기화
kf = KalmanFilter(process_variance=1e-3, measurement_variance=10)

estimated_x = []
estimated_y = []

for i in range(len(time)):
    kf.predict()
    if not (np.isnan(gps_x[i]) and np.isnan(gps_y[i])):
        kf.update(np.array([gps_x[i], gps_y[i]]))
    estimated_state = kf.get_state()
    estimated_x.append(estimated_state[0])
    estimated_y.append(estimated_state[1])

estimated_x = np.array(estimated_x)
estimated_y = np.array(estimated_y)

