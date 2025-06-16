import pandas as pd
import numpy as np

# Load data from CSV
data = pd.read_csv('./IMU_GPS_sensor_data.csv')

# Extract columns
time = data['time'].values
gps_x = data['gps_x'].values
gps_y = data['gps_y'].values
absolute_x = data['absolute_x'].values
absolute_y = data['absolute_y'].values
imu_acceleration_x = data['imu_acceleration_x'].values
imu_acceleration_y = data['imu_acceleration_y'].values
