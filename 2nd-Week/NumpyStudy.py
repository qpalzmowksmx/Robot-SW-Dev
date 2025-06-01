from dh_transform import dh_matrix
import numpy as np

# 두 관절의 DH 파라미터 (radian 단위)
joint_params = [
    (np.radians(45), 0.5, 1.0, 0),   # theta, d, a, alpha
    (np.radians(30), 0.0, 1.0, 0)
]

# 초기 행렬 (단위행렬)
T = np.identity(4)

# 연속 곱 수행
for theta, d, a, alpha in joint_params:
    T = T @ dh_matrix(theta, d, a, alpha)

print("End Effector의 위치 및 자세:")
print(np.round(T, 4))
