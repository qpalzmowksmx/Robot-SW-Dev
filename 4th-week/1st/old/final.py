import numpy as np
import matplotlib.pyplot as plt

# --- Forward Kinematics ---
def forward_kinematics(theta1, theta2, l1, l2):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

# --- Jacobian ---
def jacobian(theta1, theta2, l1, l2):
    J = np.array([
        [-l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2), -l2 * np.sin(theta1 + theta2)],
        [l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2),  l2 * np.cos(theta1 + theta2)]
    ])
    return J

# --- 특이점 판별 (determinant) ---
def singularity_check(J):
    det_J = np.linalg.det(J)
    return abs(det_J) < 1e-6

# --- 특이점 근접 여부 (condition number) ---
def singularity_approach(J):
    cond_J = np.linalg.cond(J)
    return cond_J > 1e3

# --- Damped Least Squares 역자코비안 계산 ---
def dls_pseudo_inverse(J, lambda_damping=0.1):
    Jt = J.T
    JJt = np.dot(J, Jt)
    I = np.eye(JJt.shape[0])
    damped_inv = np.linalg.inv(JJt + (lambda_damping ** 2) * I)
    J_plus = np.dot(Jt, damped_inv)
    return J_plus

# --- 설정 ---
l1 = 1.0  # Link 1 길이 (m)
l2 = 1.0  # Link 2 길이 (m)

theta1_range = np.linspace(-np.pi, np.pi, 100)
theta2_range = np.linspace(-np.pi, np.pi, 100)

# 결과 저장
X, Y = [], []
singular_pts = []
near_singular_pts = []

# --- 모든 θ 조합에 대해 분석 ---
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

        # 역자코비안 예시 계산 (특이점 근처에서도 안정적으로 나옴)
        J_pinv = dls_pseudo_inverse(J)

# --- 시각화 ---
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
