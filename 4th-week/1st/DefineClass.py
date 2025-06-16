import numpy as np

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.x = np.array([0, 0, 0, 0])  # 현재 상태 (위치 + 속도)
        self.P = np.eye(4) * 1000         # 초기 오차 공분산
        self.F = np.array([[1, 0, 1, 0],
                           [0, 1, 0, 1],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # 상태 전이 행렬
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])  # 측정 행렬
        self.Q = np.eye(4) * process_variance  # 프로세스 노이즈
        self.R = np.eye(2) * measurement_variance  # 측정 노이즈
    
    def predict(self):
        """예측 단계"""
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
    
    def update(self, z):
        """업데이트 단계"""
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(np.dot(self.H, self.P), self.H.T)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x += np.dot(K, y)
        I = np.eye(4)
        self.P = np.dot(I - np.dot(K, self.H), self.P)

    def get_state(self):
        return self.x[:2]  # 위치 반환
