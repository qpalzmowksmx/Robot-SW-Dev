import numpy as np

def dh_matrix(theta, d, a, alpha):
    """
    단일 DH 파라미터 집합에 대한 변환 행렬 생성 함수
    theta, alpha는 라디안 단위로 받음
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def main():
    print("! DH 파라미터 기반 변환 행렬 계산기 !")

    try:
        n = int(input("몇 개의 관절(DH 파라미터 셋)을 입력하시겠습니까? : "))
        dh_params = []

        for i in range(n):
            print(f"\n{i+1}번 관절의 DH 파라미터를 입력하세요:")
            theta = float(input("  θ (deg): "))
            d = float(input("  d (m): "))
            a = float(input("  a (m): "))
            alpha = float(input("  α (deg): "))

            # degree → radian 변환
            theta_rad = np.deg2rad(theta)
            alpha_rad = np.deg2rad(alpha)

            T = dh_matrix(theta_rad, d, a, alpha_rad)
            dh_params.append(T)

            print("\n  → 변환 행렬:")
            print(np.round(T, 3))

        # 추후 FK 시뮬에 넘기기 위해 반환도 가능
        return dh_params

    except Exception as e:
        print(f"!!! A L E R T !!!입력 중 오류 발생!!!: {e}")

if __name__ == "__main__":
    main()
