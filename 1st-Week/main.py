# dof_calculator-use
# 1주차 과제
# 
# 로봇의 자유도(Degree of Freedom, DOF)를 계산하는 프로그램입니다.
# 자유도는 로봇의 움직임을 정의하는 중요한 요소로, Kutzbach Formula를 사용하여 계산합니다.
# Kutzbach Formula:
#     F = m * (n - j - 1) + Σd_i
# 여기서,
#     F: 자유도-클수록 다양한 절대좌표에 도달가능
#     m: 차원 (2D = 3, 3D = 6)
#     n: 링크의 수
#     j: 관절의 수
#     d_i: 각 관절의 자유도
#

def calculate_dof(num_links, num_joints, joint_dofs, dimension=3):
    """
    Kutzbach Formula를 사용하여 로봇의 자유도를 계산

    Args:
        num_links (int): 링크의 수
        num_joints (int): 관절의 수
        joint_dofs (list of int): 각 관절의 자유도 리스트
        dimension (int): 2차원(2D) = 3, 3차원(3D) = 6 (기본값 3D)

    Returns:
        int: 계산된 자유도 수치값
    """
    if dimension == 2:
        m = 3
    else:
        m = 6

    F = m * (num_links - num_joints - 1) + sum(joint_dofs)
    return F


def main():
    print("*** 로봇 자유도 계산기 ***")
    dim = int(input("2 또는 3 인지 차원을 입력 : "))
    num_links = int(input("링크-link 수를 입력 : "))
    num_joints = int(input("관절-joint 수를 입력 : "))

    print("각 관절의 자유도를 차례로 입력하세요 !공백구분! (예: * * * * * *):")
    # 숫자 파싱할까?
    # 일단 2자리 이상의 경우와 충돌날수도?
    # 공백문자 치기 귀찮은데...
    # 고민
    joint_dofs = list(map(int, input().split()))

    if len(joint_dofs) != num_joints:
        print("!!! ALERT 입력한 관절 수와 자유도 수가 불일치 !!!")
        return

    dof = calculate_dof(num_links, num_joints, joint_dofs, dim)
    print(f"\n👉 계산된 자유도(DOF)는 {dof}입니다.")


if __name__ == "__main__":
    main()
