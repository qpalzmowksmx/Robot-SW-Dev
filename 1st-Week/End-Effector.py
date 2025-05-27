# R=i=1∑n​L 에다가 겹치는 부분의 길이 뺴기
# 작업 가능 구체 계산하기-작업 구체 부피=3/4 * ​π * R^3

# 결과 예시
# python3 End-Effector.py 
# *** 로봇 자유도 계산기 + 작업 범위 분석기 ***
# 2 또는 3 인지 차원을 입력 : 3
# 링크-link 수를 입력 : 7
# 관절-joint 수를 입력 : 6
# 각 관절의 자유도를 차례로 입력하세요 : 1 1 1 1 1 1

# 계산된 자유도(DOF)는 6입니다.

# 각 링크의 길이를 입력하세요 (단위: m):
# 1 1 1 1 1 1 1 

#  최대 도달 거리 (팔을 최대한 펼쳤을 때): 7.00 m
#  구형 작업 공간 부피 (이론상 최대): 1436.76 m³

import math

def calculate_dof(num_links, num_joints, joint_dofs, dimension=3):
    m = 3 if dimension == 2 else 6
    F = m * (num_links - num_joints - 1) + sum(joint_dofs)
    return F

def calculate_max_reach(link_lengths):
    """모든 링크가 일직선으로 연결되었을 때의 최대 도달 거리"""
    return sum(link_lengths)

def calculate_workspace_volume(max_reach):
    """구형 작업 범위 부피 계산"""
    return (4/3) * math.pi * (max_reach ** 3)

def main():
    print("*** 로봇 자유도 계산기 + 작업 범위 분석기 ***")
    dim = int(input("2 또는 3 인지 차원을 입력 : "))
    num_links = int(input("링크-link 수를 입력 : "))
    num_joints = int(input("관절-joint 수를 입력 : "))

    input_str = input("각 관절의 자유도를 차례로 입력하세요 : ")
    if ' ' in input_str:
        joint_dofs = list(map(int, input_str.split()))
    else:
        joint_dofs = list(map(int, input_str.strip()))

    if len(joint_dofs) != num_joints:
        print("!!! ALERT 입력한 관절 수와 자유도 수가 불일치 !!!")
        return

    # 자유도 계산
    dof = calculate_dof(num_links, num_joints, joint_dofs, dim)
    print(f"\n👉 계산된 자유도(DOF)는 {dof}입니다.")

    # 링크 길이 입력
    print("\n📏 각 링크의 길이를 입력하세요 (단위: m):")
    link_lengths = list(map(float, input().split()))
    if len(link_lengths) != num_links:
        print("!!! ALERT 링크 수와 입력 길이 수가 일치하지 않습니다 !!!")
        return

    # 최대 도달 거리 및 구형 작업 범위 계산
    max_reach = calculate_max_reach(link_lengths)
    workspace_volume = calculate_workspace_volume(max_reach)

    print(f"\n 최대 도달 거리 (팔을 최대한 펼쳤을 때): {max_reach:.2f} m")
    print(f" 구형 작업 공간 부피 (이론상 최대): {workspace_volume:.2f} m³")

if __name__ == "__main__":
    main()

