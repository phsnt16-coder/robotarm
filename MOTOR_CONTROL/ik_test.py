import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from ikpy.chain import Chain

# 1. 로봇 체인 모델 로드
my_chain = Chain.from_urdf_file("mybot.urdf")

# 2. 로드된 실제 링크 개수 확인 및 활성 관절 마스크 자동 생성
# (첫 번째 Base 링크는 고정(False), 나머지는 움직임(True) 허용)
total_links_count = len(my_chain.links)
custom_active_mask = [False] + [True] * (total_links_count - 1)
my_chain.active_links_mask = custom_active_mask

print(f"[INFO] 로드된 총 링크 수: {total_links_count}")
print(f"[INFO] 활성 관절 마스크: {custom_active_mask}")

# 3. 목표 위치 및 지면 수직 방향 설정 (미터 단위)
target_position = [0-.011505199361460338, 0.38488118000396805, 0.12039693138842285]
target_orientation = [0, 0, -1]

# 4. 초기 관절 위치 배열 설정 (도(Degree) 단위로 생각한 후 라디안으로 자동 변환)
# ⚠️ 주의: 실제 URDF의 링크 개수(total_links_count)와 배열의 길이가 반드시 일치해야 합니다.
# 아래는 예시 리스트이므로, 만약 링크 개수가 6개가 아니라면 개수에 맞게 값을 늘리거나 줄여주세요.
initial_deg = [0] * total_links_count
if total_links_count >= 5:
    initial_deg[1] = 0.0   # Base yaw (0도)
    initial_deg[2] = 15.0  # Shoulder (수직 특이점을 피하기 위해 15도 살짝 굽힘)
    initial_deg[3] = 45.0  # Elbow (45도 굽힘)
    initial_deg[4] = 15.0  # Wrist (15도)

# ikpy 주입을 위해 라디안(rad) 단위 리스트로 최종 변환
initial_joint_positions = np.radians(initial_deg).tolist()

# 5. 역기구학(IK) 계산 (초기 힌트 각도 주입)
print("\n역기구학 계산 중...")
joints = my_chain.inverse_kinematics(
    target_position=target_position,
    target_orientation=target_orientation,
    orientation_mode="Z",
    initial_position=initial_joint_positions  # 👈 라디안으로 변환된 초기 배열 반영
)

# 6. 계산 결과 출력 및 순기구학(FK) 검증
print("\n[계산된 관절 각도 (라디안 및 도)]")
for i, link in enumerate(my_chain.links):
    print(f"Link {i} ({link.name}): {joints[i]:.4f} rad (약 {np.degrees(joints[i]):.1f}°)")

computed_position = my_chain.forward_kinematics(joints)[:3, 3]
print(f"\n목표 좌표: {target_position}")
print(f"계산된 좌표: {computed_position.tolist()}")

# 오차 정밀도 확인
distance_error = np.linalg.norm(np.array(target_position) - computed_position)
print(f"목표치와의 직선 오차: {distance_error * 1000:.3f} mm")

# 7. 3D 포즈 시각화
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
my_chain.plot(joints, ax, target=target_position)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(0, 0.5)

plt.show()