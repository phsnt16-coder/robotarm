from scservo_sdk import *
import time
import random

# ================================
# 통신 설정
# ================================
PORT = "COM6"
BAUD = 1000000

# ================================
# Position / 각도 범위
# ================================
MAX_POS = 1023
MAX_DEG = 220.0

# ================================
# 모터 ID 정의
# ================================
ID_1 = 1
ID_2 = 2
ID_3 = 3
ID_4 = 4
ID_5 = 5
ID_6 = 6
ID_7 = 7

# ================================
# 구형 FT-SCS15 속도 설정
# ================================
OLD_SPEED = 500
OLD_TIME = 2000

# ================================
# 신형 FT-SCS15 속도 설정
# ================================
NEW_SPEED = 181
NEW_ACC = 1

# ================================
# ID6 초기 중앙각
# ================================
INIT_DEG_6 = 110.0

# ================================
# 보간 설정
# step_count:
# 보간 단계 수
#
# delay:
# 각 단계 사이 지연시간
# ================================
SMOOTH_STEP_COUNT = 30
SMOOTH_DELAY = 0.03

# ================================
# 포트 객체 생성
# ================================
portHandler = PortHandler(PORT)

# ================================
# 구형 FT-SCS15 제어 객체
# ================================
oldServo = scscl(portHandler)

# ================================
# 신형 FT-SCS15 제어 객체
# ================================
newServo = sms_sts(portHandler)

# ================================
# 마지막 각도 저장 변수
# 보간 시작점 계산에 사용
# ================================
last_deg_1 = None
last_deg_23 = None
last_deg_45 = None
last_deg_6 = None
last_deg_7 = None


# ================================
# 값 범위 제한 함수
# ================================
def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))


# ================================
# 각도 -> Position 변환
# ================================
def deg_to_pos(deg):

    deg = clamp(deg, 0, MAX_DEG)

    return int(deg * MAX_POS / MAX_DEG)


# ================================
# 역방향 Position 변환
# 동기화 모터용
# ================================
def reverse_pos(pos):

    pos = clamp(pos, 0, MAX_POS)

    return MAX_POS - pos


# ================================
# ID1 단일 제어
# ================================
def move_single_1(deg):

    global last_deg_1

    pos = deg_to_pos(deg)

    last_deg_1 = deg

    oldServo.WritePos(
        ID_1,
        pos,
        0,
        OLD_SPEED
    )


# ================================
# ID6 단일 제어
# ================================
def move_single_6(deg):

    global last_deg_6

    pos = deg_to_pos(deg)

    last_deg_6 = deg

    oldServo.WritePos(
        ID_6,
        pos,
        0,
        OLD_SPEED
    )


# ================================
# ID7 단일 제어
# ================================
def move_single_7(deg):

    global last_deg_7

    pos = deg_to_pos(deg)

    last_deg_7 = deg

    oldServo.WritePos(
        ID_7,
        pos,
        0,
        OLD_SPEED
    )


# ================================
# ID2,3 동기화
#
# ID2:
# 정방향
#
# ID3:
# 역방향
# ================================
def move_sync_2_3(deg):

    global last_deg_23

    logical_pos = deg_to_pos(deg)

    pos2 = logical_pos
    pos3 = reverse_pos(logical_pos)

    last_deg_23 = deg

    oldServo.WritePos(
        ID_2,
        pos2,
        0,
        OLD_SPEED
    )

    oldServo.WritePos(
        ID_3,
        pos3,
        0,
        OLD_SPEED
    )


# ================================
# ID4,5 동기화
#
# ID4:
# 구형 역방향
#
# ID5:
# 신형 정방향
# ================================
def move_sync_4_5(deg):

    global last_deg_45

    logical_pos = deg_to_pos(deg)

    pos5 = logical_pos
    pos4 = reverse_pos(logical_pos)

    last_deg_45 = deg

    # 신형 FT-SCS15
    newServo.WritePosEx(
        ID_5,
        pos5,
        NEW_SPEED,
        NEW_ACC
    )

    # 구형 FT-SCS15
    oldServo.WritePos(
        ID_4,
        pos4,
        OLD_TIME,
        OLD_SPEED
    )


# ================================
# 전체 축 이동 함수
# ================================
def move_all_pose(
    deg1,
    deg23,
    deg45,
    deg6,
    deg7
):

    move_single_1(deg1)

    move_sync_2_3(deg23)

    move_sync_4_5(deg45)

    move_single_6(deg6)

    move_single_7(deg7)


# ================================
# 보간 이동 함수
#
# 시작 자세와 목표 자세 사이를
# 여러 단계로 나눠 부드럽게 이동
#
# 현재 프로젝트에서 PID 대신 사용
# ================================
def smooth_move_all_pose(
    target_deg1,
    target_deg23,
    target_deg45,
    target_deg6,
    target_deg7
):

    global last_deg_1
    global last_deg_23
    global last_deg_45
    global last_deg_6
    global last_deg_7

    # 시작 각도 계산
    start_deg1 = last_deg_1 if last_deg_1 is not None else target_deg1
    start_deg23 = last_deg_23 if last_deg_23 is not None else target_deg23
    start_deg45 = last_deg_45 if last_deg_45 is not None else target_deg45
    start_deg6 = last_deg_6 if last_deg_6 is not None else target_deg6
    start_deg7 = last_deg_7 if last_deg_7 is not None else target_deg7

    # 보간 반복
    for i in range(1, SMOOTH_STEP_COUNT + 1):

        ratio = i / SMOOTH_STEP_COUNT

        # 중간 각도 계산
        deg1 = start_deg1 + (target_deg1 - start_deg1) * ratio
        deg23 = start_deg23 + (target_deg23 - start_deg23) * ratio
        deg45 = start_deg45 + (target_deg45 - start_deg45) * ratio
        deg6 = start_deg6 + (target_deg6 - start_deg6) * ratio
        deg7 = start_deg7 + (target_deg7 - start_deg7) * ratio

        # 실제 이동
        move_all_pose(
            deg1,
            deg23,
            deg45,
            deg6,
            deg7
        )

        time.sleep(SMOOTH_DELAY)


# ================================
# POSE 0 : SAFETY CHECK
#
# 목적:
# 전체 축 정상 동작 확인
# ================================
def pose_0_safety_check():

    print("\n===== POSE 0 : SAFETY CHECK =====")

    # 전체 0도
    smooth_move_all_pose(
        0,
        0,
        0,
        0,
        0
    )

    time.sleep(0.5)

    # 전체 10도
    smooth_move_all_pose(
        10,
        10,
        10,
        10,
        10
    )


# ================================
# POSE 1 : BASE
#
# 목적:
# 기본 대기 자세
# ================================
def pose_1_base():

    print("\n===== POSE 1 : BASE =====")

    smooth_move_all_pose(
        0,
        0,
        0,
        220,
        0
    )


# ================================
# POSE 2 : PICK READY
#
# 목적:
# Pick 전 대기 자세
# ================================
def pose_2_pick_ready():

    print("\n===== POSE 2 : PICK READY =====")

    smooth_move_all_pose(
        0,
        0,
        0,
        110,
        0
    )


# ================================
# POSE 3 : GO TO PLACE TEST
#
# 목적:
# Place 이동 테스트
#
# 현재:
# 랜덤값 사용
#
# 이후:
# IK 값 사용 예정
# ================================
def pose_3_go_to_place(
    min_deg=0,
    max_deg=90
):

    print("\n===== POSE 3 : GO TO PLACE TEST =====")

    deg1 = random.uniform(min_deg, max_deg)
    deg23 = random.uniform(min_deg, max_deg)
    deg45 = random.uniform(min_deg, max_deg)
    deg6 = random.uniform(min_deg, max_deg)
    deg7 = random.uniform(min_deg, max_deg)

    smooth_move_all_pose(
        deg1,
        deg23,
        deg45,
        deg6,
        deg7
    )


# ================================
# ACT_PICK
#
# 목적:
# 실제 Pick 동작
#
# 현재:
# 랜덤 테스트값 사용
#
# 이후:
# IK 결과값 사용 예정
#
# 최종 구조:
#
# 카메라 좌표
# ↓
# IK 계산
# ↓
# 관절각 생성
# ↓
# ACT_PICK
# ↓
# 보간 이동
# ↓
# 모터 구동
# ================================
def act_pick(
    min_deg=0,
    max_deg=90
):

    print("\n===== ACT_PICK : IK TEST MODE =====")

    # 현재는 랜덤 테스트값
    # 이후 실제 IK 결과값으로 교체 예정
    ik_deg1 = random.uniform(min_deg, max_deg)
    ik_deg23 = random.uniform(min_deg, max_deg)
    ik_deg45 = random.uniform(min_deg, max_deg)
    ik_deg6 = random.uniform(min_deg, max_deg)
    ik_deg7 = random.uniform(min_deg, max_deg)

    smooth_move_all_pose(
        ik_deg1,
        ik_deg23,
        ik_deg45,
        ik_deg6,
        ik_deg7
    )


# ================================
# ROTATION MODE
#
# 목적:
# 베이스 회전 테스트
#
# 특징:
# ID1만 45도 회전
# 나머지 축은 현재 상태 유지
# ================================
def rotation_mode():

    print("\n===== ROTATION MODE =====")

    target_deg1 = 45

    target_deg23 = last_deg_23 if last_deg_23 is not None else 0
    target_deg45 = last_deg_45 if last_deg_45 is not None else 0
    target_deg6 = last_deg_6 if last_deg_6 is not None else INIT_DEG_6
    target_deg7 = last_deg_7 if last_deg_7 is not None else 0

    smooth_move_all_pose(
        target_deg1,
        target_deg23,
        target_deg45,
        target_deg6,
        target_deg7
    )


# ================================
# 포트 열기
# ================================
if not portHandler.openPort():

    print("포트 열기 실패")

    quit()


# ================================
# Baudrate 설정
# ================================
if not portHandler.setBaudRate(BAUD):

    print("Baudrate 실패")

    quit()


print("FT-SCS15 보간 포즈 제어 시작")


# ================================
# 메인 루프
# ================================
while True:

    print("\n===== 포즈 선택 =====")

    print("0 : safety_check")
    print("1 : base")
    print("2 : pick_ready")
    print("3 : go_to_place 테스트")
    print("4 : ACT_PICK")
    print("5 : ROTATION")
    print("q : 종료")

    menu = input("선택 : ")

    # 종료
    if menu.lower() == "q":
        break

    # Safety Check
    if menu == "0":

        pose_0_safety_check()

    # Base
    elif menu == "1":

        pose_1_base()

    # Pick Ready
    elif menu == "2":

        pose_2_pick_ready()

    # Go To Place
    elif menu == "3":

        pose_3_go_to_place(
            0,
            90
        )

    # ACT_PICK
    elif menu == "4":

        act_pick(
            0,
            90
        )

    # Rotation
    elif menu == "5":

        rotation_mode()

    else:

        print("잘못된 입력")


# ================================
# 포트 종료
# ================================
portHandler.closePort()

print("프로그램 종료")