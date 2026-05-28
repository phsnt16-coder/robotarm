from scservo_sdk import *
import time

PORT = "COM6"
BAUD = 1000000

# 모터 ID
OLD_ID = 4   # 구형, 역방향
NEW_ID = 5   # 신형, 정방향

MAX_POS = 1023
MAX_DEG = 220.0

# 속도값
OLD_SPEED = 500
OLD_TIME = 2000

NEW_SPEED = 181   # 네가 맞춘 값으로 수정
NEW_ACC = 1       # 네가 맞춘 값으로 수정

# 마지막 목표값 저장 변수
saved_deg = 0
saved_old_pos = 0
saved_new_pos = 0

portHandler = PortHandler(PORT)

# 구형 제어용
oldServo = scscl(portHandler)

# 신형 제어용
newServo = sms_sts(portHandler)

def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))

# 각도 -> Position 변환
def deg_to_pos(deg):
    deg = clamp(deg, 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)

# Position -> 각도 변환
def pos_to_deg(pos):
    pos = clamp(pos, 0, MAX_POS)
    return pos * MAX_DEG / MAX_POS

# 역방향 변환
def reverse_pos(pos):
    pos = clamp(pos, 0, MAX_POS)
    return MAX_POS - pos

# 구형 온도 읽기
def read_old_temp(servo_id):
    temp, result, error = oldServo.read1ByteTxRx(
        servo_id,
        63
    )

    if result == COMM_SUCCESS:
        return temp

    return None

# 구형 Position 읽기
def read_old_position(servo_id):
    pos, result, error = oldServo.ReadPos(servo_id)

    if result == COMM_SUCCESS:
        return pos

    return None

# 신형 Position 읽기
# 신형은 읽기값이 깨졌던 적이 있으므로 실패 가능성 있음
def read_new_position(servo_id):
    pos, result, error = newServo.ReadPos(servo_id)

    if result == COMM_SUCCESS and 0 <= pos <= MAX_POS:
        return pos

    return None

# 동기화 이동
def move_sync_deg(deg):
    global saved_deg
    global saved_old_pos
    global saved_new_pos

    # 입력 각도 -> 논리 Position
    logical_pos = deg_to_pos(deg)

    # 5번 신형 정방향
    new_target_pos = logical_pos

    # 4번 구형 역방향
    old_target_pos = reverse_pos(logical_pos)

    # 저장
    saved_deg = deg
    saved_new_pos = new_target_pos
    saved_old_pos = old_target_pos

    # 신형 명령
    result_new, error_new = newServo.WritePosEx(
        NEW_ID,
        new_target_pos,
        NEW_SPEED,
        NEW_ACC
    )

    # 구형 명령
    result_old, error_old = oldServo.WritePos(
        OLD_ID,
        old_target_pos,
        OLD_TIME,
        OLD_SPEED
    )

    time.sleep(0.1)

    old_temp = read_old_temp(OLD_ID)
    old_current_pos = read_old_position(OLD_ID)
    new_current_pos = read_new_position(NEW_ID)

    if old_current_pos is not None:
        old_logical_pos = reverse_pos(old_current_pos)
        old_current_deg = pos_to_deg(old_logical_pos)
    else:
        old_logical_pos = None
        old_current_deg = None

    if new_current_pos is not None:
        new_current_deg = pos_to_deg(new_current_pos)
    else:
        new_current_deg = None

    print("\n===== 저장된 목표값 =====")
    print(f"입력각도 저장값 : {saved_deg:.1f}°")
    print(f"신형 ID {NEW_ID} 변환 Position 저장값 : {saved_new_pos}")
    print(f"구형 ID {OLD_ID} 변환 Position 저장값 : {saved_old_pos}")

    print("\n===== 현재 상태 =====")
    print(f"[신형] ID : {NEW_ID}")
    print(f"현재 Position : {new_current_pos}")
    if new_current_deg is not None:
        print(f"현재 각도 : {new_current_deg:.1f}°")
    else:
        print("현재 각도 : 읽기실패")
    print(f"result : {result_new}")
    print(f"error : {error_new}")

    print(f"\n[구형] ID : {OLD_ID}")
    print(f"현재 Position : {old_current_pos}")
    print(f"현재 논리 Position : {old_logical_pos}")
    if old_current_deg is not None:
        print(f"현재 각도 : {old_current_deg:.1f}°")
    else:
        print("현재 각도 : 읽기실패")
    print(f"현재 온도 : {old_temp}°C")
    print(f"result : {result_old}")
    print(f"error : {error_old}")

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("Baudrate 실패")
    quit()

print("신형 ID 5 + 구형 ID 4 동기화 제어 시작")
print("신형 5번 : 정방향")
print("구형 4번 : 역방향")
print("각도 범위 : 0~220도")

while True:
    value = input("\n각도 입력 0~220, 종료 q : ")

    if value.lower() == "q":
        break

    try:
        deg = float(value)

        move_sync_deg(deg)

    except ValueError:
        print("숫자만 입력하세요.")

portHandler.closePort()

print("프로그램 종료")
