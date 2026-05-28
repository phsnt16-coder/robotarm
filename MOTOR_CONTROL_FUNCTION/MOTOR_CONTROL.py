from scservo_sdk import *
import time

PORT = "COM6"
BAUD = 1000000

MAX_POS = 1023
MAX_DEG = 220.0

ID_1 = 1
ID_2 = 2
ID_3 = 3
ID_4 = 4
ID_5 = 5

OLD_SPEED = 500
OLD_TIME = 2000

NEW_SPEED = 181
NEW_ACC = 1

# 5번 신형은 항상 0도를 초기값으로 사용
INIT_DEG_5 = 0.0

portHandler = PortHandler(PORT)
oldServo = scscl(portHandler)
newServo = sms_sts(portHandler)

last_deg_1 = None
last_deg_23 = None
last_deg_45 = None

init_pos = {}

def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))

def deg_to_pos(deg):
    deg = clamp(deg, 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)

def pos_to_deg(pos):
    pos = clamp(pos, 0, MAX_POS)
    return pos * MAX_DEG / MAX_POS

def reverse_pos(pos):
    pos = clamp(pos, 0, MAX_POS)
    return MAX_POS - pos

def read_old_position(servo_id):
    pos, result, error = oldServo.ReadPos(servo_id)
    if result == COMM_SUCCESS:
        return pos
    return None

def read_old_temp(servo_id):
    temp, result, error = oldServo.read1ByteTxRx(servo_id, 63)
    if result == COMM_SUCCESS:
        return temp
    return None

def save_initial_positions():
    for sid in [ID_1, ID_2, ID_3, ID_4]:
        init_pos[sid] = read_old_position(sid)

    # ID5는 읽지 않고 항상 0도 기준
    init_pos[ID_5] = deg_to_pos(INIT_DEG_5)

    print("\n===== 초기 위치 저장 =====")
    for sid, pos in init_pos.items():
        if sid == ID_5:
            print(f"ID {sid} 초기 Position : {pos} / 고정 0도")
        else:
            print(f"ID {sid} 초기 Position : {pos}")

def move_single_1(deg):
    global last_deg_1

    pos = deg_to_pos(deg)
    last_deg_1 = deg

    result, error = oldServo.WritePos(ID_1, pos, 0, OLD_SPEED)

    temp = read_old_temp(ID_1)
    current_pos = read_old_position(ID_1)
    current_deg = pos_to_deg(current_pos) if current_pos is not None else None

    print(f"\n[ID1 단일]")
    print(f"입력각도:{deg:.1f}° | 목표Pos:{pos}")
    print(f"현재Pos:{current_pos} | 현재각도:{current_deg:.1f}°" if current_deg is not None else "현재각도: 읽기실패")
    print(f"온도:{temp}°C | result:{result} | error:{error}")

def move_sync_2_3(deg):
    global last_deg_23

    logical_pos = deg_to_pos(deg)
    pos2 = logical_pos
    pos3 = reverse_pos(logical_pos)

    last_deg_23 = deg

    result2, error2 = oldServo.WritePos(ID_2, pos2, 0, OLD_SPEED)
    result3, error3 = oldServo.WritePos(ID_3, pos3, 0, OLD_SPEED)

    print(f"\n[ID2,3 동기화]")
    print(f"입력각도:{deg:.1f}°")
    print(f"ID2 목표Pos:{pos2} | result:{result2} | error:{error2}")
    print(f"ID3 목표Pos:{pos3} | result:{result3} | error:{error3}")

def move_sync_4_5(deg):
    global last_deg_45

    logical_pos = deg_to_pos(deg)

    pos5 = logical_pos
    pos4 = reverse_pos(logical_pos)

    last_deg_45 = deg

    result5, error5 = newServo.WritePosEx(ID_5, pos5, NEW_SPEED, NEW_ACC)
    result4, error4 = oldServo.WritePos(ID_4, pos4, OLD_TIME, OLD_SPEED)

    print(f"\n[ID4,5 동기화]")
    print(f"입력각도:{deg:.1f}°")
    print(f"ID5 신형 목표Pos:{pos5} | result:{result5} | error:{error5}")
    print(f"ID4 구형 목표Pos:{pos4} | result:{result4} | error:{error4}")

def return_to_initial():
    print("\n===== 초기값으로 복귀 =====")

    if init_pos.get(ID_1) is not None:
        oldServo.WritePos(ID_1, init_pos[ID_1], 0, OLD_SPEED)
        print(f"ID1 -> {init_pos[ID_1]}")

    if init_pos.get(ID_2) is not None:
        oldServo.WritePos(ID_2, init_pos[ID_2], 0, OLD_SPEED)
        print(f"ID2 -> {init_pos[ID_2]}")

    if init_pos.get(ID_3) is not None:
        oldServo.WritePos(ID_3, init_pos[ID_3], 0, OLD_SPEED)
        print(f"ID3 -> {init_pos[ID_3]}")

    if init_pos.get(ID_4) is not None:
        oldServo.WritePos(ID_4, init_pos[ID_4], OLD_TIME, OLD_SPEED)
        print(f"ID4 -> {init_pos[ID_4]}")

    # ID5는 항상 0도 복귀
    init_pos_5 = deg_to_pos(INIT_DEG_5)
    newServo.WritePosEx(ID_5, init_pos_5, NEW_SPEED, NEW_ACC)
    print(f"ID5 -> 0도 / Position {init_pos_5}")

def input_angle_or_command(label, last_value):
    value = input(f"{label} 각도 입력 / N 유지 / O 초기복귀 / q 종료 : ")

    if value.lower() == "q":
        return "q"

    if value.upper() == "O":
        return "O"

    if value.upper() == "N":
        return last_value

    try:
        return float(value)
    except ValueError:
        print("숫자, N, O, q 중 하나만 입력")
        return last_value

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("Baudrate 실패")
    quit()

print("FT-SCS15 순차 각도 제어 시작")
print("N : 기존 각도 유지")
print("O : 초기값 복귀")
print("q : 종료")

save_initial_positions()

while True:
    print("\n===== 순차 입력 시작 =====")

    deg1 = input_angle_or_command("ID1 단일", last_deg_1)
    if deg1 == "q":
        break
    if deg1 == "O":
        return_to_initial()
        continue
    if deg1 is not None:
        move_single_1(deg1)
    else:
        print("ID1 기존값 없음 → 이동 생략")

    deg23 = input_angle_or_command("ID2,3 동기화", last_deg_23)
    if deg23 == "q":
        break
    if deg23 == "O":
        return_to_initial()
        continue
    if deg23 is not None:
        move_sync_2_3(deg23)
    else:
        print("ID2,3 기존값 없음 → 이동 생략")

    deg45 = input_angle_or_command("ID4,5 동기화", last_deg_45)
    if deg45 == "q":
        break
    if deg45 == "O":
        return_to_initial()
        continue
    if deg45 is not None:
        move_sync_4_5(deg45)
    else:
        print("ID4,5 기존값 없음 → 이동 생략")

    print("\n===== 한 사이클 완료 =====")

portHandler.closePort()
print("프로그램 종료")
