from scservo_sdk import *
import time

PORT = "COM6"
BAUD = 1000000

SERVO_ID = 1

MAX_POS = 1023
MAX_DEG = 220.0

# 마지막 목표값 저장 변수
saved_deg = 0
saved_pos = 0

portHandler = PortHandler(PORT)
packetHandler = scscl(portHandler)

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

# 온도 읽기
def read_temp(servo_id):

    temp, result, error = packetHandler.read1ByteTxRx(
        servo_id,
        63
    )

    if result == COMM_SUCCESS:
        return temp

    return None

# 현재 Position 읽기
def read_position(servo_id):

    pos, result, error = packetHandler.ReadPos(servo_id)

    if result == COMM_SUCCESS:
        return pos

    return None

# 모터 이동
def move_servo_deg(servo_id, deg, speed=500):

    global saved_deg
    global saved_pos

    # 입력 각도 -> Position 변환
    target_pos = deg_to_pos(deg)

    # 저장
    saved_deg = deg
    saved_pos = target_pos

    result, error = packetHandler.WritePos(
        servo_id,
        target_pos,
        0,
        speed
    )

    time.sleep(0.1)

    temp = read_temp(servo_id)
    current_pos = read_position(servo_id)

    if current_pos is not None:
        current_deg = pos_to_deg(current_pos)
    else:
        current_deg = 0

    print("\n===== 저장된 목표값 =====")
    print(f"입력각도 저장값 : {saved_deg:.1f}°")
    print(f"변환 Position 저장값 : {saved_pos}")

    print("\n===== 현재 상태 =====")
    print(f"ID : {servo_id}")
    print(f"현재 Position : {current_pos}")
    print(f"현재 각도 : {current_deg:.1f}°")
    print(f"현재 온도 : {temp}°C")
    print(f"result : {result}")
    print(f"error : {error}")

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("Baudrate 실패")
    quit()

print("구형 FT-SCS15 ID 4 제어 시작")

while True:

    value = input("\n각도 입력 0~220, 종료 q : ")

    if value.lower() == "q":
        break

    try:

        deg = float(value)

        move_servo_deg(
            SERVO_ID,
            deg,
            speed=500
        )

    except ValueError:
        print("숫자만 입력하세요.")

portHandler.closePort()

print("프로그램 종료")
