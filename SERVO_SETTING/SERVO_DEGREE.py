from scservo_sdk import *
import time

PORT = "COM6"
BAUD = 1000000

SERVO_ID = 6

MAX_POS = 1023
MAX_DEG = 220.0

SPEED = 500

saved_deg = None
saved_pos = None

portHandler = PortHandler(PORT)
servo = scscl(portHandler)

def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))

def deg_to_pos(deg):
    deg = clamp(deg, 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)

def pos_to_deg(pos):
    pos = clamp(pos, 0, MAX_POS)
    return pos * MAX_DEG / MAX_POS

def read_position():
    pos, result, error = servo.ReadPos(SERVO_ID)

    if result == COMM_SUCCESS:
        return pos

    return None

def read_temp():
    temp, result, error = servo.read1ByteTxRx(
        SERVO_ID,
        63
    )

    if result == COMM_SUCCESS:
        return temp

    return None

# 현재 상태 출력
def print_current_status():

    current_pos = read_position()

    if current_pos is not None:
        current_deg = pos_to_deg(current_pos)
    else:
        current_deg = None

    temp = read_temp()

    print("\n===== 현재 상태 =====")

    print(f"ID : {SERVO_ID}")
    print(f"현재 Position : {current_pos}")

    if current_deg is not None:
        print(f"현재 각도 : {current_deg:.1f}°")
    else:
        print("현재 각도 : 읽기실패")

    print(f"현재 온도 : {temp}°C")

    if saved_deg is not None:
        print(f"저장 각도 : {saved_deg:.1f}°")
        print(f"저장 Position : {saved_pos}")
    else:
        print("저장 각도 : 없음")

# 각도 저장
def save_angle(deg):

    global saved_deg
    global saved_pos

    saved_deg = clamp(deg, 0, MAX_DEG)
    saved_pos = deg_to_pos(saved_deg)

    print("\n===== 각도 저장 완료 =====")
    print(f"저장 각도 : {saved_deg:.1f}°")
    print(f"저장 Position : {saved_pos}")

# 저장 각도로 이동
def move_saved_angle():

    if saved_pos is None:
        print("저장된 각도가 없습니다.")
        return

    result, error = servo.WritePos(
        SERVO_ID,
        saved_pos,
        0,
        SPEED
    )

    time.sleep(0.1)

    print("\n===== 저장 각도로 이동 =====")
    print(f"목표 각도 : {saved_deg:.1f}°")
    print(f"목표 Position : {saved_pos}")
    print(f"result : {result}")
    print(f"error : {error}")

    print_current_status()

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("Baudrate 실패")
    quit()

print("단일 서보 각도 저장 코드 시작")
print("s : 각도 저장")
print("m : 저장 각도로 이동")
print("r : 현재값 읽기")
print("q : 종료")

while True:

    # 항상 메뉴 전에 현재 상태 출력
    print_current_status()

    cmd = input("\n명령 입력 s/m/r/q : ")

    if cmd.lower() == "q":
        break

    elif cmd.lower() == "s":

        try:

            deg = float(
                input("저장할 각도 입력 0~220 : ")
            )

            save_angle(deg)

        except ValueError:
            print("숫자만 입력하세요.")

    elif cmd.lower() == "m":

        move_saved_angle()

    elif cmd.lower() == "r":

        print_current_status()

    else:
        print("s, m, r, q 중 하나를 입력하세요.")

portHandler.closePort()

print("프로그램 종료")
