from scservo_sdk import *
import time

PORT = "COM6"
BAUD = 1000000

SERVO_ID = 5

MAX_POS = 1023
MAX_DEG = 220.0

START_DEG = 0.0
TARGET_DEG = 220.0

NEW_SPEED = 300
NEW_ACC = 20

portHandler = PortHandler(PORT)
packetHandler = sms_sts(portHandler)

def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))

def deg_to_pos(deg):
    deg = clamp(deg, 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)

def pos_to_deg(pos):
    pos = clamp(pos, 0, MAX_POS)
    return pos * MAX_DEG / MAX_POS

def read_position():
    pos, result, error = packetHandler.ReadPos(SERVO_ID)

    if result == COMM_SUCCESS and 0 <= pos <= 1023:
        return pos

    return None

def move_to_deg(deg):
    target_pos = deg_to_pos(deg)

    result, error = packetHandler.WritePosEx(
        SERVO_ID,
        target_pos,
        NEW_SPEED,
        NEW_ACC
    )

    print(
        f"\n[명령] ID {SERVO_ID} | "
        f"목표각도:{deg:.1f}° | "
        f"목표Pos:{target_pos} | "
        f"speed:{NEW_SPEED} | acc:{NEW_ACC} | "
        f"result:{result} | error:{error}"
    )

def monitor_motion(duration=2.5):
    start = time.time()

    last_deg = None
    last_time = None

    print("\n시간(s) | Position | 각도(deg) | 계산속도(deg/s)")
    print("------------------------------------------------")

    while time.time() - start < duration:
        now = time.time() - start

        pos = read_position()

        if pos is not None:
            deg = pos_to_deg(pos)

            if last_deg is not None:
                dt = now - last_time
                speed = (deg - last_deg) / dt if dt > 0 else 0
            else:
                speed = 0

            print(
                f"{now:6.2f} | "
                f"{pos} | "
                f"{deg:.1f}° | "
                f"{speed:.2f}"
            )

            last_deg = deg
            last_time = now

        else:
            print(f"{now:6.2f} | 읽기실패 | - | -")

        time.sleep(0.05)

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("Baudrate 실패")
    quit()

print("신형 FT-SCS15 ID 5 회전 + 각도 기반 속도 측정")

while True:
    value = input("\n0도 이동 i / 220도 이동 s / 현재값 r / 종료 q : ")

    if value.lower() == "q":
        break

    elif value.lower() == "i":
        move_to_deg(START_DEG)
        monitor_motion(duration=2.5)

    elif value.lower() == "s":
        move_to_deg(TARGET_DEG)
        monitor_motion(duration=2.5)

    elif value.lower() == "r":
        pos = read_position()

        if pos is not None:
            deg = pos_to_deg(pos)
            print(f"현재 Position:{pos} | 현재각도:{deg:.1f}°")
        else:
            print("현재값 읽기 실패")

portHandler.closePort()
print("프로그램 종료")
