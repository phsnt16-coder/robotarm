from scservo_sdk import *
import time

PORT = "COM6"
BAUD = 1000000

SERVO_ID = 4

MAX_POS = 1023
MAX_DEG = 220.0

START_DEG = 0.0
TARGET_DEG = 220.0

MOVE_TIME = 2000
OLD_SPEED = 500

portHandler = PortHandler(PORT)
packetHandler = scscl(portHandler)

def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))

def deg_to_pos(deg):
    deg = clamp(deg, 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)

def pos_to_deg(pos):
    pos = clamp(pos, 0, MAX_POS)
    return pos * MAX_DEG / MAX_POS

def read_position():
    pos, result, error = packetHandler.read2ByteTxRx(SERVO_ID, 56)
    if result == COMM_SUCCESS:
        return pos
    return None

def read_velocity():
    vel, result, error = packetHandler.read2ByteTxRx(SERVO_ID, 58)
    if result == COMM_SUCCESS:
        return vel
    return None

def move_to_deg(deg):
    target_pos = deg_to_pos(deg)

    result, error = packetHandler.WritePos(
        SERVO_ID,
        target_pos,
        MOVE_TIME,
        OLD_SPEED
    )

    print(
        f"\n[명령] ID {SERVO_ID} | "
        f"목표각도:{deg:.1f}° | "
        f"목표Pos:{target_pos} | "
        f"time:{MOVE_TIME} | speed:{OLD_SPEED} | "
        f"result:{result} | error:{error}"
    )

def monitor_motion(duration=2.5):
    start = time.time()

    print("\n시간(s) | Position | 각도(deg) | Velocity")
    print("----------------------------------------")

    while time.time() - start < duration:
        now = time.time() - start

        pos = read_position()
        vel = read_velocity()

        if pos is not None:
            deg = pos_to_deg(pos)
        else:
            deg = None

        print(
            f"{now:6.2f} | "
            f"{pos} | "
            f"{deg:.1f}° | "
            f"{vel}"
        )

        time.sleep(0.05)

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("Baudrate 실패")
    quit()

print("구형 FT-SCS15 ID 4 회전 + 실시간 속도 읽기")

while True:
    value = input("\n0도 이동 i / 220도 이동 s / 현재값 r / 종료 q : ")

    if value.lower() == "q":
        break

    elif value.lower() == "i":
        move_to_deg(START_DEG)
        monitor_motion(duration=MOVE_TIME / 1000 + 0.5)

    elif value.lower() == "s":
        move_to_deg(TARGET_DEG)
        monitor_motion(duration=MOVE_TIME / 1000 + 0.5)

    elif value.lower() == "r":
        pos = read_position()
        vel = read_velocity()

        if pos is not None:
            deg = pos_to_deg(pos)
            print(f"현재 Position:{pos} | 현재각도:{deg:.1f}° | 현재속도:{vel}")
        else:
            print("현재값 읽기 실패")

portHandler.closePort()
print("프로그램 종료")
