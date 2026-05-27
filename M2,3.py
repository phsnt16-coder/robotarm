from scservo_sdk import *

PORT = "COM6"
BAUD = 1000000

MAX_POS = 1023
MAX_DEG = 220.0

portHandler = PortHandler(PORT)
packetHandler = scscl(portHandler)

def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))

def deg_to_pos(deg):
    deg = clamp(deg, 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)

def convert_pos(pos, reverse=False):
    pos = clamp(pos, 0, MAX_POS)
    return MAX_POS - pos if reverse else pos

def read_temp(servo_id):

    temp, result, error = packetHandler.ReadTemperature(servo_id)

    if result == COMM_SUCCESS:
        return temp

    return "읽기실패"

def move_servo_deg(servo_id, deg, reverse=False, speed=500):

    logical_pos = deg_to_pos(deg)
    real_pos = convert_pos(logical_pos, reverse)

    result, error = packetHandler.WritePos(
        servo_id,
        real_pos,
        0,
        speed
    )

    temp = read_temp(servo_id)

    print(
        f"ID {servo_id} | "
        f"입력각도:{deg:.1f}° | "
        f"논리Pos:{logical_pos} | "
        f"실제전송:{real_pos} | "
        f"온도:{temp}°C | "
        f"result:{result} | error:{error}"
    )

# 2,3번 동기화
def move_sync_2_3(deg, speed=500):

    move_servo_deg(2, deg, reverse=False, speed=speed)
    move_servo_deg(3, deg, reverse=True, speed=speed)

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("baudrate 실패")
    quit()

print("FT-SCS15 제어 시작")
print("1번: 단독 정방향")
print("2번: 정방향")
print("3번: 역방향(2번과 동기화)")
print("각도 범위: 0 ~ 220도")

while True:

    print("\n==== 메뉴 ====")
    print("1 : 1번 모터 제어")
    print("2 : 2,3번 동기화 제어")
    print("q : 종료")

    menu = input("선택 : ")

    if menu.lower() == "q":
        break

    try:

        if menu == "1":

            deg = float(input("1번 모터 각도 입력 : "))

            move_servo_deg(
                servo_id=1,
                deg=deg,
                reverse=False,
                speed=500
            )

        elif menu == "2":

            deg = float(input("2,3번 동기화 각도 입력 : "))

            move_sync_2_3(
                deg=deg,
                speed=500
            )

        else:
            print("잘못된 입력")

    except:
        print("숫자 입력 오류")

portHandler.closePort()

print("프로그램 종료")