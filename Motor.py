from scservo_sdk import *
import time
from gripper import ESP32Emulator

PORT = "/dev/ttyACM1"
BAUD = 1000000

MAX_POS = 1023
MAX_DEG = 220.0

ID_1 = 1
ID_2 = 2
ID_3 = 3
ID_4 = 4
ID_5 = 5
ID_6 = 6
ID_7 = 7

OLD_SPEED = 500
OLD_TIME = 2000
NEW_SPEED = 181
NEW_ACC = 1

SMOOTH_STEP_COUNT = 30
SMOOTH_DELAY = 0.03

POSITION_TOLERANCE = 50
POSITION_WAIT_TIMEOUT = 2.0
POSITION_CHECK_INTERVAL = 0.1

portHandler = PortHandler(PORT)
oldServo = scscl(portHandler)
newServo = sms_sts(portHandler)

gripper = ESP32Emulator(pump_channel=0, valve_channel=1)

last_deg_1 = 10.3
last_deg_23 = 7.1
last_deg_45 = 0.2
last_deg_6 = 209.9
last_deg_7 = 1.5

place_index = 0


PICK_RAW_POS_LIST = {
    "A_1": {"id1": 19, "id2": 405, "id3": 909, "id4": 904, "id6": 462, "id7": 11},
    "B_3": {"id1": 19, "id2": 405, "id3": 909, "id4": 904, "id6": 462, "id7": 11},
    "B_8": {"id1": 0, "id2": 429, "id3": 891, "id4": 833, "id6": 502, "id7": 59},
}


PLACE_POS_LIST = [
    {"id1": 454, "id2": 392, "id3": 928, "id4": 932, "id6": 413, "id7": 0},
    {"id1": 387, "id2": 392, "id3": 928, "id4": 932, "id6": 413, "id7": 0},
    {"id1": 454, "id2": 392, "id3": 928, "id4": 932, "id6": 413, "id7": 0},
    {"id1": 454, "id2": 392, "id3": 928, "id4": 932, "id6": 413, "id7": 0},
    {"id1": 454, "id2": 392, "id3": 928, "id4": 932, "id6": 413, "id7": 0},
]


PLACE_LIFT_POS_LIST = [
    {"id1": 450, "id2": 0, "id3": 1022, "id4": 816, "id6": 738, "id7": 0},
    {"id1": 387, "id2": 0, "id3": 1022, "id4": 816, "id6": 738, "id7": 0},
    {"id1": 484, "id2": 0, "id3": 1022, "id4": 816, "id6": 738, "id7": 0},
    {"id1": 365, "id2": 0, "id3": 1022, "id4": 816, "id6": 738, "id7": 0},
    {"id1": 467, "id2": 0, "id3": 1022, "id4": 816, "id6": 738, "id7": 0},
]


PICK_READY_RAW_POS = {
    "id1": 53,
    "id2": 31,
    "id3": 535,
    "id4": 1022,
    "id6": 990,
    "id7": 6
}


def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))


def deg_to_pos(deg):
    deg = clamp(deg, 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)


def pos_to_deg(pos):
    pos = clamp(pos, 0, MAX_POS)
    return pos * MAX_DEG / MAX_POS


def reverse_pos(pos):
    pos = int(clamp(pos, 0, MAX_POS))
    pos3 = 1320 - pos
    return MAX_POS - pos


def get_current_base_angle():
    return last_deg_1


def read_old_pos(servo_id):
    pos, result, error = oldServo.ReadPos(servo_id)
    if result != 0:
        return None
    return pos


def wait_until_positions(targets, title="위치 도착 대기"):
    print(f"\n[WAIT] {title}")
    start_time = time.time()

    while True:
        all_reached = True

        for servo_id, target_pos in targets.items():
            current_pos = read_old_pos(servo_id)

            if current_pos is None:
                all_reached = False
                continue

            if abs(current_pos - target_pos) > POSITION_TOLERANCE:
                all_reached = False

        if all_reached:
            print("[WAIT] 목표 위치 도착")
            return True

        if time.time() - start_time > POSITION_WAIT_TIMEOUT:
            print("[WAIT 경고] 위치 도착 대기 시간 초과")
            return False

        time.sleep(POSITION_CHECK_INTERVAL)


def init_motor():
    if not portHandler.openPort():
        print("[Motor] 포트 열기 실패")
        return False

    if not portHandler.setBaudRate(BAUD):
        print("[Motor] Baudrate 실패")
        return False

    print("[Motor] 연결 성공")
    return True


def close_motor():
    portHandler.closePort()
    print("[Motor] 포트 종료")


def gripper_on():
    print("[그리퍼] 흡착 ON")
    gripper.control(True)
    time.sleep(3.0)


def gripper_off():
    print("[그리퍼] 흡착 OFF / 진공 해제")
    gripper.control(False)
    time.sleep(5.0)


def gripper_close():
    print("[그리퍼] 종료")
    gripper.control(False)
    time.sleep(0.5)
    gripper.release_hardware()


def move_single_1(deg):
    global last_deg_1
    deg = clamp(deg, 0, 220)
    pos = deg_to_pos(deg)
    last_deg_1 = deg
    result, error = oldServo.WritePos(ID_1, pos, 0, OLD_SPEED)
    print(f"[ID1] deg:{deg:.1f}° -> pos:{pos} | result:{result} | error:{error}")


def move_single_6(deg):
    global last_deg_6
    deg = clamp(deg, 0, 220)
    pos = deg_to_pos(deg)
    last_deg_6 = deg
    result, error = oldServo.WritePos(ID_6, pos, 0, OLD_SPEED)
    print(f"[ID6] deg:{deg:.1f}° -> pos:{pos} | result:{result} | error:{error}")


def move_single_7(deg):
    global last_deg_7
    deg = clamp(deg, 0, 220)
    pos = deg_to_pos(deg)
    last_deg_7 = deg
    result, error = oldServo.WritePos(ID_7, pos, 0, OLD_SPEED)
    print(f"[ID7] deg:{deg:.1f}° -> pos:{pos} | result:{result} | error:{error}")


def move_sync_2_3(deg):
    global last_deg_23
    deg = clamp(deg, 0, 220)
    pos2 = deg_to_pos(deg)
    pos3 = reverse_pos(pos2)  # 위에서 수정된 1320 수식 자동 반영
    last_deg_23 = deg

    # 모터가 서로 싸울 물리적 충격을 완화하기 위해 OLD_SPEED 대신 OLD_TIME(2000ms) 강제 적용
    result2, error2 = oldServo.WritePos(ID_2, pos2, 1500, 0)
    result3, error3 = oldServo.WritePos(ID_3, pos3, 1500, 0)

    print(f"[ID2,3 보정완료] deg:{deg:.1f}° -> ID2:{pos2}, ID3:{pos3} (실측대칭)")

    print(
        f"[ID2,3] deg:{deg:.1f}° -> ID2:{pos2}, ID3:{pos3} | "
        f"result2:{result2}, error2:{error2} | "
        f"result3:{result3}, error3:{error3}"
    )


def move_sync_4_5(deg):
    global last_deg_45
    deg = clamp(deg, 0, 220)
    logical_pos = deg_to_pos(deg)
    pos4 = reverse_pos(logical_pos)
    pos5 = logical_pos
    last_deg_45 = deg

    result4, error4 = oldServo.WritePos(ID_4, pos4, OLD_TIME, OLD_SPEED)
    result5, error5 = newServo.WritePosEx(ID_5, pos5, NEW_SPEED, NEW_ACC)

    print(
        f"[ID4,5] deg:{deg:.1f}° -> ID4:{pos4}, ID5_cmd:{pos5} | "
        f"result4:{result4}, error4:{error4} | "
        f"result5:{result5}, error5:{error5}"
    )


def move_all_pose(deg1, deg23, deg45, deg6, deg7):
    move_single_1(deg1)
    move_sync_2_3(deg23)
    move_sync_4_5(deg45)
    move_single_6(deg6)
    move_single_7(deg7)


def smooth_move_all_pose(target_deg1, target_deg23, target_deg45, target_deg6, target_deg7):
    start_deg1 = last_deg_1
    start_deg23 = last_deg_23
    start_deg45 = last_deg_45
    start_deg6 = last_deg_6
    start_deg7 = last_deg_7

    for i in range(1, SMOOTH_STEP_COUNT + 1):
        ratio = i / SMOOTH_STEP_COUNT

        deg1 = start_deg1 + (target_deg1 - start_deg1) * ratio
        deg23 = start_deg23 + (target_deg23 - start_deg23) * ratio
        deg45 = start_deg45 + (target_deg45 - start_deg45) * ratio
        deg6 = start_deg6 + (target_deg6 - start_deg6) * ratio
        deg7 = start_deg7 + (target_deg7 - start_deg7) * ratio

        move_all_pose(deg1, deg23, deg45, deg6, deg7)
        time.sleep(SMOOTH_DELAY)

    wait_until_positions(
        {
            ID_1: deg_to_pos(target_deg1),
            ID_2: deg_to_pos(target_deg23),
            ID_3: reverse_pos(deg_to_pos(target_deg23)),
            ID_4: reverse_pos(deg_to_pos(target_deg45)),
            ID_6: deg_to_pos(target_deg6),
            ID_7: deg_to_pos(target_deg7),
        },
        title="SMOOTH MOVE 목표 위치 도착 대기"
    )


def move_raw_pos(pos_data, title="RAW POS 이동", move_id6=True):
    global last_deg_1, last_deg_23, last_deg_45, last_deg_6, last_deg_7

    pos1 = int(clamp(pos_data["id1"], 0, MAX_POS))
    pos2 = int(clamp(pos_data["id2"], 0, MAX_POS))
    
    # [초응급 보정] 리스트에 등록된 잘못된 id3 데이터를 무시하고 
    # 방금 타임라인 데이터로 검증된 실측 하드웨어 맵핑 구조로 강제 변환합니다.
    pos3 = reverse_pos(pos2) 
    
    pos4 = int(clamp(pos_data["id4"], 0, MAX_POS))
    pos6 = int(clamp(pos_data["id6"], 0, MAX_POS))
    pos7 = int(clamp(pos_data["id7"], 0, MAX_POS))

    last_deg_1 = pos_to_deg(pos1)
    last_deg_23 = pos_to_deg(pos2)
    last_deg_45 = pos_to_deg(reverse_pos(pos4))
    last_deg_7 = pos_to_deg(pos7)

    # 안전하게 도달 시간을 지정하여 구동
    result1, error1 = oldServo.WritePos(ID_1, pos1, 0, OLD_SPEED)
    result2, error2 = oldServo.WritePos(ID_2, pos2, 1500, 0)
    result3, error3 = oldServo.WritePos(ID_3, pos3, 1500, 0)
    result4, error4 = oldServo.WritePos(ID_4, pos4, OLD_TIME, OLD_SPEED)
    result7, error7 = oldServo.WritePos(ID_7, pos7, 0, OLD_SPEED)

    print(f"\n[{title}]")
    print(f"ID1 -> {pos1} | result:{result1} | error:{error1}")
    print(f"ID2 -> {pos2} | result:{result2} | error:{error2}")
    print(f"ID3 -> {pos3} | result:{result3} | error:{error3}")
    print(f"ID4 -> {pos4} | result:{result4} | error:{error4}")
    print(f"ID7 -> {pos7} | result:{result7} | error:{error7}")

    wait_targets = {
        ID_1: pos1,
        ID_2: pos2,
        ID_3: pos3,
        ID_4: pos4,
        ID_7: pos7,
    }

    if move_id6:
        last_deg_6 = pos_to_deg(pos6)
        result6, error6 = oldServo.WritePos(ID_6, pos6, 0, OLD_SPEED)
        print(f"ID6 -> {pos6} | result:{result6} | error:{error6}")
        wait_targets[ID_6] = pos6
    else:
        print("ID6 -> 유지 / 명령 전송 안 함")

    wait_until_positions(wait_targets, title=title)


def pose_1_base():
    print("\n[POSE 1] BASE")
    move_raw_pos(PICK_READY_RAW_POS, title="BASE RAW 이동", move_id6=True)


def pose_2_pick_ready_without_m6():
    print("\n==============================")
    print("POSE 2 : PICK READY RAW / M1~M5, M7 먼저")
    print("==============================")

    move_raw_pos(
        PICK_READY_RAW_POS,
        title="PICK READY RAW 복귀 / ID6 제외",
        move_id6=False
    )


def pose_2_pick_ready():
    pose_2_pick_ready_without_m6()

    print("\n[PICK READY] ID6 마지막 이동")
    move_single_6(pos_to_deg(PICK_READY_RAW_POS["id6"]))

    wait_until_positions(
        {
            ID_6: PICK_READY_RAW_POS["id6"]
        },
        title="PICK READY ID6 마지막"
    )


def pose_3_go_to_place():
    deg1, deg23, deg45, deg6, deg7 = 10.3, 20.0, 10.0, 170.0, 1.5
    print("\n[POSE 3] GO TO PICK")
    smooth_move_all_pose(deg1, deg23, deg45, deg6, deg7)


def act_pick(box_type="B_3", ik_angles=None):
    print("\n===== 4 : ACT_PICK / BOX별 RAW POS 반영 =====")

    pick_pos = PICK_RAW_POS_LIST.get(box_type, PICK_RAW_POS_LIST["B_3"])

    print(f"[PICK] box_type = {box_type}")
    print(f"[PICK] 적용 RAW POS = {pick_pos}")

    move_raw_pos(
        pick_pos,
        title=f"{box_type} PICK RAW POS 이동",
        move_id6=True
    )

    gripper_on()


def pose_5_wrist_rotate():
    deg1 = last_deg_1
    deg23 = last_deg_23
    deg45 = last_deg_45
    deg6 = 107.0
    deg7 = last_deg_7

    print("\n[POSE 5] WRIST ROTATE")
    smooth_move_all_pose(deg1, deg23, deg45, deg6, deg7)


def pose_6_rotation():
    base_target = clamp(last_deg_1 + 90.0, 0, 220)

    deg1 = base_target
    deg23 = 45.0
    deg45 = 10.0
    deg6 = 110.0
    deg7 = last_deg_7

    print("\n[POSE 6] PICK 기준 +90도 ROTATION")
    smooth_move_all_pose(deg1, deg23, deg45, deg6, deg7)


def pose_return_buffer_without_m6():
    print("\n[RETURN BUFFER] ID6 제외 / PICK READY 전 완충 자세")

    move_sync_4_5(60.0)
    time.sleep(0.4)

    move_sync_2_3(60.0)
    time.sleep(0.4)

    move_single_1(30.0)
    time.sleep(0.4)

    move_single_7(last_deg_7)
    time.sleep(0.4)


def pose_safe_return_mid():
    print("\n[RETURN] 중간 회피 자세 / ID6 제외")

    move_single_1(90.0)
    move_sync_2_3(20.0)
    move_sync_4_5(10.0)
    move_single_7(last_deg_7)

    wait_until_positions(
        {
            ID_1: deg_to_pos(90.0),
            ID_2: deg_to_pos(20.0),
            ID_4: reverse_pos(deg_to_pos(10.0)),
            ID_7: deg_to_pos(last_deg_7),
        },
        title="중간 회피 자세 / ID6 제외"
    )


def act_place(ik_angles=None):
    global place_index

    print("\n===== 7 : ACT_PLACE / 순서별 RAW POS 적재 =====")

    if place_index >= len(PLACE_POS_LIST):
        print("[PLACE] 저장된 적재 위치를 모두 사용했습니다.")
        return

    pos_data = PLACE_POS_LIST[place_index]

    print(f"[PLACE] {place_index + 1}번째 박스 위치로 이동")

    move_raw_pos(
        pos_data,
        title=f"PLACE {place_index + 1} RAW POS 이동",
        move_id6=True
    )

    time.sleep(0.5)

    gripper_off()

    time.sleep(0.5)

    lift_pos_data = PLACE_LIFT_POS_LIST[place_index]

    print(f"\n[PLACE] {place_index + 1}번째 박스 상승 자세로 이동 / ID6 유지")

    move_raw_pos(
        lift_pos_data,
        title=f"PLACE {place_index + 1} 후 상승 자세",
        move_id6=False
    )

    time.sleep(0.5)

    print("\n[PLACE] PICK READY 복귀 전 완충 자세 / ID6 유지")
    pose_return_buffer_without_m6()

    place_index += 1

    print("\n[PLACE 완료] PICK READY RAW 복귀 / ID6 유지")
    pose_2_pick_ready_without_m6()

    print("\n[PLACE 완료] 마지막으로 ID6 복귀")
    move_single_6(pos_to_deg(PICK_READY_RAW_POS["id6"]))

    wait_until_positions(
        {
            ID_6: PICK_READY_RAW_POS["id6"]
        },
        title="ID6 마지막 복귀"
    )


def emergency_stop_gripper():
    gripper_off()