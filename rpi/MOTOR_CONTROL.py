#모터 제어용 모듈
from scservo_sdk import *
import time
import os
from gripper import ESP32Emulator

PORT = os.environ.get("ROBOT_PORT", "/dev/ttyACM0")
PORT_FALLBACK = "/dev/ttyACM0"
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

SERVO_IDS = [ID_1, ID_2, ID_3, ID_4, ID_5, ID_6, ID_7]

OLD_SPEED = 500
OLD_TIME = 2000

NEW_SPEED = 181
NEW_ACC = 1

INIT_DEG_6 = 110.0

OFFSET_DEG_23 = 20.0

PICK_READY_RAW_POS = {
    ID_1: 0,
    ID_2: 52,
    ID_3: 979,
    ID_4: 92,
    ID_5: 933,
    ID_6: 836,
    ID_7: 42,
}

SMOOTH_STEP_COUNT = 30
SMOOTH_DELAY = 0.03

ENABLE_GRIPPER = os.environ.get("ENABLE_GRIPPER", "1") != "0"
GRIPPER_PUMP_CHANNEL = 0
GRIPPER_VALVE_CHANNEL = 1
GRIPPER_HOLD_TIME = 3.0


def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))


def deg_to_pos(deg):
    deg = clamp(float(deg), 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)


def pos_to_deg(pos):
    pos = clamp(int(pos), 0, MAX_POS)
    return pos * MAX_DEG / MAX_POS


def reverse_pos(pos):
    pos = clamp(int(pos), 0, MAX_POS)
    return MAX_POS - pos


def make_ik_angles(deg1, deg23, deg45, deg6, deg7):
    return {
        "deg1": float(deg1),
        "deg23": float(deg23),
        "deg45": float(deg45),
        "deg6": float(deg6),
        "deg7": float(deg7),
    }


def print_pose_angles(title, deg1, deg23, deg45, deg6, deg7):
    print(f"\n[{title}]")
    print(f"ID1  Base              : {deg1:.2f} deg")
    print(f"ID2/3 Shoulder Pair    : {deg23:.2f} deg")
    print(f"ID4/5 Elbow Pair       : {deg45:.2f} deg")
    print(f"ID6  Wrist             : {deg6:.2f} deg")
    print(f"ID7  Gripper Rotation  : {deg7:.2f} deg")


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


def input_ik_angles(default=None):
    if default is None:
        default = make_ik_angles(0, 0, 0, INIT_DEG_6, 0)

    print("\nIK 각도 입력. 엔터만 누르면 기본값을 사용합니다.")
    print_pose_angles(
        "기본 IK 각도",
        default["deg1"],
        default["deg23"],
        default["deg45"],
        default["deg6"],
        default["deg7"],
    )

    raw = input("deg1 deg23 deg45 deg6 deg7 입력: ").strip()
    if raw == "":
        return default

    parts = raw.replace(",", " ").split()
    if len(parts) != 5:
        print("[오류] 각도 5개를 입력해야 합니다. 기본값을 사용합니다.")
        return default

    try:
        return make_ik_angles(*parts)
    except ValueError:
        print("[오류] 숫자 변환 실패. 기본값을 사용합니다.")
        return default


class RobotArm:
    def __init__(self, port=PORT, baud=BAUD):
        self.port = port
        self.baud = baud

        self.port_handler = PortHandler(self.port)
        self.old_servo = scscl(self.port_handler)

        self.last_deg_1 = None
        self.last_deg_23 = None
        self.last_deg_45 = None
        self.last_deg_6 = None
        self.last_deg_7 = None

        self.init_pos = {}
        self.is_open = False
        self.gripper = None

    def open(self):
        ports_to_try = [self.port]
        if self.port != PORT_FALLBACK:
            ports_to_try.append(PORT_FALLBACK)

        for port in ports_to_try:
            print(f"\n[통신] 포트 열기: {port}")
            self.port = port
            self.port_handler = PortHandler(self.port)
            self.old_servo = scscl(self.port_handler)

            if not self.port_handler.openPort():
                print(f"[오류] 포트 열기 실패: {port}")
                continue

            if not self.port_handler.setBaudRate(self.baud):
                print(f"[오류] Baudrate 설정 실패: {port}")
                self.port_handler.closePort()
                continue

            self.is_open = True
            print(f"[통신] FT-SCS15 연결 완료 ({port})")
            return True

        print("[오류] 모든 포트 연결 실패 (ttyACM0, ttyACM1)")
        return False

    def close(self):
        try:
            self.close_gripper()
        except Exception as e:
            print(f"[VACUUM] 종료 중 오류: {e}")

        if self.is_open:
            self.port_handler.closePort()
            self.is_open = False

        print("\n프로그램 종료")

    def open_robot_port(self):
        return self.open()

    def close_robot(self):
        return self.close()

    def read_old_position(self, servo_id):
        pos, result, error = self.old_servo.ReadPos(servo_id)
        if result == COMM_SUCCESS:
            return pos
        print(f"[읽기 실패] ID{servo_id} result:{result} error:{error}")
        return None

    def read_old_temp(self, servo_id):
        temp, result, error = self.old_servo.read1ByteTxRx(servo_id, 63)
        if result == COMM_SUCCESS:
            return temp
        return None

    def print_all_old_positions(self):
        print("\n" + "-" * 55)
        print("현재 모터 포지션 확인")
        print("-" * 55)

        for sid in SERVO_IDS:
            pos = self.read_old_position(sid)
            if pos is not None:
                deg = pos_to_deg(pos)
                print(f"▶ Motor ID {sid} -> Position: {pos} / {deg:.1f} deg")
            else:
                print(f"▶ Motor ID {sid} -> 읽기 실패")

        print("-" * 55)

    def save_initial_positions(self):
        for sid in [ID_1, ID_2, ID_3, ID_4, ID_5, ID_6, ID_7]:
            self.init_pos[sid] = self.read_old_position(sid)

        print("\n===== 초기 위치 저장 =====")
        for sid, pos in self.init_pos.items():
            if pos is not None:
                deg = pos_to_deg(pos)
                print(f"ID {sid} 초기 Position : {pos} / {deg:.1f} deg")
            else:
                print(f"ID {sid} 초기 Position : 읽기 실패")

    def return_to_initial(self):
        print("\n===== 초기값으로 복귀 =====")

        if self.init_pos.get(ID_1) is not None:
            self.old_servo.WritePos(ID_1, self.init_pos[ID_1], 0, OLD_SPEED)
            print(f"ID1 -> {self.init_pos[ID_1]}")

        if self.init_pos.get(ID_2) is not None:
            self.old_servo.WritePos(ID_2, self.init_pos[ID_2], 0, OLD_SPEED)
            print(f"ID2 -> {self.init_pos[ID_2]}")

        if self.init_pos.get(ID_3) is not None:
            self.old_servo.WritePos(ID_3, self.init_pos[ID_3], 0, OLD_SPEED)
            print(f"ID3 -> {self.init_pos[ID_3]}")

        if self.init_pos.get(ID_4) is not None:
            self.old_servo.WritePos(ID_4, self.init_pos[ID_4], OLD_TIME, OLD_SPEED)
            print(f"ID4 -> {self.init_pos[ID_4]}")

        if self.init_pos.get(ID_5) is not None:
            self.old_servo.WritePos(ID_5, self.init_pos[ID_5], OLD_TIME, OLD_SPEED)
            print(f"ID5 -> {self.init_pos[ID_5]}")

        if self.init_pos.get(ID_6) is not None:
            self.old_servo.WritePos(ID_6, self.init_pos[ID_6], 0, OLD_SPEED)
            print(f"ID6 -> {self.init_pos[ID_6]}")

        if self.init_pos.get(ID_7) is not None:
            self.old_servo.WritePos(ID_7, self.init_pos[ID_7], 0, OLD_SPEED)
            print(f"ID7 -> {self.init_pos[ID_7]}")

    def return_to_pick_ready(self):
        print("\n===== PICK_READY 복귀 =====")

        self.old_servo.WritePos(ID_1, PICK_READY_RAW_POS[ID_1], OLD_TIME, OLD_SPEED)
        self.old_servo.WritePos(ID_2, PICK_READY_RAW_POS[ID_2], OLD_TIME, OLD_SPEED)
        self.old_servo.WritePos(ID_3, PICK_READY_RAW_POS[ID_3], OLD_TIME, OLD_SPEED)
        self.old_servo.WritePos(ID_4, PICK_READY_RAW_POS[ID_4], OLD_TIME, OLD_SPEED)
        self.old_servo.WritePos(ID_5, PICK_READY_RAW_POS[ID_5], OLD_TIME, OLD_SPEED)
        self.old_servo.WritePos(ID_7, PICK_READY_RAW_POS[ID_7], OLD_TIME, OLD_SPEED)
        print("ID1~5, ID7 복귀 명령 완료")

        time.sleep(2.5)
        self.old_servo.WritePos(ID_6, PICK_READY_RAW_POS[ID_6], OLD_TIME, OLD_SPEED)
        print(f"ID6 -> {PICK_READY_RAW_POS[ID_6]}")

        self.last_deg_1  = pos_to_deg(PICK_READY_RAW_POS[ID_1])
        self.last_deg_23 = pos_to_deg(PICK_READY_RAW_POS[ID_2])
        self.last_deg_45 = pos_to_deg(PICK_READY_RAW_POS[ID_4])
        self.last_deg_6  = pos_to_deg(PICK_READY_RAW_POS[ID_6])
        self.last_deg_7  = pos_to_deg(PICK_READY_RAW_POS[ID_7])

    def move_single_1(self, deg):
        deg = clamp(float(deg), 0, MAX_DEG)
        pos = deg_to_pos(deg)
        self.last_deg_1 = deg
        self.old_servo.WritePos(ID_1, pos, 0, OLD_SPEED)

    def move_single_6(self, deg):
        deg = clamp(float(deg), 0, MAX_DEG)
        pos = deg_to_pos(deg)
        self.last_deg_6 = deg
        self.old_servo.WritePos(ID_6, pos, 0, OLD_SPEED)

    def move_single_7(self, deg):
        deg = clamp(float(deg), 0, MAX_DEG)
        pos = deg_to_pos(deg)
        self.last_deg_7 = deg
        self.old_servo.WritePos(ID_7, pos, 0, OLD_SPEED)

    def move_sync_2_3(self, deg):
        deg = clamp(float(deg), 0, MAX_DEG)
        actual_deg = clamp(deg + OFFSET_DEG_23, 0, MAX_DEG)
        logical_pos = deg_to_pos(actual_deg)
        pos2 = logical_pos
        pos3 = reverse_pos(logical_pos)
        self.last_deg_23 = deg
        self.old_servo.WritePos(ID_2, pos2, 0, OLD_SPEED)
        self.old_servo.WritePos(ID_3, pos3, 0, OLD_SPEED)

    def move_sync_4_5(self, deg):
        deg = clamp(float(deg), 10, MAX_DEG)
        logical_pos = deg_to_pos(deg)
        pos4 = logical_pos
        pos5 = reverse_pos(logical_pos)            
        self.last_deg_45 = deg
        self.old_servo.WritePos(ID_4, pos4, OLD_TIME, OLD_SPEED)
        self.old_servo.WritePos(ID_5, pos5, OLD_TIME, OLD_SPEED)

    def move_all_pose(self, deg1, deg23, deg45, deg6, deg7):
        self.move_single_1(deg1)
        self.move_sync_2_3(deg23)
        self.move_sync_4_5(deg45)
        self.move_single_6(deg6)
        self.move_single_7(deg7)

    def smooth_move_all_pose(self, target_deg1, target_deg23, target_deg45, target_deg6, target_deg7):
        target_deg1 = float(target_deg1)
        target_deg23 = float(target_deg23)
        target_deg45 = float(target_deg45)
        target_deg6 = float(target_deg6)
        target_deg7 = float(target_deg7)

        start_deg1 = self.last_deg_1 if self.last_deg_1 is not None else target_deg1
        start_deg23 = self.last_deg_23 if self.last_deg_23 is not None else target_deg23
        start_deg45 = self.last_deg_45 if self.last_deg_45 is not None else target_deg45
        start_deg6 = self.last_deg_6 if self.last_deg_6 is not None else target_deg6
        start_deg7 = self.last_deg_7 if self.last_deg_7 is not None else target_deg7

        for i in range(1, SMOOTH_STEP_COUNT + 1):
            ratio = i / SMOOTH_STEP_COUNT
            deg1 = start_deg1 + (target_deg1 - start_deg1) * ratio
            deg23 = start_deg23 + (target_deg23 - start_deg23) * ratio
            deg45 = start_deg45 + (target_deg45 - start_deg45) * ratio
            deg6 = start_deg6 + (target_deg6 - start_deg6) * ratio
            deg7 = start_deg7 + (target_deg7 - start_deg7) * ratio
            self.move_all_pose(deg1, deg23, deg45, deg6, deg7)
            time.sleep(SMOOTH_DELAY)

    def pose_0_safety_check(self):
        print("\n===== POSE 0 : SAFETY CHECK =====")
        self.return_to_pick_ready()

    def pose_1_base(self):
        print("\n===== POSE 1 : BASE =====")
        self.return_to_pick_ready()

    def pose_2_pick_ready(self):
        print("\n===== POSE 2 : PICK READY =====")
        self.return_to_pick_ready()

    def pose_3_go_to_place(self):
        print("\n===== POSE 3 : GO TO PLACE =====")
        self.smooth_move_all_pose(0, 50, 100, INIT_DEG_6, 0)

    def pose_5_rotation(self, target_deg1=45):
        print("\n===== POSE 5 : ROTATION MODE =====")
        target_deg23 = self.last_deg_23 if self.last_deg_23 is not None else 0
        target_deg45 = self.last_deg_45 if self.last_deg_45 is not None else 75
        target_deg6 = self.last_deg_6 if self.last_deg_6 is not None else INIT_DEG_6
        target_deg7 = self.last_deg_7 if self.last_deg_7 is not None else 0
        self.smooth_move_all_pose(target_deg1, target_deg23, target_deg45, target_deg6, target_deg7)

    def rotation_mode(self, target_deg1=45):
        return self.pose_5_rotation(target_deg1)

    def pose_6_place_origin(self):
        print("\n===== POSE 6 : PLACE IK ORIGIN =====")
        current_deg23 = self.last_deg_23 if self.last_deg_23 is not None else 50.0
        current_deg45 = self.last_deg_45 if self.last_deg_45 is not None else 75.0
        current_deg6  = self.last_deg_6  if self.last_deg_6  is not None else 110.0
        self.smooth_move_all_pose(90, current_deg23, current_deg45, current_deg6, 0)
        time.sleep(0.5)
        self.smooth_move_all_pose(90, 80.0, 50.0, INIT_DEG_6, 0)

    def act_pick(self, box_type="A_1", ik_angles=None):
        print("\n===== POSE 4 : ACT_PICK =====")
        if ik_angles is None:
            print("[오류] ACT_PICK는 IK 값이 필요합니다.")
            return False

        print(f"[대상 박스] {box_type}")
        print("[IK 사용] Pick IK 값으로 이동")
        print_pose_angles(
            "ACT_PICK IK 각도",
            ik_angles["deg1"],
            ik_angles["deg23"],
            ik_angles["deg45"],
            ik_angles["deg6"],
            ik_angles["deg7"],
        )
        self.smooth_move_all_pose(
            ik_angles["deg1"],
            ik_angles["deg23"],
            ik_angles["deg45"],
            ik_angles["deg6"],
            ik_angles["deg7"],
        )
        return True

    def act_place(self, ik_angles=None):
        print("\n===== POSE 7 : ACT_PLACE =====")
        if ik_angles is None:
            print("[오류] ACT_PLACE는 IK 값이 필요합니다.")
            return False

        print("[IK 사용] Place IK 값으로 이동")
        print_pose_angles(
            "ACT_PLACE IK 각도",
            ik_angles["deg1"],
            ik_angles["deg23"],
            ik_angles["deg45"],
            ik_angles["deg6"],
            ik_angles["deg7"],
        )
        self.smooth_move_all_pose(
            ik_angles["deg1"],
            ik_angles["deg23"],
            ik_angles["deg45"],
            ik_angles["deg6"],
            ik_angles["deg7"],
        )
        return True

    # ========================================================
    # 메인 Pick & Place 시퀀스
    # ========================================================
    def execute_pick_and_place(self, ik_solver, label, pick, load, angle):
        px, py, pz = pick
        lx, ly, lz = load

        px = px - 13.3
        py = py - 120.0

        print("\n[SEQ 1] PICK_READY")
        self.pose_2_pick_ready()
        time.sleep(0.5)

        print("\n[SEQ 2] PICK pose")
        self.pose_3_go_to_place()
        time.sleep(0.5)

        print("\n[SEQ 2-1] Base align to ArUco angle")
        angle_normalized = angle % 360
        if angle_normalized > 180:
            angle_normalized -= 360
        current_deg23 = self.last_deg_23 if self.last_deg_23 is not None else 50.0
        current_deg45 = self.last_deg_45 if self.last_deg_45 is not None else 75.0
        current_deg6  = self.last_deg_6  if self.last_deg_6  is not None else 110.0
        self.smooth_move_all_pose(angle_normalized, current_deg23, current_deg45, current_deg6, 0)
        time.sleep(0.5)

        print("\n[SEQ 3] ACT_PICK")
        pick_ik = ik_solver.calculate_pick_ik([px, py, pz])
        pick_ik["deg7"] = 10.0 + angle_normalized
        self.act_pick(box_type=label, ik_angles=pick_ik)

        print("\n[VACUUM ON]")
        self.vacuum_on()
        time.sleep(0.3)

        print("\n[SEQ box] box up")
        current_deg1  = self.last_deg_1  if self.last_deg_1  is not None else 0.0
        current_deg6  = self.last_deg_6  if self.last_deg_6  is not None else 110.0
        current_deg23 = self.last_deg_23 if self.last_deg_23 is not None else 64.0
        current_deg45 = self.last_deg_45 if self.last_deg_45 is not None else 74.0

        # box up: elbow 같이 펴기
        self.smooth_move_all_pose(current_deg1, 35.0, 120.0, current_deg6, 0)
        time.sleep(0.3)
        # Base만 90도로 회전 (Shoulder/Elbow 유지)
        self.smooth_move_all_pose(90.0, 35.0, 120.0, current_deg6, 0)
        time.sleep(0.5)

        print("\n[SEQ 7] ACT_PLACE")
        place_ik = ik_solver.calculate_place_ik([lx, ly, lz])

        print(f"[PLACE IK result]")
        print(f"  deg1  : {place_ik['deg1']:.1f}")
        print(f"  deg23 : {place_ik['deg23']:.1f}")
        print(f"  deg45 : {place_ik['deg45']:.1f}")
        print(f"  deg6  : {place_ik['deg6']:.1f}")
        print(f"  deg7  : {place_ik['deg7']:.1f}")

        # 1단계: Elbow 접고 목표 위치로 이동
        self.smooth_move_all_pose(
            place_ik["deg1"], place_ik["deg23"], 146.5,
            place_ik["deg6"], place_ik["deg7"],
        )
        time.sleep(0.8)
        # 2단계: Shoulder 뒤로 당긴 상태로 Elbow 내리기
        self.smooth_move_all_pose(
            place_ik["deg1"], place_ik["deg23"] - 20.0, place_ik["deg45"],
            place_ik["deg6"], place_ik["deg7"],
        )
        time.sleep(0.8)
        # 3단계: Shoulder 목표 위치로
        self.smooth_move_all_pose(
            place_ik["deg1"], place_ik["deg23"], place_ik["deg45"],
            place_ik["deg6"], place_ik["deg7"],
        )
        time.sleep(0.5)


        print("\n[VACUUM OFF]")
        self.vacuum_off()
        time.sleep(0.3)

        current = self.get_current_pose()
        current_deg1  = current["deg1"]  if current["deg1"]  is not None else 90.0
        current_deg23 = current["deg23"] if current["deg23"] is not None else 80.0
        current_deg6  = current["deg6"]  if current["deg6"]  is not None else 110.0
        current_deg7  = current["deg7"]  if current["deg7"]  is not None else 0.0

        print("\n[SEQ return 1] Elbow up")
        self.smooth_move_all_pose(current_deg1, current_deg23, 120.0, current_deg6, current_deg7)
        time.sleep(0.5)

        print("\n[SEQ return 2] Shoulder up")
        self.smooth_move_all_pose(current_deg1, 50.0, 120.0, current_deg6, current_deg7)
        time.sleep(0.5)

        print("\n[SEQ return 3] Wrist level + Base rotate")
        self.smooth_move_all_pose(0.0, 50.0, 120.0, 110.0, current_deg7)
        time.sleep(0.5)

        print("\n[SEQ return 4] pose_3")
        self.smooth_move_all_pose(0.0, 50.0, 75.0, 110.0, current_deg7)
        time.sleep(0.5)

        print("\n[SEQ return 5] PICK_READY")
        self.return_to_pick_ready()

        print("\n[SEQUENCE] done")

    # ========================================================
    # 그리퍼
    # ========================================================
    def init_gripper(self):
        if not ENABLE_GRIPPER:
            print("[VACUUM] ENABLE_GRIPPER=0 설정. 진공 제어 비활성화")
            return None
        if self.gripper is not None:
            return self.gripper
        try:
            self.gripper = ESP32Emulator(
                pump_channel=GRIPPER_PUMP_CHANNEL,
                valve_channel=GRIPPER_VALVE_CHANNEL,
            )
            return self.gripper
        except Exception as e:
            print(f"[VACUUM 오류] 초기화 실패: {e}")
            print("[안내] 모터만 테스트하려면 ENABLE_GRIPPER=0 으로 실행하세요.")
            self.gripper = None
            return None

    def vacuum_on(self):
        gripper = self.init_gripper()
        if gripper is None:
            print("[VACUUM] ON 생략")
            return
        print("[VACUUM] ON")
        gripper.control(True)
        time.sleep(GRIPPER_HOLD_TIME)

    def vacuum_off(self):
        gripper = self.init_gripper()
        if gripper is None:
            print("[VACUUM] OFF 생략")
            return
        print("[VACUUM] OFF")
        gripper.control(False)
        time.sleep(GRIPPER_HOLD_TIME)

    def close_gripper(self):
        if self.gripper is None:
            return
        print("[VACUUM] 종료")
        try:
            self.gripper.control(False)
            time.sleep(0.5)
            self.gripper.release_hardware()
        finally:
            self.gripper = None

    def get_current_pose(self):
        return {
            "deg1": self.last_deg_1,
            "deg23": self.last_deg_23,
            "deg45": self.last_deg_45,
            "deg6": self.last_deg_6,
            "deg7": self.last_deg_7,
        }

    def manual_debug_loop(self):
        print("FT-SCS15 순차 각도 제어 시작")
        print("N : 기존 각도 유지")
        print("O : 초기값 복귀")
        print("P : 현재 포지션 확인")
        print("q : 종료")

        self.save_initial_positions()

        while True:
            print("\n===== 순차 입력 시작 =====")

            deg1 = input_angle_or_command("ID1 단일", self.last_deg_1)
            if deg1 == "q": break
            if deg1 == "O": self.return_to_initial(); continue
            if deg1 is not None: self.move_single_1(deg1)
            else: print("ID1 기존값 없음 → 이동 생략")

            deg23 = input_angle_or_command("ID2,3 동기화", self.last_deg_23)
            if deg23 == "q": break
            if deg23 == "O": self.return_to_initial(); continue
            if deg23 is not None: self.move_sync_2_3(deg23)
            else: print("ID2,3 기존값 없음 → 이동 생략")

            deg45 = input_angle_or_command("ID4,5 동기화", self.last_deg_45)
            if deg45 == "q": break
            if deg45 == "O": self.return_to_initial(); continue
            if deg45 is not None: self.move_sync_4_5(deg45)
            else: print("ID4,5 기존값 없음 → 이동 생략")

            deg6 = input_angle_or_command("ID6 단일", self.last_deg_6)
            if deg6 == "q": break
            if deg6 == "O": self.return_to_initial(); continue
            if deg6 is not None: self.move_single_6(deg6)
            else: print("ID6 기존값 없음 → 이동 생략")

            deg7 = input_angle_or_command("ID7 단일", self.last_deg_7)
            if deg7 == "q": break
            if deg7 == "O": self.return_to_initial(); continue
            if deg7 is not None: self.move_single_7(deg7)
            else: print("ID7 기존값 없음 → 이동 생략")

            print("\n===== 한 사이클 완료 =====")

    def pose_menu_loop(self):
        print("\nFT-SCS15 포즈 제어 시작")

        while True:
            print("\n===== 포즈 선택 =====")
            print("0 : safety_check")
            print("1 : base")
            print("2 : pick_ready")
            print("3 : go_to_place")
            print("4 : ACT_PICK, IK 입력")
            print("5 : ROTATION, Base 45도")
            print("6 : PLACE IK ORIGIN")
            print("7 : ACT_PLACE, IK 입력")
            print("8 : 현재 포지션 확인")
            print("9 : 초기값 복귀")
            print("q : 종료")

            menu = input("선택 : ").strip().lower()

            if menu == "q":
                break
            elif menu == "0":
                self.pose_0_safety_check()
            elif menu == "1":
                self.pose_1_base()
            elif menu == "2":
                self.pose_2_pick_ready()
            elif menu == "3":
                self.pose_3_go_to_place()
            elif menu == "4":
                ik = input_ik_angles(default=make_ik_angles(0, 50, 75, INIT_DEG_6, 0))
                self.act_pick(box_type="A_1", ik_angles=ik)
            elif menu == "5":
                self.rotation_mode(target_deg1=45)
            elif menu == "6":
                self.pose_6_place_origin()
            elif menu == "7":
                ik = input_ik_angles(default=make_ik_angles(90, 40, 75, INIT_DEG_6, 0))
                self.act_place(ik_angles=ik)
            elif menu == "8":
                self.print_all_old_positions()
            elif menu == "9":
                self.return_to_initial()
            else:
                print("잘못된 입력")


# ============================================================
# 기본 인스턴스 래퍼
# ============================================================
_DEFAULT_ARM = None


def get_default_arm():
    global _DEFAULT_ARM
    if _DEFAULT_ARM is None:
        _DEFAULT_ARM = RobotArm()
    return _DEFAULT_ARM


def open_robot_port():
    return get_default_arm().open_robot_port()

def close_robot():
    return get_default_arm().close_robot()

def get_current_pose():
    return get_default_arm().get_current_pose()

def vacuum_on():
    return get_default_arm().vacuum_on()

def vacuum_off():
    return get_default_arm().vacuum_off()

def save_initial_positions():
    return get_default_arm().save_initial_positions()

def return_to_initial():
    return get_default_arm().return_to_initial()

def print_all_old_positions():
    return get_default_arm().print_all_old_positions()

def move_single_1(deg):
    return get_default_arm().move_single_1(deg)

def move_sync_2_3(deg):
    return get_default_arm().move_sync_2_3(deg)

def move_sync_4_5(deg):
    return get_default_arm().move_sync_4_5(deg)

def move_single_6(deg):
    return get_default_arm().move_single_6(deg)

def move_single_7(deg):
    return get_default_arm().move_single_7(deg)

def move_all_pose(deg1, deg23, deg45, deg6, deg7):
    return get_default_arm().move_all_pose(deg1, deg23, deg45, deg6, deg7)

def smooth_move_all_pose(deg1, deg23, deg45, deg6, deg7):
    return get_default_arm().smooth_move_all_pose(deg1, deg23, deg45, deg6, deg7)

def pose_0_safety_check():
    return get_default_arm().pose_0_safety_check()

def pose_1_base():
    return get_default_arm().pose_1_base()

def pose_2_pick_ready():
    return get_default_arm().pose_2_pick_ready()

def pose_3_go_to_place():
    return get_default_arm().pose_3_go_to_place()

def rotation_mode(target_deg1=45):
    return get_default_arm().rotation_mode(target_deg1)

def pose_6_place_origin():
    return get_default_arm().pose_6_place_origin()

def act_pick(box_type="A_1", ik_angles=None):
    return get_default_arm().act_pick(box_type=box_type, ik_angles=ik_angles)

def act_place(ik_angles=None):
    return get_default_arm().act_place(ik_angles=ik_angles)


def main():
    arm = RobotArm()

    if not arm.open():
        return

    try:
        print("\n실행 모드 선택")
        print("1 : 수동 디버그 순차 입력")
        print("2 : 포즈 메뉴")
        mode = input("선택 : ").strip()

        if mode == "1":
            arm.manual_debug_loop()
        else:
            arm.save_initial_positions()
            arm.pose_menu_loop()

    except KeyboardInterrupt:
        print("\n[중단] 사용자 종료")

    finally:
        arm.close()


if __name__ == "__main__":
    main()
