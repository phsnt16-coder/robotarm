from scservo_sdk import *
import time
import os

# ============================================================
# motor.py
# 목적:
# - FT-SCS15 로봇암 모터 제어
# - 라즈베리파이 5 RP1 PWM 기반 진공 그리퍼 제어 통합
# - ACT_PICK / ACT_PLACE에서 IK 각도값 기반 이동 지원
#
# 포즈 구성:
# 0 : Safety Check
# 1 : Base
# 2 : Pick Ready
# 3 : Go To Place 고정 포즈
# 4 : ACT_PICK, Pick IK 기반 동작
# 5 : Rotation, Base 회전
# 6 : Place 기준 포즈, Place IK 계산 기준 원점
# 7 : ACT_PLACE, 6번 기준 포즈 + Place IK 기반 동작
# 8 : Gripper ON
# 9 : Gripper OFF
# ============================================================

# ================================
# 통신 설정
# ================================
# 라즈베리파이 사용 시 보통 /dev/ttyACM0 또는 /dev/ttyACM1
# 필요하면 실행 전 환경변수로 변경 가능:
# export ROBOT_PORT=/dev/ttyACM1
PORT = os.environ.get("ROBOT_PORT", "/dev/ttyACM0")
BAUD = 1000000

# ================================
# Position / 각도 범위
# ================================
MAX_POS = 1023
MAX_DEG = 220.0

# ================================
# 모터 ID 정의
# ================================
ID_1 = 1   # Base
ID_2 = 2   # Shoulder Right
ID_3 = 3   # Shoulder Left
ID_4 = 4   # Elbow Right, old, reverse
ID_5 = 5   # Elbow Left, new, forward
ID_6 = 6   # Wrist
ID_7 = 7   # Gripper rotation servo

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
# 기준 각도
# ================================
INIT_DEG_6 = 110.0


# 6번 포즈: Place IK 계산 기준 원점
POSE_6_PLACE_ORIGIN = {
    "deg1": 45.0,
    "deg23": 20.0,
    "deg45": 20.0,
    "deg6": INIT_DEG_6,
    "deg7": 0.0,
}

# ================================
# 보간 설정
# ================================
SMOOTH_STEP_COUNT = 30
SMOOTH_DELAY = 0.03

# ================================
# 그리퍼 설정
# ================================
ENABLE_GRIPPER = os.environ.get("ENABLE_GRIPPER", "1") != "0"
GRIPPER_PUMP_CHANNEL = 0
GRIPPER_VALVE_CHANNEL = 1
GRIPPER_HOLD_TIME = 3.0

# ================================
# 포트 / 서보 객체
# ================================
portHandler = PortHandler(PORT)
oldServo = scscl(portHandler)
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

# 6번 포즈 진입 여부
place_origin_ready = False

# 그리퍼 객체는 프로그램 시작 후 안전하게 초기화
_gripper = None


# ============================================================
# ESP32Emulator
# 라즈베리파이 5 RP1 PWM으로 ESP32-S3 특수 500 Hz 서보 신호 모방
# ============================================================
class ESP32Emulator:

    def __init__(self, pump_channel=0, valve_channel=1):
        print("\n[그리퍼 모듈] PWM 초기화")

        self.pwm_base = "/sys/class/pwm/pwmchip0"
        self.pump_ch = pump_channel
        self.valve_ch = valve_channel

        # 500 Hz = 2 ms period
        self.period_ns = 2_000_000

        # 실측 기반 duty
        self.p_high_ns = 1_637_800
        self.p_low_ns = 362_200

        if not os.path.exists(self.pwm_base):
            raise FileNotFoundError(
                f"PWM 경로가 없습니다: {self.pwm_base}. "
                "라즈베리파이 5에서 실행 중인지, PWM overlay 설정이 되었는지 확인하세요."
            )

        self._export_channel(self.pump_ch)
        self._export_channel(self.valve_ch)
        time.sleep(0.2)

        self._clear_channel(self.pump_ch)
        self._clear_channel(self.valve_ch)
        self._set_period(self.pump_ch, self.period_ns)
        self._set_period(self.valve_ch, self.period_ns)

    def _channel_path(self, channel):
        return f"{self.pwm_base}/pwm{channel}"

    def _export_channel(self, channel):
        if not os.path.exists(self._channel_path(channel)):
            with open(f"{self.pwm_base}/export", "w") as f:
                f.write(str(channel))
            time.sleep(0.1)

    def _clear_channel(self, channel):
        path = self._channel_path(channel)
        if not os.path.exists(path):
            return

        try:
            with open(f"{path}/enable", "w") as f:
                f.write("0")
        except OSError:
            pass

        try:
            with open(f"{path}/duty_cycle", "w") as f:
                f.write("0")
        except OSError:
            pass

    def _set_period(self, channel, period):
        path = self._channel_path(channel)
        with open(f"{path}/period", "w") as f:
            f.write(str(period))

    def _write_raw_duty(self, channel, duty_ns):
        path = self._channel_path(channel)
        with open(f"{path}/duty_cycle", "w") as f:
            f.write(str(duty_ns))
        with open(f"{path}/enable", "w") as f:
            f.write("1")

    def control(self, pump_on=True):
        if pump_on:
            self._write_raw_duty(self.pump_ch, self.p_high_ns)
            self._write_raw_duty(self.valve_ch, self.p_low_ns)
        else:
            self._write_raw_duty(self.pump_ch, self.p_low_ns)
            self._write_raw_duty(self.valve_ch, self.p_high_ns)

    def release_hardware(self):
        self._clear_channel(self.pump_ch)
        self._clear_channel(self.valve_ch)


# ============================================================
# 공통 유틸 함수
# ============================================================
def clamp(value, min_v, max_v):
    return max(min_v, min(max_v, value))


def deg_to_pos(deg):
    deg = clamp(float(deg), 0, MAX_DEG)
    return int(deg * MAX_POS / MAX_DEG)


def reverse_pos(pos):
    pos = clamp(int(pos), 0, MAX_POS)
    return MAX_POS - pos


def print_pose_angles(title, deg1, deg23, deg45, deg6, deg7):
    print(f"\n[{title}]")
    print(f"ID1  Base              : {float(deg1):.2f} deg")
    print(f"ID2/3 Shoulder Pair    : {float(deg23):.2f} deg")
    print(f"ID4/5 Elbow Pair       : {float(deg45):.2f} deg")
    print(f"ID6  Wrist             : {float(deg6):.2f} deg")
    print(f"ID7  Gripper Rotation  : {float(deg7):.2f} deg")


def make_ik_angles(deg1, deg23, deg45, deg6, deg7):
    return {
        "deg1": float(deg1),
        "deg23": float(deg23),
        "deg45": float(deg45),
        "deg6": float(deg6),
        "deg7": float(deg7),
    }


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


def pose_dict_to_args(pose):
    return (
        pose["deg1"],
        pose["deg23"],
        pose["deg45"],
        pose["deg6"],
        pose["deg7"],
    )


# ============================================================
# 그리퍼 제어 함수
# ============================================================
def init_gripper():
    global _gripper

    if not ENABLE_GRIPPER:
        print("[그리퍼] ENABLE_GRIPPER=0 설정. 그리퍼 비활성화")
        return None

    if _gripper is not None:
        return _gripper

    try:
        _gripper = ESP32Emulator(
            pump_channel=GRIPPER_PUMP_CHANNEL,
            valve_channel=GRIPPER_VALVE_CHANNEL,
        )
        return _gripper
    except Exception as e:
        print(f"[그리퍼 오류] 초기화 실패: {e}")
        print("[안내] 모터만 테스트하려면 ENABLE_GRIPPER=0 으로 실행하세요.")
        _gripper = None
        return None


def gripper_on():
    gripper = init_gripper()
    if gripper is None:
        print("[그리퍼] 흡착 ON 생략")
        return

    print("[그리퍼] 흡착 ON")
    gripper.control(True)
    time.sleep(GRIPPER_HOLD_TIME)


def gripper_off():
    gripper = init_gripper()
    if gripper is None:
        print("[그리퍼] 흡착 OFF 생략")
        return

    print("[그리퍼] 흡착 OFF")
    gripper.control(False)
    time.sleep(GRIPPER_HOLD_TIME)


def gripper_close():
    global _gripper

    if _gripper is None:
        return

    print("[그리퍼] 종료")
    try:
        _gripper.control(False)
        time.sleep(0.5)
        _gripper.release_hardware()
    finally:
        _gripper = None


# ============================================================
# 모터 단일 / 동기화 제어 함수
# ============================================================
def move_single_1(deg):
    global last_deg_1
    deg = clamp(float(deg), 0, MAX_DEG)
    pos = deg_to_pos(deg)
    last_deg_1 = deg
    oldServo.WritePos(ID_1, pos, 0, OLD_SPEED)


def move_single_6(deg):
    global last_deg_6
    deg = clamp(float(deg), 0, MAX_DEG)
    pos = deg_to_pos(deg)
    last_deg_6 = deg
    oldServo.WritePos(ID_6, pos, 0, OLD_SPEED)


def move_single_7(deg):
    global last_deg_7
    deg = clamp(float(deg), 0, MAX_DEG)
    pos = deg_to_pos(deg)
    last_deg_7 = deg
    oldServo.WritePos(ID_7, pos, 0, OLD_SPEED)


def move_sync_2_3(deg):
    global last_deg_23
    deg = clamp(float(deg), 0, MAX_DEG)
    logical_pos = deg_to_pos(deg)
    pos2 = logical_pos
    pos3 = reverse_pos(logical_pos)
    last_deg_23 = deg

    oldServo.WritePos(ID_2, pos2, 0, OLD_SPEED)
    oldServo.WritePos(ID_3, pos3, 0, OLD_SPEED)


def move_sync_4_5(deg):
    global last_deg_45
    deg = clamp(float(deg), 0, MAX_DEG)
    logical_pos = deg_to_pos(deg)
    pos5 = logical_pos
    pos4 = reverse_pos(logical_pos)
    last_deg_45 = deg

    # 신형 FT-SCS15: sms_sts는 WritePosEx 사용
    newServo.WritePosEx(ID_5, pos5, NEW_SPEED, NEW_ACC)

    # 구형 FT-SCS15
    oldServo.WritePos(ID_4, pos4, OLD_TIME, OLD_SPEED)


# ============================================================
# 전체 축 이동 / 보간 이동
# ============================================================
def move_all_pose(deg1, deg23, deg45, deg6, deg7):
    move_single_1(deg1)
    move_sync_2_3(deg23)
    move_sync_4_5(deg45)
    move_single_6(deg6)
    move_single_7(deg7)


def smooth_move_all_pose(target_deg1, target_deg23, target_deg45, target_deg6, target_deg7):
    global last_deg_1, last_deg_23, last_deg_45, last_deg_6, last_deg_7

    target_deg1 = float(target_deg1)
    target_deg23 = float(target_deg23)
    target_deg45 = float(target_deg45)
    target_deg6 = float(target_deg6)
    target_deg7 = float(target_deg7)

    start_deg1 = last_deg_1 if last_deg_1 is not None else target_deg1
    start_deg23 = last_deg_23 if last_deg_23 is not None else target_deg23
    start_deg45 = last_deg_45 if last_deg_45 is not None else target_deg45
    start_deg6 = last_deg_6 if last_deg_6 is not None else target_deg6
    start_deg7 = last_deg_7 if last_deg_7 is not None else target_deg7

    for i in range(1, SMOOTH_STEP_COUNT + 1):
        ratio = i / SMOOTH_STEP_COUNT

        deg1 = start_deg1 + (target_deg1 - start_deg1) * ratio
        deg23 = start_deg23 + (target_deg23 - start_deg23) * ratio
        deg45 = start_deg45 + (target_deg45 - start_deg45) * ratio
        deg6 = start_deg6 + (target_deg6 - start_deg6) * ratio
        deg7 = start_deg7 + (target_deg7 - start_deg7) * ratio

        move_all_pose(deg1, deg23, deg45, deg6, deg7)
        time.sleep(SMOOTH_DELAY)


# ============================================================
# 포즈 함수
# ============================================================
def pose_0_safety_check():
    print("\n===== POSE 0 : SAFETY CHECK =====")
    smooth_move_all_pose(0, 0, 0, 0, 0)
    time.sleep(0.5)
    smooth_move_all_pose(10, 10, 10, 10, 10)


def pose_1_base():
    print("\n===== POSE 1 : BASE =====")
    # 사용자가 말한 Z자 기준 자세
    smooth_move_all_pose(0, 30, 0, 200, 0)


def pose_2_pick_ready():
    print("\n===== POSE 2 : PICK READY =====")
    smooth_move_all_pose(0, 0, 0, INIT_DEG_6, 0)


def pose_3_go_to_place():
    print("\n===== POSE 3 : GO TO PLACE =====")
    smooth_move_all_pose(0, 60, -160, INIT_DEG_6, 0)
    )


def act_pick(box_type="A_1", ik_angles=None):
    print("\n===== POSE 4 : ACT_PICK =====")

    if ik_angles is None:
        print("[오류] ACT_PICK는 Pick IK 값이 필요합니다.")
        return False

    print(f"[대상 박스] {box_type}")
    print("[IK 사용] Pick IK 값으로 이동")

    print_pose_angles(
        "ACT_PICK Pick IK 각도",
        ik_angles["deg1"],
        ik_angles["deg23"],
        ik_angles["deg45"],
        ik_angles["deg6"],
        ik_angles["deg7"],
    )

    smooth_move_all_pose(
        ik_angles["deg1"],
        ik_angles["deg23"],
        ik_angles["deg45"],
        ik_angles["deg6"],
        ik_angles["deg7"],
    )

    gripper_on()
    return True


def rotation_mode(target_deg1=45):
    print("\n===== POSE 5 : ROTATION MODE =====")

    target_deg23 = last_deg_23 if last_deg_23 is not None else 0
    target_deg45 = last_deg_45 if last_deg_45 is not None else 0
    target_deg6 = last_deg_6 if last_deg_6 is not None else INIT_DEG_6
    target_deg7 = last_deg_7 if last_deg_7 is not None else 0

    smooth_move_all_pose(
        target_deg1,
        target_deg23,
        target_deg45,
        target_deg6,
        target_deg7,
    )


def pose_6_place_origin():
    global place_origin_ready

    print("\n===== POSE 6 : PLACE IK ORIGIN =====")
    print("[의미] 7번 ACT_PLACE의 Place IK 계산 기준 원점")
    print_pose_angles("POSE 6 기준 각도", *pose_dict_to_args(POSE_6_PLACE_ORIGIN))
    smooth_move_all_pose(*pose_dict_to_args(POSE_6_PLACE_ORIGIN))
    place_origin_ready = True


def act_place(ik_angles=None):
    global place_origin_ready

    print("\n===== POSE 7 : ACT_PLACE =====")

    if ik_angles is None:
        print("[오류] ACT_PLACE는 Place IK 값이 필요합니다.")
        return False

    if not place_origin_ready:
        print("[안내] 6번 Place 기준 포즈가 아직 실행되지 않아 먼저 6번 포즈로 이동합니다.")
        pose_6_place_origin()

    print("[IK 사용] 6번 기준 원점에서 계산된 Place IK 값으로 이동")

    print_pose_angles(
        "ACT_PLACE Place IK 각도",
        ik_angles["deg1"],
        ik_angles["deg23"],
        ik_angles["deg45"],
        ik_angles["deg6"],
        ik_angles["deg7"],
    )

    smooth_move_all_pose(
        ik_angles["deg1"],
        ik_angles["deg23"],
        ik_angles["deg45"],
        ik_angles["deg6"],
        ik_angles["deg7"],
    )

    gripper_off()
    return True


# ============================================================
# 포트 연결 / 종료
# ============================================================
def open_robot_port():
    print(f"\n[통신] 포트 열기: {PORT}")

    if not portHandler.openPort():
        print("[오류] 포트 열기 실패")
        return False

    if not portHandler.setBaudRate(BAUD):
        print("[오류] Baudrate 설정 실패")
        portHandler.closePort()
        return False

    print("[통신] FT-SCS15 연결 완료")
    return True


def close_robot():
    gripper_close()
    portHandler.closePort()
    print("\n프로그램 종료")


# ============================================================
# 메인 루프
# ============================================================
def main():
    if not open_robot_port():
        return

    print("\nFT-SCS15 + PWM 그리퍼 통합 제어 시작")

    try:
        while True:
            print("\n===== 포즈 선택 =====")
            print("0 : safety_check")
            print("1 : base, Z자 기준")
            print("2 : pick_ready")
            print("3 : go_to_place, ㄴ자 고정 포즈")
            print("4 : ACT_PICK, Pick IK 입력")
            print("5 : ROTATION, Base 45도")
            print("6 : PLACE IK 기준 원점")
            print("7 : ACT_PLACE, Place IK 입력")
            print("8 : 그리퍼 ON")
            print("9 : 그리퍼 OFF")
            print("q : 종료")

            menu = input("선택 : ").strip().lower()

            if menu == "q":
                break
            elif menu == "0":
                pose_0_safety_check()
            elif menu == "1":
                pose_1_base()
            elif menu == "2":
                pose_2_pick_ready()
            elif menu == "3":
                pose_3_go_to_place()
            elif menu == "4":
                pick_ik = input_ik_angles(
                    default=make_ik_angles(0, 20, 20, INIT_DEG_6, 0)
                )
                act_pick(box_type="A_1", ik_angles=pick_ik)
            elif menu == "5":
                rotation_mode(target_deg1=45)
            elif menu == "6":
                pose_6_place_origin()
            elif menu == "7":
                place_ik = input_ik_angles(
                    default=make_ik_angles(
                        POSE_6_PLACE_ORIGIN["deg1"],
                        POSE_6_PLACE_ORIGIN["deg23"],
                        POSE_6_PLACE_ORIGIN["deg45"],
                        POSE_6_PLACE_ORIGIN["deg6"],
                        POSE_6_PLACE_ORIGIN["deg7"],
                    )
                )
                act_place(ik_angles=place_ik)
            elif menu == "8":
                gripper_on()
            elif menu == "9":
                gripper_off()
            else:
                print("잘못된 입력")

    except KeyboardInterrupt:
        print("\n[중단] 사용자 종료")
    finally:
        close_robot()


if __name__ == "__main__":
    main()
