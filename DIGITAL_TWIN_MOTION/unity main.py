# main.py
# Raspberry Pi 실행용 메인 파일
# 같은 폴더에 live_bin_raspberry.py가 있어야 합니다.

from live_bin_raspberry import CoordinateConverter, RaspberryLiveBinTester


# =========================
# 1. 사용자 설정 부분
# =========================

# 적재함 크기(mm)
BIN_WIDTH = 200.0
BIN_HEIGHT = 200.0
BIN_DEPTH = 300.0

# 적재함 시작점 보정(mm)
# 현재는 알고리즘 좌표를 그대로 사용합니다.
# 로봇 기준 적재 시작 위치가 다르면 여기 값을 바꾸면 됩니다.
BIN_OFFSET_X = 0.0
BIN_OFFSET_Y = 0.0
BIN_OFFSET_Z = 0.0

# 로봇 좌표계 원점 보정(mm)
# 예: 적재함의 왼쪽 앞 아래 기준점이 로봇 좌표 (25, 70, 100)이면
# ROBOT_LOAD_ORIGIN = (25.0, 70.0, 100.0) 으로 변경
ROBOT_LOAD_ORIGIN = (0.0, 0.0, 0.0)


# =========================
# 2. 로봇 제어 연결 부분
# =========================

def send_to_robot(place_pos_mm, rotation_angle_deg, label):
    """
    여기에 실제 로봇암 이동 코드를 연결하면 됩니다.

    place_pos_mm 예: (70.0, 50.0, 0.0)
    rotation_angle_deg 예: 0.0 또는 90.0

    네 기존 main.py에 연결할 때는 아래처럼 사용하면 됩니다.
      - place_pos_mm -> robot_ik.py의 target position
      - rotation_angle_deg -> 손목/박스 회전 제어값
    """
    print("\n[ROBOT SEND]")
    print(f"라벨: {label}")
    print(f"적재 좌표(mm): {place_pos_mm}")
    print(f"박스 회전각(deg): {rotation_angle_deg}")

    # 예시 연결 형태:
    # x, y, z = place_pos_mm
    # ik_result = ik_controller.solve_mm(x, y, z)
    # motor.move_to_ik(ik_result)
    # wrist.rotate(rotation_angle_deg)
    # gripper.pump_off()


# =========================
# 3. 메인 실행부
# =========================

def main():
    converter = CoordinateConverter(
        origin_mm=ROBOT_LOAD_ORIGIN,
        sign=(1.0, 1.0, 1.0),
        axis_order=(0, 1, 2),
    )

    tester = RaspberryLiveBinTester(
        converter=converter,
        bin_width=BIN_WIDTH,
        bin_height=BIN_HEIGHT,
        bin_depth=BIN_DEPTH,
        bin_offset_x=BIN_OFFSET_X,
        bin_offset_y=BIN_OFFSET_Y,
        bin_offset_z=BIN_OFFSET_Z,
        print_label_debug=True,
        print_position_debug=True,
    )

    print("\n===== Raspberry Live Bin Main 시작 =====")
    print("입력 가능한 라벨: A1, A_1, B3, B_3, B8, B_8")
    print("종료: q")

    while True:
        label = input("\n인식된 박스 라벨 입력: ").strip()

        if label.lower() == "q":
            print("프로그램 종료")
            break

        result = tester.prepare_place_target_for_box(label)

        if result is None:
            print("[FAIL] 적재 좌표 계산 실패")
            continue

        place_pos_mm = result["place_pos_mm"]
        rotation_angle_deg = result["rotation_angle_deg"]
        normalized_label = result["label"]

        send_to_robot(
            place_pos_mm=place_pos_mm,
            rotation_angle_deg=rotation_angle_deg,
            label=normalized_label,
        )


if __name__ == "__main__":
    main()
