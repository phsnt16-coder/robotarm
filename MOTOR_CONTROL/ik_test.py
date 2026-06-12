import os
import inspect
import time
import numpy as np
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

import robot_ik
from robot_ik import RobotIKController
from MOTOR_CONTROL import RobotArm


URDF_PATH = "mybot.urdf"


def supports_label_argument(method):
    try:
        sig = inspect.signature(method)
        return "label" in sig.parameters
    except Exception:
        return False


def input_float(prompt):
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print("숫자만 입력하세요.")


def input_label():
    print("\n박스 라벨 입력")
    print("1 : A_1")
    print("2 : B_3")
    print("3 : B_8")
    print("또는 직접 입력")

    raw = input("라벨 선택: ").strip()

    if raw == "1":
        return "A_1"
    if raw == "2":
        return "B_3"
    if raw == "3":
        return "B_8"
    if raw == "":
        return "A_1"

    return raw


def print_motor_angles(title, motor_angles):
    print(f"\n[{title}]")
    print(f"deg1  : {motor_angles['deg1']:.3f}")
    print(f"deg23 : {motor_angles['deg23']:.3f}")
    print(f"deg45 : {motor_angles['deg45']:.3f}")
    print(f"deg6  : {motor_angles['deg6']:.3f}")
    print(f"deg7  : {motor_angles['deg7']:.3f}")


def validate_motor_angles(motor_angles):
    for key in ["deg1", "deg23", "deg45", "deg6", "deg7"]:
        if key not in motor_angles:
            print(f"[오류] {key} 누락")
            return False

        value = float(motor_angles[key])

        if not (0.0 <= value <= 220.0):
            print(f"[오류] {key} 범위 초과: {value}")
            return False

    return True


def get_pick_target_position_m(ik_solver, pick_position_mm, label):
    px, py, pz = map(float, pick_position_mm)

    if hasattr(ik_solver, "get_pick_z_offset"):
        offset = ik_solver.get_pick_z_offset(label)
    elif hasattr(ik_solver, "GRIPPER_TCP_LENGTH_MM"):
        offset = ik_solver.GRIPPER_TCP_LENGTH_MM
    else:
        offset = 0.0

    target_z = pz - offset

    if target_z < 0.0:
        target_z = 0.0

    return [
        px / 1000.0,
        py / 1000.0,
        target_z / 1000.0
    ]


def verify_fk(ik_solver, joints, target_position_m):
    computed_position = ik_solver.chain.forward_kinematics(joints)[:3, 3]

    error_mm = np.linalg.norm(
        np.array(target_position_m) -
        computed_position
    ) * 1000.0

    print("\n[FK 검증]")
    print(f"목표 좌표 m : {target_position_m}")
    print(f"계산 좌표 m : {computed_position.tolist()}")
    print(f"오차 mm     : {error_mm:.3f}")


def safe_return_to_initial(arm):
    """
    사용자가 a를 입력했을 때만 호출된다.
    테스트 종료 후 자동 복귀에는 사용하지 않는다.
    """
    print("\n===== 초기 자세 복귀 =====")

    try:
        if hasattr(arm, "return_to_initial"):
            arm.return_to_initial()
        elif hasattr(arm, "pose_1_base"):
            arm.pose_1_base()
        else:
            print("[경고] 초기 복귀 함수 없음")
    except Exception as e:
        print(f"[경고] 초기 복귀 실패: {e}")


def make_new_ik_solver():
    """
    매 테스트마다 새로운 IK 객체를 생성한다.
    IK 계산값이 객체 내부에 남는 가능성을 차단한다.

    단, RobotArm의 last_deg 캐시는 지우지 않는다.
    last_deg는 IK 저장값이 아니라 보간 시작점이므로 유지해야 한다.
    """
    ik_solver = RobotIKController(URDF_PATH)

    print("\n[DEBUG] 새 RobotIKController 생성")
    print(f"[DEBUG] robot_ik path: {robot_ik.__file__}")

    if hasattr(ik_solver, "PICK_Z_OFFSET_BY_LABEL"):
        print(f"[DEBUG] PICK_Z_OFFSET_BY_LABEL: {ik_solver.PICK_Z_OFFSET_BY_LABEL}")

    if hasattr(ik_solver, "SERVO_DIRECTIONS"):
        print(f"[DEBUG] SERVO_DIRECTIONS: {ik_solver.SERVO_DIRECTIONS}")

    return ik_solver


def calculate_pick_once_no_cache(label, pick_position_mm):
    """
    - 매번 새 IK 객체 사용
    - calculate_pick_ik() 결과만 실제 이동에 사용
    - 이전 motor_angles를 전역/클래스에 저장하지 않음
    - RobotArm last_deg 캐시는 건드리지 않음
    """
    ik_solver = make_new_ik_solver()

    print("\n[현재 입력값]")
    print(f"label : {label}")
    print(f"pick  : {pick_position_mm}")

    if supports_label_argument(ik_solver.calculate_pick_ik):
        motor_angles = ik_solver.calculate_pick_ik(
            pick_position_mm,
            label=label,
        )
    else:
        print("[경고] calculate_pick_ik()가 label 인자를 지원하지 않습니다.")
        motor_angles = ik_solver.calculate_pick_ik(
            pick_position_mm
        )

    # FK 검증용 joints 계산
    if hasattr(ik_solver, "calculate_pick_joints"):
        if supports_label_argument(ik_solver.calculate_pick_joints):
            joints = ik_solver.calculate_pick_joints(
                pick_position_mm,
                label=label,
            )
        else:
            joints = ik_solver.calculate_pick_joints(
                pick_position_mm
            )

        target_position_m = get_pick_target_position_m(
            ik_solver,
            pick_position_mm,
            label,
        )

        verify_fk(
            ik_solver,
            joints,
            target_position_m,
        )

    print_motor_angles(
        "이번 입력으로 계산된 최종 motor_angles",
        motor_angles,
    )

    if not validate_motor_angles(motor_angles):
        print("[안전중단] motor_angles 범위 이상")
        return None

    command_id = int(time.time() * 1000)
    print(f"\n[COMMAND_ID] {command_id}")
    print("[INFO] 이 COMMAND_ID의 motor_angles만 실제 이동에 사용됩니다.")

    return motor_angles


def test_pick_no_cache(arm):
    print("\n===== Pick IK 무저장 테스트 =====")

    label = input_label()

    px = input_float("Pick X : ")
    py = input_float("Pick Y : ")
    pz = input_float("Pick Z : ")

    pick_position_mm = [
        px,
        py,
        pz,
    ]

    motor_angles = calculate_pick_once_no_cache(
        label,
        pick_position_mm,
    )

    if motor_angles is None:
        return

    move = input("\n이번 입력값으로 실제 모터 이동? (y/n): ").strip().lower()

    if move != "y":
        print("[SKIP] 실제 이동 생략")
        return

    print("\n[STEP] 3번 포즈 이동")
    arm.pose_3_go_to_place()

    print("\n[STEP] 이번 입력값의 IK 각도로 이동")
    print_motor_angles(
        "실제 모터로 전달 직전 motor_angles",
        motor_angles,
    )

    input("\n안전 확인 후 Enter 입력")

    arm.act_pick(
        box_type=label,
        ik_angles=motor_angles,
    )


def main():
    print("\n===== IK 무저장 / 새 입력 강제 테스터 =====")
    print("[목적] 좌표를 바꿔도 처음 입력값으로 움직이는 문제 분리")
    print("[정책] 매 Pick 테스트마다 새 RobotIKController 생성")
    print("[정책] 이전 motor_angles 저장/재사용 없음")
    print("[정책] RobotArm last_deg 캐시는 유지")
    print("[정책] 초기 자세 복귀는 a 입력 시에만 실행")

    arm = RobotArm()

    if not arm.open_robot_port():
        print("[오류] 로봇 포트 연결 실패")
        return

    arm.save_initial_positions()

    try:
        while True:
            print("\n메뉴")
            print("1 : Pick IK 새 입력 무저장 테스트")
            print("a : 초기 자세 복귀")
            print("q : 종료")

            mode = input("선택: ").strip().lower()

            if mode == "q":
                break

            if mode == "1":
                test_pick_no_cache(arm)

            elif mode == "a":
                safe_return_to_initial(arm)

            else:
                print("[INFO] 잘못된 입력")

    finally:
        # 종료 시 자동 초기화하지 않음
        arm.close_robot()
        print("\n[TEST 종료]")


if __name__ == "__main__":
    main()
