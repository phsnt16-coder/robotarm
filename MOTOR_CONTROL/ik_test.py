import os
import inspect
import numpy as np
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

import robot_ik
from robot_ik import RobotIKController
from MOTOR_CONTROL import RobotArm


URDF_PATH = "mybot.urdf"


def safe_filename(title):
    return (
        title
        .replace(" ", "_")
        .replace("/", "_")
        .replace(":", "_")
        .replace("-", "_")
    )


def supports_label_argument(method):
    try:
        sig = inspect.signature(method)
        return "label" in sig.parameters
    except Exception:
        return False


def safe_return_to_initial(arm):
    """
    테스트 1회 종료 후 로봇암을 항상 초기 자세로 복귀시킨다.
    우선순위:
    1. return_to_initial()
    2. pose_1_base()
    """
    print("\n===== 초기 자세 복귀 =====")

    try:
        if hasattr(arm, "return_to_initial"):
            arm.return_to_initial()
            print("[RESET] return_to_initial() 실행 완료")
            return

        if hasattr(arm, "pose_1_base"):
            arm.pose_1_base()
            print("[RESET] pose_1_base() 실행 완료")
            return

        print("[RESET 경고] 초기화 함수가 없습니다.")

    except Exception as e:
        print(f"[RESET 오류] 초기 자세 복귀 실패: {e}")


def call_calculate_pick_joints(ik_solver, pick_position_mm, label):
    if supports_label_argument(ik_solver.calculate_pick_joints):
        return ik_solver.calculate_pick_joints(
            pick_position_mm,
            label=label,
        )

    print("[경고] 현재 robot_ik.py의 calculate_pick_joints()가 label 인자를 지원하지 않습니다.")
    print("[경고] 박스별 Pick Z Offset이 joints 계산에 직접 반영되지 않을 수 있습니다.")
    return ik_solver.calculate_pick_joints(
        pick_position_mm
    )


def call_calculate_pick_ik(ik_solver, pick_position_mm, label):
    if supports_label_argument(ik_solver.calculate_pick_ik):
        return ik_solver.calculate_pick_ik(
            pick_position_mm,
            label=label,
        )

    print("[경고] 현재 robot_ik.py의 calculate_pick_ik()가 label 인자를 지원하지 않습니다.")
    print("[경고] 기존 robot_ik.py의 기본 Z 보정값으로 계산합니다.")
    return ik_solver.calculate_pick_ik(
        pick_position_mm
    )


def plot_chain(chain, joints, target_position, title):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    chain.plot(
        joints,
        ax,
        target=target_position
    )

    ax.set_title(title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)

    image_file = safe_filename(title) + ".png"

    plt.savefig(
        image_file,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close(fig)

    print(f"\n[IMAGE] 저장 완료: {os.path.abspath(image_file)}")


def save_motor_angles(filename, motor_angles):
    with open(filename, "w", encoding="utf-8") as f:
        for key, value in motor_angles.items():
            f.write(f"{key}={value:.3f}\n")

    print(f"\n[SAVE] 저장 완료: {os.path.abspath(filename)}")


def print_motor_angles(motor_angles):
    print("\n[모터 입력 각도]")
    print(f"deg1  : {motor_angles['deg1']:.2f}")
    print(f"deg23 : {motor_angles['deg23']:.2f}")
    print(f"deg45 : {motor_angles['deg45']:.2f}")
    print(f"deg6  : {motor_angles['deg6']:.2f}")
    print(f"deg7  : {motor_angles['deg7']:.2f}")


def print_joints(chain, joints):
    print("\n[IK Raw Joint 결과]")
    for i, link in enumerate(chain.links):
        print(
            f"Link {i} ({link.name}) : "
            f"{joints[i]:.4f} rad / {np.degrees(joints[i]):.2f} deg"
        )


def verify_fk(chain, joints, target_position_m):
    computed_position = chain.forward_kinematics(joints)[:3, 3]

    distance_error = np.linalg.norm(
        np.array(target_position_m) -
        computed_position
    )

    print("\n[FK 검증]")
    print(f"목표 좌표      : {target_position_m}")
    print(f"계산 좌표      : {computed_position.tolist()}")
    print(f"직선 오차(mm)  : {distance_error * 1000:.3f}")


def validate_motor_angles(motor_angles):
    print("\n[각도 범위 검사]")

    ok = True

    for key, value in motor_angles.items():
        if not (0.0 <= float(value) <= 220.0):
            print(f"[경고] {key} 범위 초과: {value}")
            ok = False
        else:
            print(f"[OK] {key}: {value:.2f}")

    return ok


def move_real_robot(arm, motor_angles):
    print("\n[DEBUG] 실제 모터로 전달되는 현재 각도")
    print_motor_angles(motor_angles)

    input("\n안전 확인 후 Enter 입력 시 실제 모터 이동")

    print("\n[실제 모터 테스트 시작]")

    arm.smooth_move_all_pose(
        motor_angles["deg1"],
        motor_angles["deg23"],
        motor_angles["deg45"],
        motor_angles["deg6"],
        motor_angles["deg7"]
    )

    print("[실제 모터 이동 완료]")


def print_debug_header(ik_solver):
    print("\n" + "=" * 60)
    print("[DEBUG] 실행 환경 확인")
    print("=" * 60)
    print(f"[DEBUG] 현재 작업 폴더     : {os.getcwd()}")
    print(f"[DEBUG] robot_ik import 경로: {robot_ik.__file__}")
    print(f"[DEBUG] URDF_PATH          : {URDF_PATH}")

    print(f"[DEBUG] calculate_pick_joints label 지원: {supports_label_argument(ik_solver.calculate_pick_joints)}")
    print(f"[DEBUG] calculate_pick_ik label 지원    : {supports_label_argument(ik_solver.calculate_pick_ik)}")

    if hasattr(ik_solver, "SERVO_DIRECTIONS"):
        print(f"[DEBUG] SERVO_DIRECTIONS  : {ik_solver.SERVO_DIRECTIONS}")

    if hasattr(ik_solver, "PICK_Z_OFFSET_BY_LABEL"):
        print(f"[DEBUG] PICK_Z_OFFSET     : {ik_solver.PICK_Z_OFFSET_BY_LABEL}")
    else:
        print("[DEBUG] PICK_Z_OFFSET_BY_LABEL 없음")

    if hasattr(ik_solver, "PICK_ORIGIN_POSE_DEG"):
        print(f"[DEBUG] PICK_ORIGIN       : {ik_solver.PICK_ORIGIN_POSE_DEG}")

    if hasattr(ik_solver, "PLACE_ORIGIN_POSE_DEG"):
        print(f"[DEBUG] PLACE_ORIGIN      : {ik_solver.PLACE_ORIGIN_POSE_DEG}")

    print("=" * 60)


def input_float(prompt):
    while True:
        raw = input(prompt).strip()

        try:
            return float(raw)
        except ValueError:
            print("숫자만 입력하세요.")


def input_label():
    """
    박스 라벨 직접 입력.
    숫자 선택과 텍스트 입력 모두 허용.
    """
    allowed = ["A_1", "B_3", "B_8"]

    while True:
        print("\n박스 라벨 입력")
        print("1 : A_1")
        print("2 : B_3")
        print("3 : B_8")
        print("또는 A_1 / B_3 / B_8 직접 입력")

        raw = input("라벨 선택: ").strip()

        if raw == "1":
            return "A_1"

        if raw == "2":
            return "B_3"

        if raw == "3":
            return "B_8"

        if raw in allowed:
            return raw

        retry = input(
            f"[경고] 등록되지 않은 라벨 '{raw}' 입니다. 그대로 사용할까요? (y/n): "
        ).strip().lower()

        if retry == "y":
            return raw


def get_pick_target_position_m(ik_solver, pick_position_mm, label):
    px, py, pz = map(float, pick_position_mm)

    if hasattr(ik_solver, "get_pick_z_offset"):
        offset = ik_solver.get_pick_z_offset(label)
    elif hasattr(ik_solver, "GRIPPER_TCP_LENGTH_MM"):
        offset = ik_solver.GRIPPER_TCP_LENGTH_MM
    else:
        offset = 0.0

    target_z = pz - offset

    if target_z < 0:
        target_z = 0.0

    return [
        px / 1000.0,
        py / 1000.0,
        target_z / 1000.0
    ]


def test_pick_once(ik_solver, arm):
    print("\n=== PICK 새 좌표 입력(mm) ===")

    label = input_label()

    px = input_float("Pick X : ")
    py = input_float("Pick Y : ")
    pz = input_float("Pick Z : ")

    pick_position_mm = [
        px,
        py,
        pz
    ]

    print("\n[현재 입력값]")
    print(f"label : {label}")
    print(f"pick  : {pick_position_mm}")

    try:
        joints = call_calculate_pick_joints(
            ik_solver,
            pick_position_mm,
            label,
        )

        motor_angles = call_calculate_pick_ik(
            ik_solver,
            pick_position_mm,
            label,
        )

        target_position_m = get_pick_target_position_m(
            ik_solver,
            pick_position_mm,
            label,
        )

        print_joints(
            ik_solver.chain,
            joints
        )

        print_motor_angles(
            motor_angles
        )

        if not validate_motor_angles(motor_angles):
            print("[안전중단] 각도 범위 이상. 실제 모터 이동을 생략합니다.")
            return

        verify_fk(
            ik_solver.chain,
            joints,
            target_position_m
        )

        image_input = input(
            "\nURDF 자세 이미지 저장? (y/n) : "
        ).strip().lower()

        if image_input == "y":
            plot_chain(
                ik_solver.chain,
                joints,
                target_position_m,
                f"PICK_IK_TEST_{label}"
            )

        save_input = input(
            "\n현재 각도 txt 저장? (y/n) : "
        ).strip().lower()

        if save_input == "y":
            save_motor_angles(
                f"pick_ik_result_{label}.txt",
                motor_angles
            )

        move_input = input(
            "\n현재 입력값으로 실제 모터 이동? (y/n) : "
        ).strip().lower()

        if move_input == "y":
            print("\n3번 포즈 이동")
            arm.pose_3_go_to_place()

            if hasattr(arm, "get_current_pose"):
                print("\n[DEBUG] 3번 포즈 이후 현재 명령각")
                print(arm.get_current_pose())

            move_real_robot(
                arm,
                motor_angles
            )

    finally:
        safe_return_to_initial(arm)


def test_place_once(ik_solver, arm):
    print("\n=== LOAD 새 좌표 입력(mm) ===")

    lx = input_float("Load X : ")
    ly = input_float("Load Y : ")
    lz = input_float("Load Z : ")

    load_position_mm = [
        lx,
        ly,
        lz
    ]

    print("\n[현재 입력값]")
    print(f"load : {load_position_mm}")

    try:
        place_origin = ik_solver.PLACE_ORIGIN_POSE_DEG

        joints = ik_solver.calculate_place_joints(
            load_position_mm,
            place_origin
        )

        motor_angles = ik_solver.calculate_place_ik(
            load_position_mm,
            place_origin
        )

        local_x_mm, local_y_mm = ik_solver.rotate_xy_mm(
            lx,
            ly,
            place_origin["deg1"]
        )

        target_position_m = [
            local_x_mm / 1000.0,
            local_y_mm / 1000.0,
            lz / 1000.0
        ]

        print_joints(
            ik_solver.chain,
            joints
        )

        print_motor_angles(
            motor_angles
        )

        if not validate_motor_angles(motor_angles):
            print("[안전중단] 각도 범위 이상. 실제 모터 이동을 생략합니다.")
            return

        verify_fk(
            ik_solver.chain,
            joints,
            target_position_m
        )

        image_input = input(
            "\nURDF 자세 이미지 저장? (y/n) : "
        ).strip().lower()

        if image_input == "y":
            plot_chain(
                ik_solver.chain,
                joints,
                target_position_m,
                "PLACE_IK_TEST"
            )

        save_input = input(
            "\n현재 각도 txt 저장? (y/n) : "
        ).strip().lower()

        if save_input == "y":
            save_motor_angles(
                "place_ik_result.txt",
                motor_angles
            )

        move_input = input(
            "\n현재 입력값으로 실제 모터 이동? (y/n) : "
        ).strip().lower()

        if move_input == "y":
            print("\n6번 포즈 이동")
            arm.pose_6_place_origin()

            if hasattr(arm, "get_current_pose"):
                print("\n[DEBUG] 6번 포즈 이후 현재 명령각")
                print(arm.get_current_pose())

            move_real_robot(
                arm,
                motor_angles
            )

    finally:
        safe_return_to_initial(arm)


def main():
    ik_solver = RobotIKController(
        URDF_PATH
    )

    print_debug_header(
        ik_solver
    )

    arm = RobotArm()

    print("\n===== IK 실모터 반복 검증기 =====")
    print("[설정] 박스 라벨 직접 입력 가능")
    print("[설정] 이전 입력값 저장 없음")
    print("[설정] 매 테스트마다 새 입력값으로 IK 재계산")
    print("[설정] 테스트 종료마다 초기 자세 복귀")
    print("[설정] 이미지 창 표시 없음 / PNG 저장 방식")

    if not arm.open_robot_port():
        print("[오류] 로봇 포트 연결 실패")
        return

    arm.save_initial_positions()

    try:
        while True:
            print("\n메뉴")
            print("1 : Pick IK 새 입력 테스트")
            print("2 : Place IK 새 입력 테스트")
            print("3 : Pick + Place 새 입력 테스트")
            print("q : 종료")

            mode = input("\n선택 : ").strip().lower()

            if mode == "q":
                break

            if mode == "1":
                test_pick_once(
                    ik_solver,
                    arm
                )

            elif mode == "2":
                test_place_once(
                    ik_solver,
                    arm
                )

            elif mode == "3":
                test_pick_once(
                    ik_solver,
                    arm
                )

                test_place_once(
                    ik_solver,
                    arm
                )

            else:
                print("[INFO] 잘못된 입력")

    finally:
        safe_return_to_initial(arm)
        arm.close_robot()
        print("\n[TEST 종료]")


if __name__ == "__main__":
    main()
