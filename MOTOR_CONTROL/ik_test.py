import os
import numpy as np
import matplotlib

# GUI 창이 안 떠도 이미지 저장 가능
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


def print_debug_header(ik_solver):
    print("\n" + "=" * 60)
    print("[DEBUG] 실행 환경 확인")
    print("=" * 60)
    print(f"[DEBUG] 현재 작업 폴더     : {os.getcwd()}")
    print(f"[DEBUG] robot_ik import 경로: {robot_ik.__file__}")
    print(f"[DEBUG] URDF_PATH          : {URDF_PATH}")

    if hasattr(ik_solver, "SERVO_DIRECTIONS"):
        print(f"[DEBUG] SERVO_DIRECTIONS  : {ik_solver.SERVO_DIRECTIONS}")
    else:
        print("[DEBUG] SERVO_DIRECTIONS 없음")

    if hasattr(ik_solver, "SERVO_OFFSETS"):
        print(f"[DEBUG] SERVO_OFFSETS     : {ik_solver.SERVO_OFFSETS}")

    if hasattr(ik_solver, "PICK_ORIGIN_POSE_DEG"):
        print(f"[DEBUG] PICK_ORIGIN       : {ik_solver.PICK_ORIGIN_POSE_DEG}")

    if hasattr(ik_solver, "PLACE_ORIGIN_POSE_DEG"):
        print(f"[DEBUG] PLACE_ORIGIN      : {ik_solver.PLACE_ORIGIN_POSE_DEG}")

    if hasattr(ik_solver, "GRIPPER_TCP_LENGTH_MM"):
        print(f"[DEBUG] TCP 보정값        : {ik_solver.GRIPPER_TCP_LENGTH_MM}")

    print("=" * 60)


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
    print("\n[DEBUG] 실제 모터로 전달되는 각도")
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


def calculate_and_print_pick(ik_solver, pick_position_mm):
    # 테스트 단계에서는 TCP 보정 제거
    if hasattr(ik_solver, "GRIPPER_TCP_LENGTH_MM"):
        ik_solver.GRIPPER_TCP_LENGTH_MM = 0.0

    joints = ik_solver.calculate_pick_joints(
        pick_position_mm
    )

    motor_angles = ik_solver.calculate_pick_ik(
        pick_position_mm
    )

    x_mm, y_mm, z_mm = map(float, pick_position_mm)

    target_position_m = [
        x_mm / 1000.0,
        y_mm / 1000.0,
        z_mm / 1000.0
    ]

    print_joints(
        ik_solver.chain,
        joints
    )

    print_motor_angles(
        motor_angles
    )

    validate_motor_angles(
        motor_angles
    )

    verify_fk(
        ik_solver.chain,
        joints,
        target_position_m
    )

    return joints, motor_angles, target_position_m


def calculate_and_print_place(ik_solver, load_position_mm):
    place_origin = ik_solver.PLACE_ORIGIN_POSE_DEG

    joints = ik_solver.calculate_place_joints(
        load_position_mm,
        place_origin
    )

    motor_angles = ik_solver.calculate_place_ik(
        load_position_mm,
        place_origin
    )

    lx, ly, lz = map(float, load_position_mm)

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

    validate_motor_angles(
        motor_angles
    )

    verify_fk(
        ik_solver.chain,
        joints,
        target_position_m
    )

    return joints, motor_angles, target_position_m


def test_pick(ik_solver, arm):
    print("\n=== PICK 좌표 입력(mm) ===")
    print("[TCP 보정 없음] 입력한 Pick Z를 그대로 IK 목표 Z로 사용합니다.")

    px = float(input("Pick X : "))
    py = float(input("Pick Y : "))
    pz = float(input("Pick Z : "))

    pick_position_mm = [
        px,
        py,
        pz
    ]

    joints, motor_angles, target_position_m = calculate_and_print_pick(
        ik_solver,
        pick_position_mm
    )

    save_input = input(
        "\n각도 저장? (y/n) : "
    )

    if save_input.lower() == "y":
        save_motor_angles(
            "pick_ik_result.txt",
            motor_angles
        )

    image_input = input(
        "\nURDF 자세 이미지 저장? (y/n) : "
    )

    if image_input.lower() == "y":
        plot_chain(
            ik_solver.chain,
            joints,
            target_position_m,
            "PICK_IK_TEST_NO_TCP_OFFSET"
        )

    move_input = input(
        "\n실제 모터 이동? (y/n) : "
    )

    if move_input.lower() == "y":

        print("\n3번 포즈 이동")
        arm.pose_3_go_to_place()

        print("\n[DEBUG] 3번 포즈 이후 현재 명령각")
        if hasattr(arm, "get_current_pose"):
            print(arm.get_current_pose())

        move_real_robot(
            arm,
            motor_angles
        )


def test_place(ik_solver, arm):
    print("\n=== LOAD 좌표 입력(mm) ===")

    lx = float(input("Load X : "))
    ly = float(input("Load Y : "))
    lz = float(input("Load Z : "))

    load_position_mm = [
        lx,
        ly,
        lz
    ]

    joints, motor_angles, target_position_m = calculate_and_print_place(
        ik_solver,
        load_position_mm
    )

    save_input = input(
        "\n각도 저장? (y/n) : "
    )

    if save_input.lower() == "y":
        save_motor_angles(
            "place_ik_result.txt",
            motor_angles
        )

    image_input = input(
        "\nURDF 자세 이미지 저장? (y/n) : "
    )

    if image_input.lower() == "y":
        plot_chain(
            ik_solver.chain,
            joints,
            target_position_m,
            "PLACE_IK_TEST"
        )

    move_input = input(
        "\n실제 모터 이동? (y/n) : "
    )

    if move_input.lower() == "y":

        print("\n6번 포즈 이동")
        arm.pose_6_place_origin()

        print("\n[DEBUG] 6번 포즈 이후 현재 명령각")
        if hasattr(arm, "get_current_pose"):
            print(arm.get_current_pose())

        move_real_robot(
            arm,
            motor_angles
        )


def main():
    ik_solver = RobotIKController(
        URDF_PATH
    )

    # 테스트 단계에서는 TCP 보정 제거
    if hasattr(ik_solver, "GRIPPER_TCP_LENGTH_MM"):
        ik_solver.GRIPPER_TCP_LENGTH_MM = 0.0

    print_debug_header(
        ik_solver
    )

    arm = RobotArm()

    print("\n===== IK 실모터 검증기 =====")
    print("[설정] TCP 보정 없음")
    print("[설정] 이미지 창 표시 없음 / PNG 저장 방식")
    print("[설정] robot_ik 경로 및 SERVO_DIRECTIONS 출력")

    print("1 : Pick IK")
    print("2 : Place IK")
    print("3 : Pick + Place")
    print("q : 종료")

    mode = input(
        "\n선택 : "
    )

    if mode.lower() == "q":
        return

    arm.open_robot_port()
    arm.save_initial_positions()

    try:

        if mode == "1":

            test_pick(
                ik_solver,
                arm
            )

        elif mode == "2":

            test_place(
                ik_solver,
                arm
            )

        else:

            test_pick(
                ik_solver,
                arm
            )

            test_place(
                ik_solver,
                arm
            )

    finally:

        arm.close_robot()

        print(
            "\n[TEST 종료]"
        )


if __name__ == "__main__":
    main()
