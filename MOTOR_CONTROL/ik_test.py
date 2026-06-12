import os
import numpy as np
import matplotlib

# GUI 창이 안 떠도 이미지 저장이 가능하도록 Agg 사용
# 라즈베리파이 SSH 환경에서도 동작
matplotlib.use("Agg")

import matplotlib.pyplot as plt

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

    print(f"\n[IMAGE] 저장 완료: {image_file}")
    print("[안내] GUI 창은 띄우지 않고 PNG 파일로 저장했습니다.")


def save_motor_angles(filename, motor_angles):
    with open(filename, "w", encoding="utf-8") as f:
        for key, value in motor_angles.items():
            f.write(f"{key}={value:.3f}\n")

    print(f"\n[SAVE] 저장 완료: {filename}")


def print_motor_angles(motor_angles):
    print("\n[모터 입력 각도]")
    print(f"deg1  : {motor_angles['deg1']:.2f}")
    print(f"deg23 : {motor_angles['deg23']:.2f}")
    print(f"deg45 : {motor_angles['deg45']:.2f}")
    print(f"deg6  : {motor_angles['deg6']:.2f}")
    print(f"deg7  : {motor_angles['deg7']:.2f}")


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


def move_real_robot(arm, motor_angles):
    print("\n[실제 모터 테스트 시작]")

    arm.smooth_move_all_pose(
        motor_angles["deg1"],
        motor_angles["deg23"],
        motor_angles["deg45"],
        motor_angles["deg6"],
        motor_angles["deg7"]
    )

    print("[실제 모터 이동 완료]")


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

    # 테스트 단계에서는 TCP 보정 제거
    if hasattr(ik_solver, "GRIPPER_TCP_LENGTH_MM"):
        ik_solver.GRIPPER_TCP_LENGTH_MM = 0.0

    joints = ik_solver.calculate_pick_joints(
        pick_position_mm
    )

    motor_angles = ik_solver.calculate_pick_ik(
        pick_position_mm
    )

    print_motor_angles(
        motor_angles
    )

    # TCP 보정 제거: pz를 그대로 사용
    target_position_m = [
        px / 1000.0,
        py / 1000.0,
        pz / 1000.0
    ]

    verify_fk(
        ik_solver.chain,
        joints,
        target_position_m
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

        input(
            "\nEnter 입력 시 IK 자세 이동"
        )

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

    joints = ik_solver.calculate_place_joints(
        load_position_mm,
        ik_solver.PLACE_ORIGIN_POSE_DEG
    )

    motor_angles = ik_solver.calculate_place_ik(
        load_position_mm,
        ik_solver.PLACE_ORIGIN_POSE_DEG
    )

    print_motor_angles(
        motor_angles
    )

    local_x_mm, local_y_mm = (
        ik_solver.rotate_xy_mm(
            lx,
            ly,
            ik_solver.PLACE_ORIGIN_POSE_DEG["deg1"]
        )
    )

    target_position_m = [
        local_x_mm / 1000.0,
        local_y_mm / 1000.0,
        lz / 1000.0
    ]

    verify_fk(
        ik_solver.chain,
        joints,
        target_position_m
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

        input(
            "\nEnter 입력 시 IK 자세 이동"
        )

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
        print("[INFO] GRIPPER_TCP_LENGTH_MM = 0.0 으로 설정")

    arm = RobotArm()

    print("\n===== IK 실모터 검증기 =====")
    print("[설정] TCP 보정 없음")
    print("[설정] 이미지 창 표시 없음 / PNG 저장 방식")

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
