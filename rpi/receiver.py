import time

from robot_ik import RobotIKController
from MOTOR_CONTROL import RobotArm


def parse_packet(packet_data):
    if packet_data.get("cmd") == "shutdown":
        return "shutdown", None, None, None

    required = ["label", "pick", "load", "angle"]

    for key in required:
        if key not in packet_data:
            raise KeyError(f"packet_data에 '{key}' 키가 없습니다.")

    label = packet_data["label"]
    pick = tuple(map(float, packet_data["pick"]))
    load = tuple(map(float, packet_data["load"]))
    angle = float(packet_data["angle"])

    if len(pick) != 3:
        raise ValueError("pick은 [px, py, pz] 형식이어야 합니다.")

    if len(load) != 3:
        raise ValueError("load는 [lx, ly, lz] 형식이어야 합니다.")

    return label, pick, load, angle


def execute_sequence_1_2_3_4_vac_2_6_7_vacoff(arm, ik_solver, label, pick, load, angle):
    px, py, pz = pick
    lx, ly, lz = load

    px = px - 13.3
    py = py - 80
    pz = pz + 80

    print("\n[RECEIVER] start: 1-2-3-4-VAC_ON-2-6-7-VAC_OFF")

    print("\n[SEQ 1] PICK_READY")
    arm.pose_2_pick_ready()
    time.sleep(0.5)

    print("\n[SEQ 2] PICK pose")
    arm.pose_3_go_to_place()
    time.sleep(0.5)

    print("\n[SEQ 2-1] Base align to ArUco angle")
    angle_normalized = angle % 360
    if angle_normalized > 180:
        angle_normalized -= 360
    base_deg = angle_normalized
    current_deg23 = arm.last_deg_23 if arm.last_deg_23 is not None else 50.0
    current_deg45 = arm.last_deg_45 if arm.last_deg_45 is not None else 75.0
    current_deg6  = arm.last_deg_6  if arm.last_deg_6  is not None else 110.0
    arm.smooth_move_all_pose(base_deg, current_deg23, current_deg45, current_deg6, 0)
    time.sleep(0.5)

    print("\n[SEQ 3] ACT_PICK")
    pick_ik = ik_solver.calculate_pick_ik([px, py, pz])
    pick_ik["deg7"] = 10.0 + angle_normalized
    arm.act_pick(box_type=label, ik_angles=pick_ik)

    print("\n[VACUUM ON]")
    arm.vacuum_on()
    time.sleep(0.3)

    print("\n[SEQ box] box up")
    current_deg1  = arm.last_deg_1  if arm.last_deg_1  is not None else 0.0
    current_deg6  = arm.last_deg_6  if arm.last_deg_6  is not None else 110.0
    current_deg23 = arm.last_deg_23 if arm.last_deg_23 is not None else 64.0
    current_deg45 = arm.last_deg_45 if arm.last_deg_45 is not None else 74.0

    # box up: elbow 같이 펴기
    arm.smooth_move_all_pose(current_deg1, 35.0, 120.0, current_deg6, 0)
    time.sleep(0.3)
    # Base만 90도로 회전 (Shoulder/Elbow 유지)
    arm.smooth_move_all_pose(90.0, 35.0, 120.0, current_deg6, 0)
    time.sleep(0.5)

    print("\n[SEQ 7] ACT_PLACE")
    place_ik = ik_solver.calculate_place_ik([lx, ly, lz])

    print(f"[PLACE IK result]")
    print(f"  deg1  : {place_ik['deg1']:.1f}")
    print(f"  deg23 : {place_ik['deg23']:.1f}")
    print(f"  deg45 : {place_ik['deg45']:.1f}")
    print(f"  deg6  : {place_ik['deg6']:.1f}")
    print(f"  deg7  : {place_ik['deg7']:.1f}")

    # 1단계: Elbow 접고 Shoulder 세워서 뒤로 당김
    arm.smooth_move_all_pose(
        place_ik["deg1"],
        place_ik["deg23"],
        146.5,
        place_ik["deg6"],
        place_ik["deg7"],
    )
    time.sleep(0.8)
    # 2단계: Shoulder 뒤로 당긴 상태로 Elbow 내리기
    arm.smooth_move_all_pose(
        place_ik["deg1"],
        place_ik["deg23"] - 20.0,
        place_ik["deg45"],
        place_ik["deg6"],
        place_ik["deg7"],
    )
    time.sleep(0.8)
    # 3단계: Shoulder 목표 위치로
    arm.smooth_move_all_pose(
        place_ik["deg1"],
        place_ik["deg23"],
        place_ik["deg45"],
        place_ik["deg6"],
        place_ik["deg7"],
    )
    time.sleep(0.5)

    print("\n[VACUUM OFF]")
    arm.vacuum_off()
    time.sleep(0.3)

    current = arm.get_current_pose()
    current_deg1  = current["deg1"]  if current["deg1"]  is not None else 90.0
    current_deg23 = current["deg23"] if current["deg23"] is not None else 80.0
    current_deg6  = current["deg6"]  if current["deg6"]  is not None else 110.0
    current_deg7  = current["deg7"]  if current["deg7"]  is not None else 0.0

    print("\n[SEQ return 1] Elbow up")
    arm.smooth_move_all_pose(current_deg1, current_deg23, 120.0, current_deg6, current_deg7)
    time.sleep(0.5)

    print("\n[SEQ return 2] Shoulder up")
    arm.smooth_move_all_pose(current_deg1, 50.0, 120.0, current_deg6, current_deg7)
    time.sleep(0.5)

    print("\n[SEQ return 3] Wrist level + Base rotate")
    arm.smooth_move_all_pose(0.0, 50.0, 120.0, 110.0, current_deg7)
    time.sleep(0.5)

    print("\n[SEQ return 4] pose_3")
    arm.smooth_move_all_pose(0.0, 50.0, 75.0, 110.0, current_deg7)
    time.sleep(0.5)

    print("\n[SEQ return 5] PICK_READY")
    arm.return_to_pick_ready()

    print("\n[RECEIVER] sequence done")


def motor_control_receiver(q):
    ik_solver = RobotIKController("mybot.urdf")
    arm = RobotArm()

    if not arm.open_robot_port():
        print("[RECEIVER] port open failed")
        return

    arm.save_initial_positions()

    print("\n[RECEIVER] ready - waiting for queue...")

    try:
        while True:
            if not q.empty():
                packet_data = q.get()
                print(f"\n[RECEIVER] queue received: {packet_data}")

                try:
                    label, pick, load, angle = parse_packet(packet_data)
                except Exception as e:
                    print(f"[RECEIVER] packet error: {e}")
                    continue

                if label == "shutdown":
                    print("[RECEIVER] shutdown received")
                    break

                px, py, pz = pick
                lx, ly, lz = load

                print("\n" + "=" * 50)
                print(f"[RECEIVER] label: {label} | angle: {angle:.1f}")
                print(f"Pick: [{px:.1f}, {py:.1f}, {pz:.1f}] mm")
                print(f"Load: [{lx:.1f}, {ly:.1f}, {lz:.1f}] mm")
                print("=" * 50)

                execute_sequence_1_2_3_4_vac_2_6_7_vacoff(
                    arm=arm,
                    ik_solver=ik_solver,
                    label=label,
                    pick=pick,
                    load=load,
                    angle=angle,
                )

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[RECEIVER] interrupted")

    finally:
        try:
            arm.vacuum_off()
        except Exception:
            pass

        arm.close_robot()
        print("[RECEIVER] done")


if __name__ == "__main__":
    print("use as multiprocessing.Process target")
    print("ex: Process(target=motor_control_receiver, args=(q,)).start()")
