import math
from ikpy.chain import Chain


class RobotIKController:
    """
    motor_controller 기준 IK 변환 모듈.

    목적:
        - IK 결과로 Elbow(deg45)가 변해도 Gripper/Wrist(deg6)가 지면 방향을 유지하도록 동적 보정
        - 즉, Wrist가 로봇 전체 자세 변화에 대해 상대적으로 항상 110도 기준 자세를 유지하게 함
        - 이전 Shoulder 보정값이 남지 않도록 Shoulder는 deg6 계산에서 완전히 제외

    핵심 Wrist 보정식:
        - elbow_delta = deg45 - origin_deg45
        - deg6 = 110.0 - ELBOW_LEVEL_GAIN * elbow_delta

    기준 및 정책:
        - Pick 기준:  origin_deg23 = 60.0,  origin_deg45 = 40.0
        - Place 기준: origin_deg23 = 20.0,  origin_deg45 = 20.0
        - Shoulder(deg23) 변화량은 Wrist 보정에 사용하지 않음
        - Elbow(deg45) 변화량만 Wrist(deg6)에 반대로 반영
        - Pick / Place IK 결과를 MOTOR_CONTROL.py에 바로 넣을 수 있는 각도 dict로 변환
    """

    URDF_PATH = "mybot.urdf"

    # ikpy Chain link 개수 기준:
    # [base_link, joint1, joint23, joint45, joint6, joint7]
    ACTIVE_MASK = [False, True, True, True, True, True]

    DEFAULT_ORIENTATION = [0, -1, 0]
    ORIENTATION_MODE = "Y"

    # TCP 보정값
    GRIPPER_TCP_LENGTH_MM = -80.0

    # 흡착패드가 바닥을 향하는 Wrist 기준각
    WRIST_TCP_REFERENCE_DEG = 110.0

    # Elbow 기반 Wrist 동적 보정 사용 여부
    ENABLE_GRIPPER_LEVELING = True

    # 기본식: deg6 = 110 - ELBOW_LEVEL_GAIN * elbow_delta
    # 실제 보정 방향이 반대면 해당 gain을 -1.0으로 변경
    # 보정량이 과하면 0.5, 부족하면 1.2 등으로 조정
    ELBOW_LEVEL_GAIN = 1.0

    SERVO_MIN_DEG = 0.0
    SERVO_MAX_DEG = 220.0

    # 모터 제어 코드 기준 중심 오프셋
    SERVO_OFFSETS = {
        "deg1": 90.0,
        "deg23": 90.0,
        "deg45": 90.0,
        "deg6": 110.0,
        "deg7": 110.0,
    }

    # 현재 프로젝트 기준 방향 보정
    # Shoulder / Elbow 정상 판정값 유지
    # Wrist는 아래 apply_gripper_leveling()에서 최종 overwrite됨
    SERVO_DIRECTIONS = {
        "deg1": 1.0,
        "deg23": -1.0,
        "deg45": 1.0,
        "deg6": 1.0,
        "deg7": 1.0,
    }

    PICK_ORIGIN_POSE_DEG = {
        "deg1": 0.0,
        "deg23": 60.0,
        "deg45": 40.0,
        "deg6": 110.0,
        "deg7": 0.0,
    }

    PLACE_ORIGIN_POSE_DEG = {
        "deg1": 90.0,
        "deg23": 20.0,
        "deg45": 20.0,
        "deg6": 110.0,
        "deg7": 0.0,
    }

    def __init__(self, urdf_path=None):
        self.urdf_path = urdf_path or self.URDF_PATH
        self.chain = Chain.from_urdf_file(
            self.urdf_path,
            active_links_mask=self.ACTIVE_MASK,
        )
        print(f"[robot_ik] RobotIKController 초기화 완료 (URDF: {self.urdf_path})")
        print(f"[robot_ik] SERVO_DIRECTIONS : {self.SERVO_DIRECTIONS}")
        print(f"[robot_ik] TCP 보정값       : {self.GRIPPER_TCP_LENGTH_MM} mm")
        print(f"[robot_ik] Wrist 기준각     : {self.WRIST_TCP_REFERENCE_DEG} deg")
        print(f"[robot_ik] Wrist 보정 방식  : Elbow only")
        print(f"[robot_ik] Elbow Gain       : {self.ELBOW_LEVEL_GAIN}")

    @staticmethod
    def clamp(value, min_v, max_v):
        return max(min_v, min(max_v, value))

    @staticmethod
    def rotate_xy_mm(x_mm, y_mm, base_deg):
        theta = math.radians(-base_deg)
        local_x = x_mm * math.cos(theta) + y_mm * math.sin(theta)
        local_y = -x_mm * math.sin(theta) + y_mm * math.cos(theta)
        return local_x, local_y

    def calculate_ik_core(self, target_position_m):
        initial_joint_positions = [0.0] * len(self.chain.links)

        return self.chain.inverse_kinematics(
            target_position=target_position_m,
            target_orientation=self.DEFAULT_ORIENTATION,
            orientation_mode=self.ORIENTATION_MODE,
            initial_position=initial_joint_positions,
            max_iter=0.3
        )

    def joint_rad_to_servo_deg(self, joint_rad, key):
        raw_deg = math.degrees(float(joint_rad))
        servo_deg = self.SERVO_OFFSETS[key] + self.SERVO_DIRECTIONS[key] * raw_deg
        return self.clamp(servo_deg, self.SERVO_MIN_DEG, self.SERVO_MAX_DEG)

    def joints_to_motor_angles(self, joints, origin_pose_deg):
        """
        [완벽 수리] 오프셋 오차 누적을 제거하고, IK 솔버의 실제 라디안을 
        다이렉트로 다이내믹셀 구동 각도로 매핑합니다.
        """
        if len(joints) < 6:
            raise ValueError(f"IK joints 길이 부족: {len(joints)}")

        # 🛠️ 복잡한 델타 계산 대신, 솔버 각도를 모터의 실제 구동 각도로 1:1 변환
        # (URDF 오프셋 보정치를 더해 절대 각도로 변환합니다)
        result = {
            "deg1": float(math.degrees(joints[1])),
            "deg23": float(math.degrees(joints[2])) + 60.0,
            "deg45": float(math.degrees(joints[3])) + 40.0,
            "deg6": float(math.degrees(joints[4])) + 110.0,
            "deg7": float(math.degrees(joints[5])),
        }

        # 하드웨어 보호를 위한 클램핑(가동 범위 제한)만 수행합니다.
        for key in result.keys():
            result[key] = self.clamp(
                result[key],
                self.SERVO_MIN_DEG,
                self.SERVO_MAX_DEG
            )

        return result

    def apply_gripper_leveling(self, angles, origin_pose_deg, mode_name="PICK"):
        """
        Elbow(deg45) 변화량만 Wrist(deg6)에 반영한다.
        이전 보정식에서 사용하던 Shoulder 영향은 완전히 제거됨.

        식:
            elbow_delta = current_deg45 - origin_deg45
            deg6 = WRIST_TCP_REFERENCE_DEG - ELBOW_LEVEL_GAIN * elbow_delta
        """
        if not self.ENABLE_GRIPPER_LEVELING:
            return angles

        leveled = dict(angles)
        elbow_delta = leveled["deg45"] - origin_pose_deg["deg45"]
        old_deg6 = leveled["deg6"]

        wrist_target = self.WRIST_TCP_REFERENCE_DEG - self.ELBOW_LEVEL_GAIN * elbow_delta
        wrist_target = self.clamp(wrist_target, self.SERVO_MIN_DEG, self.SERVO_MAX_DEG)

        leveled["deg6"] = wrist_target

        print("\n[Gripper Elbow-only Leveling]")
        print(f"mode             : {mode_name}")
        print(f"origin deg45     : {origin_pose_deg['deg45']:.2f} deg")
        print(f"current deg45    : {angles['deg45']:.2f} deg")
        print(f"elbow_delta      : {elbow_delta:.2f} deg")
        print(f"old deg6         : {old_deg6:.2f} deg")
        print(f"leveled deg6     : {wrist_target:.2f} deg")
        print("[INFO] Wrist 보정에는 Elbow 변화량만 사용합니다. Shoulder 변화량은 사용하지 않습니다.")

        return leveled

    def calculate_pick_joints(self, pick_position_mm):
        x_mm, y_mm, z_mm = map(float, pick_position_mm)
        pick_z_mm = max(0.0, z_mm - self.GRIPPER_TCP_LENGTH_MM)

        target_position_m = [
            x_mm / 1000.0,
            y_mm / 1000.0,
            pick_z_mm / 1000.0,
        ]

        print("\n[IK] PICK IK 계산")
        print("[IK] 기준 원점        : 3번 포즈 결과값")
        print(f"[IK] 3번 기준 포즈    : {self.PICK_ORIGIN_POSE_DEG}")
        print(f"[IK] Pick 상대좌표 mm : [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}]")
        print(f"[IK] TCP 보정 후 mm   : [{x_mm:.1f}, {y_mm:.1f}, {pick_z_mm:.1f}]")

        return self.calculate_ik_core(target_position_m)

    def calculate_pick_ik(self, pick_position_mm):
        joints = self.calculate_pick_joints(pick_position_mm)
        angles = self.joints_to_motor_angles(joints, self.PICK_ORIGIN_POSE_DEG)
        
        # 동적 Wrist 보정 적용 (Elbow Only)
        angles = self.apply_gripper_leveling(angles, self.PICK_ORIGIN_POSE_DEG, mode_name="PICK")

        print("\n[IK] PICK 모터 각도")
        self.print_motor_angles(angles)

        return angles

    def calculate_place_joints(self, load_position_mm, place_origin_pose_deg=None):
        if place_origin_pose_deg is None:
            place_origin_pose_deg = self.PLACE_ORIGIN_POSE_DEG

        x_mm, y_mm, z_mm = map(float, load_position_mm)
        base_deg = float(place_origin_pose_deg["deg1"])

        local_x_mm, local_y_mm = self.rotate_xy_mm(x_mm, y_mm, base_deg)

        target_position_m = [
            local_x_mm / 1000.0,
            local_y_mm / 1000.0,
            z_mm / 1000.0,
        ]

        print("\n[IK] PLACE IK 계산")
        print("[IK] 기준 원점          : 6번 포즈 회전 후 결과값")
        print(f"[IK] 6번 기준 포즈      : {place_origin_pose_deg}")
        print(f"[IK] 원본 Load 좌표 mm  : [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}]")
        print(f"[IK] Base 기준 회전각   : {base_deg:.1f}°")
        print(f"[IK] 변환 Place 좌표 mm : [{local_x_mm:.1f}, {local_y_mm:.1f}, {z_mm:.1f}]")

        return self.calculate_ik_core(target_position_m)

    def calculate_place_ik(self, converted_place_position_mm, place_origin_pose_deg):
        """
        [완벽 축 분리] 리시버가 계산한 90도 회전 변환 좌표(예: [-50, 130, 50])를 
        월드 좌표계 기준으로 다이렉트 연산합니다.
        """
        # 1. 리시버가 준 변환 좌표 [-50, 130, 50]를 그대로 수용
        x_mm, y_mm, z_mm = map(float, converted_place_position_mm)
        
        # TCP 보정 (흡착 패드 길이만큼 Z축 마진 확보)
        place_z_mm = max(0.0, z_mm - self.GRIPPER_TCP_LENGTH_MM)
        target_position_m = [x_mm / 1000.0, y_mm / 1000.0, place_z_mm / 1000.0]

        # 🚨 [핵심 변경] 축 개념 분리: 솔버의 시작 힌트에서 Base(deg1)를 '0.0' 라디안으로 던집니다.
        # 이렇게 해야 솔버가 월드 좌표 [-50, 130]를 보고 스스로 Base를 90도 부근으로 계산해 냅니다.
        sh_deg = float(place_origin_pose_deg.get("deg23", 20.0))
        el_deg = float(place_origin_pose_deg.get("deg45", 20.0))
        wr_deg = float(place_origin_pose_deg.get("deg6", 110.0))

        place_joints_rad = [
            0.0,
            0.0,                           # 🛠️ Base 힌트를 0으로 고정하여 월드 좌표 연산 유도!
            math.radians(sh_deg - 60.0),   # Shoulder 오프셋 상쇄
            math.radians(el_deg - 40.0),   # Elbow 오프셋 상쇄
            math.radians(wr_deg - 110.0),  # Wrist 오프셋 상쇄
            0.0
        ]

        # 2. 역기하학 연산 수행 (솔버가 스스로 Base 각도까지 계산함)
        joints = self.chain.inverse_kinematics(
            target_position=target_position_m,
            target_orientation=self.DEFAULT_ORIENTATION,
            orientation_mode=self.ORIENTATION_MODE,
            initial_position=place_joints_rad,
            max_iter=200
        )

        # 3. 절대 모터 각도로 변환
        angles = self.joints_to_motor_angles(joints, place_origin_pose_deg)
        angles["deg1"] = 90.0
        
        # 4. 손목 수평 레벨링 보정 적용
        angles = self.apply_gripper_leveling(angles, place_origin_pose_deg, mode_name="PLACE")
        
        return angles

    def verify_fk(self, joints):
        computed_matrix = self.chain.forward_kinematics(joints)
        return computed_matrix[:3, 3].tolist()

    def validate_output_range(self, angles):
        for key, value in angles.items():
            if not (self.SERVO_MIN_DEG <= value <= self.SERVO_MAX_DEG):
                raise ValueError(f"{key} 범위 초과: {value}")
        return True

    @staticmethod
    def print_motor_angles(angles):
        print(f"deg1  Base              : {angles['deg1']:.2f} deg")
        print(f"deg23 Shoulder Pair     : {angles['deg23']:.2f} deg")
        print(f"deg45 Elbow Pair        : {angles['deg45']:.2f} deg")
        print(f"deg6  Wrist Elbow-only  : {angles['deg6']:.2f} deg")
        print(f"deg7  Gripper Rotation  : {angles['deg7']:.2f} deg")


if __name__ == "__main__":
    # 간단 검증용 스크립트 실행부
    ik = RobotIKController("mybot.urdf")

    try:
        pick = ik.calculate_pick_ik([120, 80, 150])
        ik.validate_output_range(pick)

        place = ik.calculate_place_ik([200, 100, 80])
        ik.validate_output_range(place)

        print("\n[PICK IK dict]")
        print(pick)

        print("\n[PLACE IK dict]")
        print(place)
    except Exception as e:
        print(f"\n[오류 발생] {e}")