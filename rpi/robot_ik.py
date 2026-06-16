import math
from scipy.optimize import minimize


class RobotIKController:
    """
    수동 FK + scipy minimize 기반 IK 모듈.

    FK 공식 (지면 기준 각도):
        a_sh = 90 - sh          (sh: 수직에서 아래로 구부러진 각도)
        a_el = 90 - sh - el     (el: shoulder 기준 상대각)
        a_wr = 90 - sh - el - wr(wr: elbow 기준 상대각)

        Y = L_sh*cos(a_sh) + L_el*cos(a_el) + L_wr*cos(a_wr)
        Z = L_base + L_sh*sin(a_sh) + L_el*sin(a_el) + L_wr*sin(a_wr)

    모터 변환:
        sh  = mot23 - OFFSET_23
        el  = OFFSET_45 - mot45  (감소=구부러짐)
        wr  = OFFSET_6  - mot6
        mot23 = sh  + OFFSET_23
        mot45 = OFFSET_45 - el
        mot6  = OFFSET_6  - wr
    """

    URDF_PATH = "mybot.urdf"

    L_BASE = 135.0
    L_SH   = 120.0
    L_EL   = 160.0
    L_WR   = 65.0

    GRIPPER_TCP_LENGTH_MM = 80.0

    JOINT_OFFSETS = {
        "deg1":  0.0,
        "deg23": 56.1,
        "deg45": 146.5,
        "deg6":  116.8,
        "deg7":  0.0,
    }

    SERVO_MIN_DEG = 0.0
    SERVO_MAX_DEG = 220.0

    PICK_ORIGIN_POSE_DEG = {
        "deg1":  0.0,
        "deg23": 50.0,
        "deg45": 75.0,
        "deg6":  110.0,
        "deg7":  0.0,
    }

    PLACE_ORIGIN_POSE_DEG = {
        "deg1":  90.0,
        "deg23": 80.0,
        "deg45": 50.0,
        "deg6":  110.0,
        "deg7":  0.0,
    }

    def __init__(self, urdf_path=None):
        self.urdf_path = urdf_path or self.URDF_PATH
        print(f"[robot_ik] RobotIKController 초기화 완료")
        print(f"[robot_ik] JOINT_OFFSETS : {self.JOINT_OFFSETS}")
        print(f"[robot_ik] 링크 길이     : base={self.L_BASE} sh={self.L_SH} el={self.L_EL} wr={self.L_WR}mm")
        print(f"[robot_ik] TCP 보정값    : {self.GRIPPER_TCP_LENGTH_MM}mm")

    @staticmethod
    def clamp(value, min_v, max_v):
        return max(min_v, min(max_v, value))

    @staticmethod
    def rotate_xy_mm(x_mm, y_mm, base_deg):
        theta = math.radians(-base_deg)
        local_x = x_mm * math.cos(theta) + y_mm * math.sin(theta)
        local_y = -x_mm * math.sin(theta) + y_mm * math.cos(theta)
        return local_x, local_y

    def forward_kinematics(self, sh_deg, el_deg, wr_deg):
        a_sh = math.radians(90 - sh_deg)
        a_el = math.radians(90 - sh_deg - el_deg)
        a_wr = math.radians(90 - sh_deg - el_deg - wr_deg)
        Y = (self.L_SH * math.cos(a_sh) +
             self.L_EL * math.cos(a_el) +
             self.L_WR * math.cos(a_wr))
        Z = (self.L_BASE +
             self.L_SH * math.sin(a_sh) +
             self.L_EL * math.sin(a_el) +
             self.L_WR * math.sin(a_wr))
        return Y, Z

    def solve_ik(self, target_y_mm, target_z_mm, hint_sh=30.0, hint_el=40.0):
        # wr = 90-sh-el (그리퍼 수직 유지)
        def error(params):
            sh, el = params
            wr = 90.0 - sh - el
            Y, Z = self.forward_kinematics(sh, el, wr)
            return (Y - target_y_mm)**2 + (Z - target_z_mm)**2

        result = minimize(
            error,
            x0=[hint_sh, hint_el],
            method='Nelder-Mead',
            options={'maxiter': 10000, 'xatol': 1e-6, 'fatol': 1e-6}
        )
        sh, el = result.x
        # mot23 = sh + OFFSET_23 - 20 → sh_real = sh + 20
        wr = 90.0 - sh - el
        err = math.sqrt(result.fun)
        return sh, el, wr, err

    def bend_to_motor(self, sh, el, wr):
        mot23 = sh  + self.JOINT_OFFSETS["deg23"] - 20.0  # MOTOR_CONTROL OFFSET_DEG_23 보정
        mot45 = self.JOINT_OFFSETS["deg45"] - el
        mot6  = self.JOINT_OFFSETS["deg6"] + wr  # sh+el<90→증가, sh+el>90→감소
        return mot23, mot45, mot6

    def motor_to_bend(self, mot23, mot45, mot6=None):
        sh = mot23 - self.JOINT_OFFSETS["deg23"]
        el = self.JOINT_OFFSETS["deg45"] - mot45
        wr = (self.JOINT_OFFSETS["deg6"] - mot6) if mot6 is not None else None
        return sh, el, wr

    def calculate_pick_ik(self, pick_position_mm):
        x_mm, y_mm, z_mm = map(float, pick_position_mm)
        pick_z_mm = float(z_mm) + self.GRIPPER_TCP_LENGTH_MM
        target_y = math.sqrt(x_mm**2 + y_mm**2)

        print("\n[IK] PICK IK 계산")
        print(f"[IK] Pick 상대좌표 mm : [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}]")
        print(f"[IK] TCP 보정 없음    : [{x_mm:.1f}, {y_mm:.1f}, {pick_z_mm:.1f}]")

        sh, el, wr, err = self.solve_ik(
            target_y, pick_z_mm,
            hint_sh=45.0, hint_el=30.0
        )
        mot23, mot45, mot6 = self.bend_to_motor(sh, el, wr)
        mot1 = math.degrees(math.atan2(x_mm, y_mm))

        angles = {
            "deg1":  mot1,
            "deg23": self.clamp(mot23, self.SERVO_MIN_DEG, self.SERVO_MAX_DEG),
            "deg45": self.clamp(mot45, self.SERVO_MIN_DEG, self.SERVO_MAX_DEG),
            "deg6":  self.clamp(mot6,  self.SERVO_MIN_DEG, self.SERVO_MAX_DEG),
            "deg7":  0.0,
        }

        print(f"\n[IK] PICK 모터 각도 (FK오차={err:.1f}mm)")
        self.print_motor_angles(angles)
        return angles

    def calculate_place_ik(self, converted_place_position_mm):
        x_mm, y_mm, z_mm = map(float, converted_place_position_mm)
        place_z_mm = float(z_mm) + self.GRIPPER_TCP_LENGTH_MM  # TCP 보정
        target_y = math.sqrt(x_mm**2 + y_mm**2)

        print(f"\n[IK] PLACE IK 계산")
        print(f"[IK] Place 좌표 mm    : [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}]")
        print(f"[IK] TCP 보정 없음    : [{x_mm:.1f}, {y_mm:.1f}, {place_z_mm:.1f}]")

        sh, el, wr, err = self.solve_ik(
            target_y, place_z_mm,
            hint_sh=30.0, hint_el=40.0
        )
        mot23, mot45, mot6 = self.bend_to_motor(sh, el, wr)

        angles = {
            "deg1":  90.0,
            "deg23": self.clamp(mot23, self.SERVO_MIN_DEG, self.SERVO_MAX_DEG),
            "deg45": self.clamp(mot45, self.SERVO_MIN_DEG, self.SERVO_MAX_DEG),
            "deg6":  self.clamp(mot6,  self.SERVO_MIN_DEG, self.SERVO_MAX_DEG),
            "deg7":  0.0,
        }

        print(f"\n[IK] PLACE 모터 각도 (FK오차={err:.1f}mm)")
        self.print_motor_angles(angles)
        return angles

    def verify_fk(self, mot23, mot45, mot6):
        sh, el, wr = self.motor_to_bend(mot23, mot45, mot6)
        Y, Z = self.forward_kinematics(sh, el, wr)
        return Y, Z

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
        print(f"deg6  Wrist             : {angles['deg6']:.2f} deg")
        print(f"deg7  Gripper Rotation  : {angles['deg7']:.2f} deg")


if __name__ == "__main__":
    ik = RobotIKController()

    print("\n=== Pick IK 테스트 ===")
    pick = ik.calculate_pick_ik([-13.7, 264.2, 50.0])
    print(f"실측: deg23=60.4  deg45=64.1")

    print("\n=== Place IK 테스트 ===")
    place = ik.calculate_place_ik([30.0, 270.0, 50.0])

    print("\n=== FK 검증 ===")
    for name, a in [("Pick", pick), ("Place", place)]:
        Y, Z = ik.verify_fk(a['deg23'], a['deg45'], a['deg6'])
        print(f"{name}: Y={Y:.1f} Z={Z:.1f}mm")
