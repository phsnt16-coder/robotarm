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

    def calculate_pick_lut(self, pick_position_mm, *, shoulder_mode="raw"):
        from pick_ik_lookup import PickIKLookup
        if not hasattr(self, '_pick_lut'):
            self._pick_lut = PickIKLookup()
        result = self._pick_lut.solve(pick_position_mm, shoulder_mode=shoulder_mode)
        self.last_pick_result = result
        angles = dict(result['angles'])
        self.validate_output_range(angles)
        print('[IK] PICK LUT:', result['xyz_mm'], result['classification'])
        print('[IK] PICK LUT contributors:', result['contributors'])
        for warning in result['warnings']:
            print('[IK/주의]', warning)
        self.print_motor_angles(angles)
        return angles

    def calculate_pick_ik(self, pick_position_mm):
        """Actual Pick: measured LUT only; never fall back to analytic IK."""
        self.last_pick_valid = False
        self.last_pick_result = None
        angles = self.calculate_pick_lut(pick_position_mm)
        self.last_pick_valid = True
        return angles

    def calculate_place_ik(self, converted_place_position_mm, *,
                           allow_interpolation=False, allow_sparse=False,
                           allow_corrected=False, allow_suspect=False):
        """168점 Place LUT. 입력: 상자 중앙 [X,Y,Z] mm.

        Z=50mm(1F) / 100mm(2F) 실측 기반 조회; Z 사이 보간 없음.
        기본은 정확한 검증 대상 실측점만 허용. 계산 보간은 미리보기 전용.
        Place에 TCP +80mm 또는 박스 -55/-60mm를 재적용하지 마세요.
        """
        from place_ik_lookup import PlaceIKLookup
        if not hasattr(self, "_place_lut"):
            self._place_lut = PlaceIKLookup()
        result = self._place_lut.solve(
            converted_place_position_mm,
            require_measured=not allow_interpolation,
            allow_sparse=allow_sparse,
            allow_corrected=allow_corrected,
            allow_suspect=allow_suspect,
        )
        self.last_place_result = result
        print("\n[IK] PLACE 168점 (1F/2F) 실측 기반 조회")
        print("[IK] 입력 박스 중앙 (mm):", result.xyz_mm)
        print("[IK] 측정구역:", result.region, "| 분류:", result.classification)
        print("[IK] 참조:", result.contributors)
        for warning in result.warnings:
            print("[IK 주의]", warning)
        self.print_motor_angles(result.angles)
        return dict(result.angles)

    def operational_place_ik(self, top_xyz_mm):
        """Base XY and bin-floor suction height. No Z projection or TCP offset."""
        from place_ik_lookup import PlaceIKLookup
        requested = tuple(map(float,top_xyz_mm))
        try:
            if not hasattr(self, '_place_lut'):
                self._place_lut = PlaceIKLookup()
            result = self._place_lut.solve_operational(requested)
            angles = dict(result.angles)
            angles["deg7"] = max(0.0, angles["deg7"])
            self.validate_output_range(angles)
            self.last_place_result = result
            print('[IK] PLACE 윗면 LUT 목표:', requested)
            print('[IK] PLACE 분류:',result.classification)
            if 50 < requested[2] < 100:
                t=(requested[2]-50)/50
                print(f'[IK] Z 보간: 1F={1-t:.4f}, 2F={t:.4f}')
            self.print_motor_angles(angles)
            return {'status':'ok','angles':angles,'requested_xyz_mm':list(requested),
                'projected_xyz_mm':list(result.xyz_mm),'z_difference_mm':0.0,
                'classification':result.classification,'region':result.region,
                'contributors':[list(v) for v in result.contributors],
                'warnings':list(result.warnings),'nearest_xy_mm':result.nearest_distance_mm}
        except (ValueError,FileNotFoundError) as exc:
            return {'status':'blocked','error':str(exc),'requested_xyz_mm':list(requested)}

    def preview_place_ik(self, box_center_mm):
        """Preview-only angles for measured XY, even when requested Z is unmeasured.

        For unsupported Z, project the query to the nearest Z plane (50/100 mm)
        WITHOUT asserting that the returned angles reach the requested Z.
        No servo/motor side effects; current physical target is never mutated.
        """
        from place_ik_lookup import PlaceIKLookup, SUPPORTED_Z_MM
        if not hasattr(self, "_place_lut"):
            self._place_lut = PlaceIKLookup()
        lookup = self._place_lut
        requested = tuple(float(x) for x in box_center_mm)
        if len(requested) != 3 or not all(math.isfinite(x) for x in requested):
            raise ValueError('Place 상자 중심 좌표 [x,y,z]는 유한한 숫자 3개여야 합니다.')
        x, y, z = requested
        nearest_plane = min(SUPPORTED_Z_MM, key=lambda plane: (abs(z - plane), plane))
        z_difference = float(nearest_plane - z)
        z_projected = abs(z_difference) > 0.5
        projected = (x, y, float(nearest_plane)) if z_projected else requested
        neighbors = lookup.nearest_xy_by_plane(requested, limit=4)
        try:
            # Existing LUT: identical-Z 6-point IDW with convex-hull protection.
            result = lookup.solve(projected, require_measured=False, allow_sparse=False)
            angles = dict(result.angles)
            self.validate_output_range(angles)
            warnings = list(result.warnings)
            if z_projected:
                warnings.insert(0,
                    f'실제 목표 Z={z:.2f}mm는 미측정. Z={nearest_plane:.0f}mm의 '
                    '가상 평면에서만 계산한 참고 관절각이며 실제 목표 Z에 도달하지 않습니다.')
            self.last_place_result = result
            print('[IK] PLACE 원래 상자 중심:', requested)
            print('[IK] PLACE LUT 조회 좌표:', projected)
            print('[IK] PLACE 분류:', 'z_projected_preview' if z_projected else result.classification)
            print('[IK] PLACE 모터 미전송 관절각:')
            self.print_motor_angles(angles)
            for warning in warnings:
                print('[IK/주의]', warning)
            return {
                'status': 'approximate_z_projection' if z_projected else 'ok',
                'angles': angles,
                'requested_xyz_mm': list(requested),
                'projected_xyz_mm': list(projected),
                'z_difference_mm': round(z_difference, 3),
                'classification': result.classification,
                'region': result.region,
                'contributors': [list(v) for v in result.contributors],
                'warnings': warnings,
                'nearest_xy_mm': round(result.nearest_distance_mm, 3),
                'neighbors_by_plane': neighbors,
                'motor_approved': False,
            }
        except (ValueError, FileNotFoundError) as exc:
            return {
                'status': 'blocked', 'error': str(exc),
                'requested_xyz_mm': list(requested),
                'projected_xyz_mm': list(projected),
                'z_difference_mm': round(z_difference, 3),
                'neighbors_by_plane': neighbors, 'motor_approved': False,
            }

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
    pick = ik.calculate_pick_ik([350.0, 55.0, 110.0])
    print("실측: deg23=44.95  deg45=53.98")

    print("\n=== Place IK 테스트 ===")
    # CSV의 정확한 실측 중앙좌표를 사용한 1F 예시
    place = ik.calculate_place_ik([55.0, -235.0, 50.0])

    print("Pick/Place LUT 관절각은 기존 평면 FK 공식으로 검증하지 않습니다.")

