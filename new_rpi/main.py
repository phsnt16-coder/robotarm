#!/usr/bin/env python3
"""Operational BPP pick/place. Place LUT uses Base XY and bin-floor-referenced suction height."""
from __future__ import annotations
import math
from multiprocessing import Process, Queue, Event
from queue import Empty, Full

ROI_CENTER_BASE_MM = (245.0, 0.0)  # XY of ROI center in robot Base frame
CONVEYOR_Z_BASE_MM = 80.0           # conveyor top in robot Base frame
# IMPORTANT: image axes relative to robot axes NOT independently calibrated.
# Matrix maps [image right, image down] -> [robot X, robot Y].
# Identity is provisional and is NOT va
#lidated for motor operation.
IMAGE_TO_ROBOT_XY = ((1.0, 0.0), (0.0, 1.0))
XY_TRANSFORM_CALIBRATED = True  # previous user verification; recheck after camera relocation

# BPP local origin at bin corner; local +X/+Y/+Z toward robot -X/-Y/-Z.
BIN_ORIGIN_BASE_MM = (110.0, -175.0, 0.0)
BIN_AXIS_SIGNS = (-1.0, -1.0, -1.0)
BIN_SIZE_MM = (300.0, 200.0, 200.0)  # width/depth/height
# Physical conflict: desired fixed Y/Z positive vs BPP base-frame Y/Z negative.
# Do not equate these target types without verifying actual setup.
FIXED_PLACE_TARGETS = {'A': (60.0, 285.0, 50.0),
                       'B': (-60.0, 295.0, 50.0)}
DEFAULT_BOX_WEIGHT = 1.0


def pick_roi_to_base(dx_mm: float, dy_mm: float, h_mm: float):
    """Convert ROI-centered offsets and measured height to nominal robot TCP."""
    a, b = IMAGE_TO_ROBOT_XY[0]
    c, d = IMAGE_TO_ROBOT_XY[1]
    x0, y0 = ROI_CENTER_BASE_MM
    return [x0 + a * dx_mm + b * dy_mm,
            y0 + c * dx_mm + d * dy_mm,
            CONVEYOR_Z_BASE_MM + h_mm]


def bpp_item_to_base(item):
    """BPP lower-corner -> box upper-face center in Base coordinates.

    WARNING: BPP local +Z corresponds to Base -Z by user-specified convention.
    This must be reconciled with real container floor/top before execution.
    """
    ox, oy, oz = BIN_ORIGIN_BASE_MM
    sx, sy, sz = BIN_AXIS_SIGNS
    cx = float(item.x) + float(item.w) / 2.0
    cy = float(item.y) + float(item.d) / 2.0
    cz = float(item.z) + float(item.h)
    return [ox + sx * cx, oy + sy * cy, oz + sz * cz]


def bpp_item_top_to_lut(item):
    """Base XY at upper-face center; Z above bin floor (positive upward)."""
    ox, oy, oz = BIN_ORIGIN_BASE_MM
    sx, sy, sz = BIN_AXIS_SIGNS
    return [ox + sx * (float(item.x) + float(item.w) / 2),
            oy + sy * (float(item.y) + float(item.d) / 2),
            float(item.z) + float(item.h)]


def validate_outline(result):
    if not isinstance(result, (tuple, list)) or len(result) != 4:
        raise ValueError('Outline 반환 형식은 (angle, [dx,dy,h], [w,d], h)여야 합니다.')
    angle, pick, size, height = result
    if len(pick) != 3 or len(size) != 2:
        raise ValueError('Outline 좌표/크기의 차원이 잘못되었습니다.')
    values = [float(angle), *(float(v) for v in pick),
              *(float(v) for v in size), float(height)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Outline 결과에 유효하지 않은 숫자가 포함되어 있습니다.')
    angle, dx, dy, raw_h, width, depth, height = values
    if width <= 0 or depth <= 0 or height <= 0:
        raise ValueError('너비·깊이·ToF 높이가 양수여야 합니다.')
    if abs(raw_h - height) > 2.0:
        raise ValueError('Outline pick Z와 높이 측정값이 일치하지 않습니다.')
    return angle, dx, dy, width, depth, height


def choose_mode():
    print('\n=== 적재 모드 ===\n1: BPP 자동 배치\n2: FIXED A/B 목표 좌표 확인')
    while True:
        selection = input('선택 [1/2, Enter=1]: ').strip()
        if selection in ('', '1'):
            return 'bpp'
        if selection == '2':
            return 'fixed'
        print('1 또는 2를 입력하세요.')


def run():
    live = True
    if not XY_TRANSFORM_CALIBRATED:
        raise RuntimeError('카메라-로봇 좌표 변환 미확정')
    # Delay Raspberry Pi dependencies until after geometry helpers can be tested.
    from picamera2 import Picamera2
    import outline
    from outline import live_outline_detection
    from py3Dbp import Item, LiveBin
    from visualizer import BinVisualizer
    from receiver import motor_control_receiver

    # Verify exactly which outline module Python imports from the project.
    print(f"[MAIN] outline 모듈 경로: {outline.__file__}")

    # Apply firmware-path correction before live_outline_detection initializes ToF.
    # This also protects runs using an older outline.py without the fix.
    import sys
    from pathlib import Path
    import vl53lxcx

    if not getattr(vl53lxcx._find_file, "_robot_fw_fixed", False):
        _original_find_file = vl53lxcx._find_file
        _firmware_dir = Path(vl53lxcx.__file__).resolve().parent

        def _robot_find_file(name, req_size):
            requested = Path(name)
            candidates = [requested] if requested.is_absolute() else []
            candidates.append(_firmware_dir / requested.name)
            if not requested.is_absolute():
                candidates.extend(Path(p) / requested for p in sys.path if p)
            for path in candidates:
                if path.is_file():
                    size = path.stat().st_size
                    if size != req_size:
                        raise ValueError(
                            f"Firmware size mismatch: {path} ({size} != {req_size})"
                        )
                    print(f"[MAIN/TOF FW] 펌웨어 경로: {path.resolve()}")
                    return str(path)
            return _original_find_file(name, req_size)

        _robot_find_file._robot_fw_fixed = True
        vl53lxcx._find_file = _robot_find_file
    print("[MAIN] ToF 펌웨어 경로 보정 적용 완료")

    mode = 'bpp'
    print('[MAIN] 좌표 원점: 로봇 Base; ROI 중심=', ROI_CENTER_BASE_MM,
          ', 컨베이어 Z=', CONVEYOR_Z_BASE_MM)
    if not XY_TRANSFORM_CALIBRATED:
        print('[참고] XY_TRANSFORM_CALIBRATED 플래그는 False이며, 사용자가 로봇 Base 좌표 일치를 확인했다고 전달했습니다.')
    print('[MAIN] Place LUT: Base XY, 적재함 바닥 기준 윗면 Z (+)')
    print("[MAIN] 실제 자동 파지/적재 모드")

    camera = None
    worker = None
    q = None
    result_q = None
    bin_system = LiveBin(*BIN_SIZE_MM, max_weight=100)
    box_count = 0
    motion_in_progress = False
    stop_event = Event()
    try:
        camera = Picamera2()
        config = camera.create_preview_configuration(
            main={'size': (3280, 2464), 'format': 'BGR888'}, buffer_count=4)
        camera.configure(config)
        camera.start()
        print('[MAIN] 카메라 시작')

        q = Queue(maxsize=5)
        result_q = Queue(maxsize=5)
        worker = Process(target=motor_control_receiver, args=(q, result_q, stop_event), daemon=False)
        worker.start()
        print("[MAIN] Receiver -> RobotIK -> MOTOR_CONTROL 포즈 연동")

        while True:
            cmd = input('\nOutline 인식 [Enter/y=시작, q=종료]: ').strip().lower()
            if cmd in ('q', 'quit', 'esc'):
                break
            if cmd not in ('', 'y'):
                continue
            if mode == 'fixed' and box_count >= len(FIXED_PLACE_TARGETS):
                print('[MAIN] FIXED A/B 좌표를 모두 사용했습니다.')
                continue

            # Inside Outline window, press 'a' to accept or 'q' to cancel.
            result = live_outline_detection(camera)
            if result is None:
                print('[MAIN] 검출 취소/실패; 큐 전송하지 않음')
                continue
            try:
                angle, dx, dy, width, depth, box_h = validate_outline(result)
            except (ValueError, TypeError) as exc:
                print('[MAIN] 센서/검출값 오류:', exc)
                continue

            pick = pick_roi_to_base(dx, dy, box_h)
            label = ('A' if box_count == 0 else 'B') if mode == 'fixed' else f'BOX_{box_count+1}'
            item = Item(label, width, box_h, depth, DEFAULT_BOX_WEIGHT)
            success, rotated = bin_system.place_item(item)
            if not success:
                print('[MAIN] BPP 배치 실패: 적재 패킷 생성하지 않음')
                continue
            bpp_load = bpp_item_to_base(item)
            load = list(FIXED_PLACE_TARGETS[label]) if mode == 'fixed' else bpp_load
            place_ik_input = load if mode == 'fixed' else bpp_item_top_to_lut(item)

            payload = {
                'packet_id': box_count + 1,
                'label': label,
                'pick': [round(v, 2) for v in pick],
                'load': [round(v, 2) for v in load],
                'angle': round(angle, 2),
                'is_rotated': bool(rotated),
                'success': True,
                'box_size': {'w': round(width, 2), 'd': round(depth, 2), 'h': round(box_h, 2)},
                'load_reference': 'box_center' if mode == 'fixed' else 'upper_face_center',
                'place_ik_input': [round(v, 2) for v in place_ik_input],
                'place_ik_reference': 'upper_face_bin_z',
            }
            print('\n[MAIN] ROI offset:', (dx, dy), 'mm; ToF height:', box_h, 'mm')
            print('[MAIN] Pick(Base):', pick)
            print('[MAIN] BPP corner:', (item.x, item.y, item.z))
            print('[MAIN] BPP target(Base):', bpp_load)
            print('[MAIN] Final load(Base):', load, '(mode:', mode, ')')
            print('[MAIN] Place IK 입력(윗면 XY Base / Z 적재함 바닥):', place_ik_input)
            if mode == 'fixed':
                print('[주의] FIXED 좌표는 BPP 결과와 별개이며 실제 적재를 검증하지 않습니다.')
            try:
                BinVisualizer(bin_system).update_plot()
            except Exception as exc:
                print('[MAIN] 시각화 오류:', exc)
            try:
                q.put(payload, timeout=2)
            except Full:
                print('[MAIN] Receiver Queue 가득 참: 전송 실패')
                # BPP already committed the item; stop to avoid inconsistent states.
                print('[MAIN] BPP 상태와 큐 불일치. 안전하게 종료합니다.')
                break
            print('[MAIN] Queue 전송 성공 -> Receiver IK 결과 대기')
            try:
                answer = result_q.get(timeout=20)
            except Empty:
                print('[MAIN] Receiver IK 응답 시간 초과. 안전하게 중단합니다.')
                break
            if answer.get('packet_id') != payload['packet_id']:
                print('[MAIN] IK 응답 ID 불일치. 안전하게 중단합니다:', answer)
                break
            print('[MAIN] IK 상태:', answer.get('status'))
            for name in ('pick_ik','place_ik'):
                angles = answer.get(name)
                if angles:
                    print(f'[MAIN] {name} 관절각:')
                    for joint, value in angles.items():
                        print(f'    {joint}: {value:.2f} deg')
            details = answer.get('place_diagnostics') or {}
            if details.get('projected_xyz_mm'):
                print('[MAIN] Place 실제 목표(윗면 / LUT 좌표):', details.get('requested_xyz_mm'))
                print('[MAIN] Place LUT 조회 목표:', details['projected_xyz_mm'])
                print('[MAIN] Z 차이:', details.get('z_difference_mm'), 'mm')
            if details.get('contributors'):
                print('[MAIN] 참조 실측점(IDW 가중치):', details['contributors'])
            for warning in details.get('warnings', []):
                print('[MAIN/주의]', warning)
            if answer.get('error'):
                print('[MAIN/오류]', answer['error'])
            if not answer.get('motor_approved'):
                print('[MAIN] 실제 구동 차단:', answer.get('motor_block_reason', answer.get('error')))
                break
            motion_in_progress = True
            q.put({'cmd': 'execute_cycle', 'packet_id': payload['packet_id']}, timeout=2)
            try:
                motion = result_q.get(timeout=320)
            except Empty:
                print('[MAIN] 동작 응답 없음. 물리적 비상정지 및 장비 확인 필요')
                break
            print('[MAIN] 자동 적재 결과:', motion)
            if (motion.get('packet_id') != payload['packet_id'] or
                    motion.get('status') != 'cycle_complete'):
                print('[MAIN] 동작 중단. 적재 상태 수동 확인 필요')
                break
            motion_in_progress = False
            print('[MAIN] 파지/적재/복귀 명령 완료 (흡착 확인 센서 없음)')
            box_count += 1
            bin_system.print_state()
    finally:
        stop_event.set()
        if motion_in_progress:
            print('[MAIN] 물체를 지지하고 실제 장비를 안전 상태로 만든 후 종료하세요. 자동 진공 해제 금지.')
            if worker is not None and worker.is_alive():
                # Avoid closing vacuum hardware over an unsupported suspended load.
                try:
                    while input('장비 안전 확보 후 SAFE 입력: ').strip() != 'SAFE':
                        pass
                except (EOFError, KeyboardInterrupt):
                    print('[MAIN] 안전 확인 실패: 물리적 장비 상태 확인 필요')
        if q is not None and worker is not None:
            try:
                q.put({'cmd': 'shutdown', 'preserve_vacuum': motion_in_progress}, timeout=2)
            except Full:
                print('[MAIN] 종료 패킷 전송 실패')
            worker.join(timeout=320 if motion_in_progress else 30)
            if worker.is_alive():
                print("[MAIN] 프로세스 종료는 물리적 비상정지가 아닙니다. 장비 확인 필요")
                worker.terminate()
                worker.join(timeout=2)
            q.close()
            if result_q is not None:
                result_q.close()
        if camera is not None:
            camera.stop()
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass
        print('[MAIN] 종료')


if __name__ == '__main__':
    run()

