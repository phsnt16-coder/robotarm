"""Outline + BPP + placement-coordinate test main.

Default: camera and BPP operate; NO receiver or motor movement.
A/B fixed TCP points are intended for IK coordinate checks, NOT proof of BPP placement.
Switch TARGET_MODE to 'bpp' to use bin-corner geometry instead.
"""

import cv2
from multiprocessing import Process, Queue
from picamera2 import Picamera2

from outline import live_outline_detection
from py3Dbp import Item, LiveBin
from receiver import motor_control_receiver
from visualizer import BinVisualizer

# ---- Safety and coordinate mode ---------------------------------------------
ENABLE_MOTOR_CONTROL = False  # Do not change before independent IK/collision checks
TARGET_MODE = 'fixed'        # 'fixed': A/B test targets; 'bpp': bin placement

# Latest user-specified robot Base-frame TCP targets (millimeters).
FIXED_PLACE_TARGETS = {
    'A': (60.0, 285.0, 50.0),
    'B': (-60.0, 295.0, 50.0),
}

# BPP origin in robot Base frame, with positive BPP axes mapping to -X/-Y/-Z.
# This yields NEGATIVE robot Y and Z for the majority of container locations;
# it is intentionally NOT used in 'fixed' mode.
BIN_ORIGIN_ROBOT_MM = (120.0, -225.0, 0.0)
BIN_SIZE_MM = (200.0, 200.0, 200.0)  # width, depth, height

DEFAULT_BOX_HEIGHT_MM = 50.0  # Temporary value until height sensor is integrated
DEFAULT_BOX_WEIGHT = 1.0


def bin_item_to_robot_tcp(item):
    """Convert BPP item lower corner -> upper-face center in robot coordinates."""
    ox, oy, oz = BIN_ORIGIN_ROBOT_MM
    cx = float(item.x) + float(item.w) / 2.0
    cy = float(item.y) + float(item.d) / 2.0
    top = float(item.z) + float(item.h)
    return [ox - cx, oy - cy, oz - top]


def get_fixed_target(label):
    """Only A and B are defined; never silently invent a third target."""
    if label not in FIXED_PLACE_TARGETS:
        raise ValueError(f'고정 적재 목표가 없습니다: {label}')
    return list(FIXED_PLACE_TARGETS[label])


def main():
    if TARGET_MODE not in ('fixed', 'bpp'):
        raise ValueError('TARGET_MODE must be fixed or bpp')
    if ENABLE_MOTOR_CONTROL and TARGET_MODE == 'fixed':
        raise RuntimeError('고정 목표 모드는 좌표 테스트 전용입니다. 모터 연결을 허용하지 않습니다.')

    picam2 = None
    p_receiver = None
    shared_queue = None
    box_count = 0

    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={'size': (1280, 720), 'format': 'BGR888'},
            buffer_count=4,
        )
        picam2.configure(config)
        picam2.start()
        print('[MAIN] 카메라 시작')

        bin_system = LiveBin(*BIN_SIZE_MM, max_weight=100)
        print('[MAIN] BPP 초기화 완료')
        print(f'[MAIN] TARGET_MODE={TARGET_MODE}, ENABLE_MOTOR_CONTROL={ENABLE_MOTOR_CONTROL}')
        print('[MAIN] 고정 목표 A/B:', FIXED_PLACE_TARGETS)
        print('[MAIN] BPP 원점:', BIN_ORIGIN_ROBOT_MM)

        if ENABLE_MOTOR_CONTROL:
            shared_queue = Queue(maxsize=5)
            p_receiver = Process(
                target=motor_control_receiver,
                args=(shared_queue,),
                daemon=True,
            )
            p_receiver.start()
            print('[MAIN] receiver 실행')
        else:
            print('[MAIN] 안전 테스트 모드: receiver 실행 안 함, 모터 구동 없음')

        print('[MAIN] Enter/y = 물체 인식, q = 종료')
        while True:
            cmd = input('\n명령 [Enter/y/q]: ').strip().lower()
            if cmd in ('q', 'quit', 'esc'):
                break
            if cmd not in ('', 'y'):
                continue

            # Stop after A and B in fixed mode rather than assigning other labels.
            if TARGET_MODE == 'fixed' and box_count >= 2:
                print('[MAIN] A/B 테스트 목표 모두 사용. 종료 후 다시 시작하세요.')
                continue

            result = live_outline_detection(picam2)
            if result is None:
                print('[MAIN] Outline 검출 실패/취소')
                continue

            matched_angle, pick_coords, box_size = result
            if len(box_size) != 2 or len(pick_coords) != 3:
                print('[MAIN] 검출 데이터 형식 오류')
                continue

            w_top, d_top = map(float, box_size)
            h_now = DEFAULT_BOX_HEIGHT_MM
            if w_top <= 0 or d_top <= 0 or h_now <= 0:
                print('[MAIN] 상자 치수가 올바르지 않습니다')
                continue

            label = ('A' if box_count == 0 else 'B') if TARGET_MODE == 'fixed' else f'BOX_{box_count+1}'
            print(f'[MAIN] label={label}, angle={matched_angle:.2f}, pick={pick_coords}')
            print(f'[MAIN] 감지 크기 w={w_top:.1f}, d={d_top:.1f}, 가정 높이={h_now:.1f}mm')

            new_item = Item(label, w_top, h_now, d_top, DEFAULT_BOX_WEIGHT)
            success, is_rotated = bin_system.place_item(new_item)
            if not success:
                print(f'[MAIN] BPP 공간 부족: {label}')
                continue

            bpp_target = bin_item_to_robot_tcp(new_item)
            if TARGET_MODE == 'fixed':
                load_coords = get_fixed_target(label)
                print('[MAIN] 주의: 고정 목표는 BPP 배치 위치와 독립적입니다.')
                print('[MAIN] BPP 배치/시각화는 참고용이며 실제 적재와 동기화되지 않습니다.')
            else:
                load_coords = bpp_target

            print('\n' + '=' * 50)
            print(f'Label        : {label}')
            print(f'Pick         : {list(map(float, pick_coords))}')
            print(f'BPP corner   : ({new_item.x}, {new_item.y}, {new_item.z})')
            print(f'BPP TCP robot: {bpp_target}')
            print(f'Final target : {load_coords} mm')
            print(f'Rotated      : {is_rotated}')
            print('=' * 50)

            try:
                BinVisualizer(bin_system).update_plot()
            except Exception as exc:
                print(f'[MAIN] 시각화 오류: {exc}')

            packet_payload = {
                'label': label,
                'pick': list(map(float, pick_coords)),
                'load': load_coords,
                'angle': float(matched_angle),
                'is_rotated': is_rotated,
                'success': success,
                'box_size': {'w': w_top, 'h': h_now, 'd': d_top},
            }
            if not ENABLE_MOTOR_CONTROL:
                print('[MAIN] Queue 전송 생략 (모터 없음)')
                print('[MAIN] 전송 예정 패킷:', packet_payload)
            else:
                # For bpp mode only, and only if explicitly enabled above.
                shared_queue.put(packet_payload, timeout=2)
                print('[MAIN] receiver에 패킷 전송')

            box_count += 1
            bin_system.print_state()
    finally:
        print('[MAIN] 종료 처리')
        if p_receiver is not None:
            try:
                shared_queue.put({'cmd': 'shutdown'}, timeout=1)
            except Exception as exc:
                print(f'[MAIN] 종료 패킷 오류: {exc}')
            p_receiver.join(timeout=2)
            if p_receiver.is_alive():
                p_receiver.terminate()
                p_receiver.join(timeout=2)
        if picam2 is not None:
            picam2.stop()
        cv2.destroyAllWindows()
        print('[MAIN] 종료 완료')


if __name__ == '__main__':
    main()
