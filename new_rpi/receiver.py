#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Updated pose sequence using the current main/IK queue interface."""
import math
import time
from robot_ik import RobotIKController
from MOTOR_CONTROL import RobotArm


def parse_packet(packet):
    if not isinstance(packet, dict):
        raise ValueError('Queue 패킷은 dict여야 합니다')
    for key in ('label', 'pick', 'load', 'angle', 'place_ik_input', 'place_ik_reference'):
        if key not in packet:
            raise KeyError(f'필수 패킷 항목 누락: {key}')
    if packet['place_ik_reference'] != 'upper_face_bin_z':
        raise ValueError('Place IK는 upper_face_bin_z 기준 좌표만 허용합니다')
    pick = tuple(float(v) for v in packet['pick'])
    load = tuple(float(v) for v in packet['load'])
    place = tuple(float(v) for v in packet['place_ik_input'])
    angle = float(packet['angle'])
    if any(len(v) != 3 for v in (pick, load, place)):
        raise ValueError('pick/load/place_ik_input은 길이 3이어야 합니다')
    if not all(math.isfinite(v) for v in (*pick, *load, *place, angle)):
        raise ValueError('좌표 및 각도에 유효하지 않은 값이 있습니다')
    return str(packet['label']), pick, load, angle, place


def execute_original_sequence(arm, label, pick_ik, place_ik, angle,
                              reply, state, stop_event=None):
    """Pose order: 4, 5, 1, 5, 2, 3, 5, 4."""
    def step(name, func, *args, delay=0.0, **kwargs):
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError('중단 요청: 추가 포즈 전송 금지')
        reply['stage'] = name
        print('[AUTO]', name, flush=True)
        result = func(*args, **kwargs)
        if result is False:
            raise RuntimeError(f'{name} 실행 실패')
        if delay:
            time.sleep(delay)

    # 4 : reset
    step('RESET_START', arm.reset, delay=2.0)
    # 5 : ready
    step('READY_PICK', arm.ready, delay=2.0)
    # 1 : pick_ik
    step('PICK_IK', arm.pick_ik,
         box_type=label, ik_angles=pick_ik, delay=1.0)
    state['holding'] = True
    step('VACUUM_ON', arm.vacuum_on, delay=0.3)
    # 5 : ready
    step('READY_LIFT', arm.ready, delay=2.0)
    # 2 : rotation
    step('ROTATION', arm.rotation, target_deg1=90, delay=1.0)
    # 3 : place_ik
    step('PLACE_IK', arm.place_ik, ik_angles=place_ik, delay=1.0)
    step('VACUUM_OFF', arm.vacuum_off, delay=0.3)
    state['holding'] = False
    # 5 : ready
    step('READY_RETURN', arm.ready, delay=2.0)
    # 4 : reset
    step('RESET_END', arm.reset, delay=2.0)
    print('[AUTO] SEQUENCE COMPLETE', flush=True)


def motor_control_receiver(q, result_q, stop_event=None):
    ik = RobotIKController()
    arm = None
    pending = None
    state = {'holding': False}
    initialized = False
    while True:
        packet = q.get()
        if isinstance(packet, dict) and packet.get('cmd') == 'shutdown':
            if arm is not None:
                if state['holding'] or packet.get('preserve_vacuum'):
                    # Do not vent a potentially suspended box on fault.
                    if arm.is_open:
                        arm.port_handler.closePort()
                        arm.is_open = False
                    print('[RECEIVER] 진공 출력 유지; 작업자 안전 조치 필요', flush=True)
                else:
                    arm.close_robot()
            return
        if isinstance(packet, dict) and packet.get('cmd') == 'execute_cycle':
            pid = packet.get('packet_id')
            reply = {'packet_id': pid, 'status': 'cycle_failed'}
            try:
                if pending is None or pending[0] != pid:
                    raise RuntimeError('실행 승인된 포즈 없음')
                _, label, pick_ik, place_ik, angle = pending
                pending = None
                if stop_event is not None and stop_event.is_set():
                    raise RuntimeError('중단 요청')
                if arm is None:
                    arm = RobotArm()
                    if not arm.open_robot_port():
                        raise RuntimeError('모터 포트 열기 실패')
                if not initialized:
                    arm.save_initial_positions()
                    initialized = True
                if arm.init_gripper() is None:
                    raise RuntimeError('진공 그리퍼 초기화 실패')
                execute_original_sequence(arm, label, pick_ik, place_ik, angle,
                                          reply, state, stop_event)
                reply['status'] = 'cycle_complete'
            except Exception as exc:
                pending = None
                reply['error'] = str(exc)
                reply['may_hold_box'] = state['holding']
                print('[AUTO ERROR]', reply, flush=True)
            result_q.put(reply)
            continue
        pending = None
        packet_id = packet.get('packet_id') if isinstance(packet, dict) else None
        answer = {'packet_id': packet_id, 'status': 'packet_failed',
                  'pick_ik': None, 'place_ik': None,
                  'motor_approved': False}
        try:
            label, pick, load, angle, place = parse_packet(packet)
            print(f'[RECEIVER] #{packet_id} {label}: Pick={pick}, BPP load={load}, '
                  f'Place top LUT={place}, Outline angle={angle}', flush=True)
            # No duplicate legacy offset (-13.3,-80,+80). Base coordinate from main.
            answer['status'] = 'pick_failed'
            pick_ik = ik.calculate_pick_ik(pick)
            ik.validate_output_range(pick_ik)
            # Preserve original receiver wrist-angle rule; no duplicate XYZ offsets.
            answer['pick_ik'] = pick_ik
            answer['status'] = 'place_failed'
            place_result = ik.operational_place_ik(place)
            answer['place_diagnostics'] = place_result
            if place_result['status'] == 'ok':
                answer['place_ik'] = place_result['angles']
                answer['status'] = 'ik_ready'
            else:
                answer['error'] = place_result['error']
            if answer['pick_ik'] and answer['place_ik']:
                # Both IK calculations returned angles; no additional approval filter.
                answer['motor_approved'] = True
                pending = (packet_id, label, pick_ik, answer['place_ik'], angle)
            print(f'[RECEIVER] #{packet_id}: {answer["status"]}', flush=True)
        except (ValueError, TypeError, KeyError, FileNotFoundError) as exc:
            answer['error'] = str(exc)
            print(f'[RECEIVER] {answer["status"]}: {exc}', flush=True)
        except Exception as exc:
            answer['status'] = 'unexpected_error'
            answer['error'] = f'{type(exc).__name__}: {exc}'
            print(f'[RECEIVER] {answer["error"]}', flush=True)
        result_q.put(answer)


# Backward-compatible import name for callers using the new two-argument API.
ik_preview_receiver = motor_control_receiver

