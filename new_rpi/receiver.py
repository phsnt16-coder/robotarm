#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Queue -> receiver -> Pick/Place IK non-actuating integration test.

Compatible with main.py from the previous preview ZIP. No motor imports.
Unsupported Z queries are PREVIEW ONLY projected to the nearest measured plane.
"""
import math
from robot_ik import RobotIKController


def parse_packet(packet):
    if not isinstance(packet, dict):
        raise ValueError('Queue 패킷은 dict여야 합니다')
    for key in ('label', 'pick', 'load', 'angle', 'place_ik_input', 'place_ik_reference'):
        if key not in packet:
            raise KeyError(f'필수 패킷 항목 누락: {key}')
    if packet['place_ik_reference'] != 'box_center':
        raise ValueError('Place IK는 box_center 기준 좌표만 허용합니다')
    pick = tuple(float(v) for v in packet['pick'])
    load = tuple(float(v) for v in packet['load'])
    place = tuple(float(v) for v in packet['place_ik_input'])
    angle = float(packet['angle'])
    if any(len(v) != 3 for v in (pick, load, place)):
        raise ValueError('pick/load/place_ik_input은 길이 3이어야 합니다')
    if not all(math.isfinite(v) for v in (*pick, *load, *place, angle)):
        raise ValueError('좌표 및 각도에 유효하지 않은 값이 있습니다')
    return str(packet['label']), pick, load, angle, place


def ik_preview_receiver(q, result_q):
    """Main multiprocessing.Process target; never commands servos or vacuum."""
    ik = RobotIKController()
    print('[RECEIVER] IK 미리보기 시작 (모터 비구동)', flush=True)
    while True:
        packet = q.get()
        if isinstance(packet, dict) and packet.get('cmd') == 'shutdown':
            print('[RECEIVER] 종료', flush=True)
            return
        packet_id = packet.get('packet_id') if isinstance(packet, dict) else None
        answer = {'packet_id': packet_id, 'status': 'packet_failed',
                  'pick_ik': None, 'place_ik': None,
                  'motor_approved': False}
        try:
            label, pick, load, angle, place = parse_packet(packet)
            print(f'[RECEIVER] #{packet_id} {label}: Pick={pick}, BPP load={load}, '
                  f'Place center={place}, Outline angle={angle}', flush=True)
            # No duplicate legacy offset (-13.3,-80,+80). Base coordinate from main.
            answer['status'] = 'pick_failed'
            pick_ik = ik.calculate_pick_ik(pick)
            ik.validate_output_range(pick_ik)
            answer['pick_ik'] = pick_ik
            answer['status'] = 'place_blocked'
            place_result = ik.preview_place_ik(place)
            answer['place_diagnostics'] = place_result
            if place_result['status'] in ('ok', 'approximate_z_projection'):
                answer['place_ik'] = place_result['angles']
                answer['status'] = ('ik_preview_approximate' if place_result['status'] == 'approximate_z_projection' else 'ik_preview_ok')
            else:
                answer['error'] = place_result['error']
            print(f'[RECEIVER] #{packet_id}: {answer["status"]}', flush=True)
        except (ValueError, TypeError, KeyError, FileNotFoundError) as exc:
            answer['error'] = str(exc)
            print(f'[RECEIVER] {answer["status"]}: {exc}', flush=True)
        except Exception as exc:
            answer['status'] = 'unexpected_error'
            answer['error'] = f'{type(exc).__name__}: {exc}'
            print(f'[RECEIVER] {answer["error"]}', flush=True)
        result_q.put(answer)


def motor_control_receiver(q):
    raise RuntimeError('안전 차단: 이 버전은 모터 구동을 지원하지 않습니다. IK 계산 검증 전용입니다.')


if __name__ == '__main__':
    print('main.py에서 Process(target=ik_preview_receiver, args=(q, result_q))로 실행하세요.')
