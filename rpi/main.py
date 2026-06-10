import cv2
import numpy as np
import pickle
import os
import serial
import time
from multiprocessing import Process, Queue
from picamera2 import Picamera2

# 1. 모듈 import
from ArUco import live_aruco_detection  # 파지좌표 반환 알고리즘
from py3Dbp import Item, LiveBin        # 적재 알고리즘
from receiver import motor_control_receiver #수신 모듈
from visualizer import BinVisualizer   # 시각화 모듈

# 2. 메인 프로세스
if __name__ == "__main__":
    # 상자 규격 데이터
    BOX_DATA = {
        1: {"label": "A_1", "w": 140.0, "h": 50.0,  "d": 100.0, "weight": 1.0},
        2: {"label": "B_3", "w": 120.0, "h": 53.0,  "d": 110.0, "weight": 1.0},
        3: {"label": "B_8", "w": 150.0, "h": 100.0, "d": 100.0, "weight": 1.0}
    }

    # 카메라 캘리브레이션 로드
    try:
        with open('camera_calibration.pkl', 'rb') as f:
            calib = pickle.load(f)
    except FileNotFoundError:
        print("에러: camera_calibration.pkl 파일이 없습니다.")
        exit()

    # 카메라 초기화
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()

    # 적재 시스템 초기화
    bin_system = LiveBin(200, 200, 200, max_weight=100)
    
    # 프로세스 큐 생성
    shared_queue = Queue(maxsize=5)
    
    # receiver 프로세스 시작
    p_receiver = Process(target=motor_control_receiver, args=(shared_queue,))
    p_receiver.daemon = True
    p_receiver.start()

    print("\n[시스템 가동] 'y'를 눌러 인식을 시작하고 'q'로 종료합니다.")

    try:
        while True:
            # ArUco 실행
            result = live_aruco_detection(calib, picam2)

            if result:
                marker_id, matched_angle, pick_coords, up_axis = result
                
                if marker_id in BOX_DATA:
                    spec = BOX_DATA[marker_id]
                    label = spec["label"]
                    
                    # 수직 축 기준 평면 결정
                    if up_axis == 0:
                        w_top, d_top, h_now = spec['w'], spec['d'], spec['h']
                    elif up_axis == 1:
                        w_top, d_top, h_now = spec['w'], spec['d'], spec['h']
                    else:
                        w_top, d_top, h_now = spec['w'], spec['d'], spec['h']

                    pick_coords[2] += 80.0

                    # 적재 위치 계산
                    new_item = Item(label, w_top, h_now, d_top, spec['weight'])
                    success, is_rotated = bin_system.place_item(new_item)

                    if success:
                        # 그리퍼 목표 좌표 계산
                        gripper_lx = new_item.x + (new_item.w / 2)
                        gripper_ly = new_item.y + (new_item.d / 2)
                        gripper_lz = new_item.z + new_item.h
                        load_coords = [gripper_lx, gripper_ly, gripper_lz]
                        
                        # 화면 출력
                        print("\n" + "="*50)
                        print(f"[데이터 확정] ID: {marker_id} ({label})")
                        print(f"[파지 좌표] X:{pick_coords[0]:.1f}, Y:{pick_coords[1]:.1f}, Z:{pick_coords[2]:.1f}")
                        print(f"[적재 좌표] X:{load_coords[0]:.1f}, Y:{load_coords[1]:.1f}, Z:{load_coords[2]:.1f}")
                        print("="*50)
                        
                        visualizer = BinVisualizer(bin_system)
                        visualizer.update_plot()

                        # 전송용 딕셔너리 구성
                        packet_payload = {
                            'label': label,
                            'pick': pick_coords,
                            'load': load_coords,
                            'angle': matched_angle,
                            'is_rotated': is_rotated,
                            'success': success
                        }
                        # 3. 큐에 데이터 삽입
                        if not shared_queue.full():
                            shared_queue.put(packet_payload)
                        
                        bin_system.print_state()
                    else:
                        print(f"\n >>> [알림] {label} 적재 공간 부족")
            # 4. 계속 진행 확인
            user_input = input("\n다음 상자를 인식하시겠습니까? (y/q): ")
            if user_input.lower() == 'q':
                break

    finally:
        picam2.stop()
        p_receiver.terminate()
        print("\n시스템을 종료합니다.")
