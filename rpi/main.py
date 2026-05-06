import cv2
import numpy as np
import pickle
import time
from picamera2 import Picamera2

# 1. 모듈 import (RobotArmSerial -> sendrpi)
from ArUco import live_aruco_detection
from py3Dbp import Item, LiveBin
from sendrpi import RobotArmSerial 

if __name__ == "__main__":
    # 2. 상자 데이터 
    BOX_DATA = {
        1: {"name": "A_1", "w": 140.0, "h": 50.0,  "d": 100.0, "weight": 1.0},
        2: {"name": "B_3", "w": 120.0, "h": 53.0,  "d": 110.0, "weight": 1.0},
        3: {"name": "B_8", "w": 150.0, "h": 100.0, "d": 100.0, "weight": 1.0}
    }

    # 3. 통신 및 카메라 초기화
    arm = RobotArmSerial(port='/dev/ttyAMA0') # ttyAMA0 사용
    
    try:
        with open('camera_calibration.pkl', 'rb') as f:
            calib = pickle.load(f)
    except FileNotFoundError:
        print("에러: 캘리브레이션 파일 없음")
        exit()

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    # 4. 적재 시스템 초기화
    bin_system = LiveBin(200, 200, 200, max_weight=100)

    print("\n[시스템 가동] 인식 시작...")

    try:
        while True:
            result = live_aruco_detection(calib, picam2)

            if result:
                marker_id, matched_angle, pick_coords, up_axis = result
                
                if marker_id in BOX_DATA:
                    spec = BOX_DATA[marker_id]
                    
                    # 축 방향에 따른 규격 결정
                    if up_axis == 0:   w_top, d_top, h_now = spec['h'], spec['w'], spec['d']
                    elif up_axis == 1: w_top, d_top, h_now = spec['h'], spec['d'], spec['w']
                    else:              w_top, d_top, h_now = spec['w'], spec['d'], spec['h']

                    # 5. 적재 알고리즘 계산
                    new_item = Item(spec['name'], w_top, h_now, d_top, spec['weight'])
                    success, is_rotated = bin_system.place_item(new_item)

                    if success:
                        # 적재 목표 좌표(그리퍼 기준)
                        load_coords = [
                            new_item.x + (new_item.w / 2),
                            new_item.y + (new_item.d / 2),
                            new_item.z + new_item.h
                        ]

                        print(f"\n[데이터 확정] ID: {marker_id} ({spec['name']})")
                        
                        # 6. 압축 패킷 전송 (19바이트)
                        # send_mission(ID, 회전여부, 성공여부, 파지좌표, 적재좌표, 각도)
                        arm.send_mission(marker_id, is_rotated, success, 
                                         pick_coords, load_coords, matched_angle)
                        
                        # 7. 동작 시작 명령
                        arm.start_task()
                        
                        bin_system.print_state()
                    else:
                        print(f"\n >>> [알림] {spec['name']} 공간 부족")
            
            if input("\n계속하시겠습니까? (y/q): ").lower() == 'q':
                break

    finally:
        picam2.stop()
        arm.close()
        print("\n시스템을 종료합니다.")
