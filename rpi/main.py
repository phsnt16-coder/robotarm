import cv2
import numpy as np
import pickle
import os
import serial
from picamera2 import Picamera2

# 1. 분리된 모듈 import
from ArUco import live_aruco_detection  # 파지좌표 반환 알고리즘
from py3Dbp import Item, LiveBin        # 적재 알고리즘

# 2. UART 통신 설정 
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

def send_combined_packet(label, pick_coords, load_coords, angle, is_rotated, success):
    
    #파지 좌표(Pick)와 적재 좌표(Load)를 하나의 패킷 단위로 전송 
    #포맷: STX,라벨,PickX,PickY,PickZ,LoadX,LoadY,LoadZ,각도,ETX
    
    px, py, pz = pick_coords
    lx, ly, lz = load_coords
    #TRUE, FALSE를 정수(0 또는 1)로 변환하여 전송
    rotated_int = 1 if is_rotated else 0
    success_int = 1 if success else 0
    
    packet = (f"STX,{label},{px:.1f},{py:.1f},{pz:.1f},"
              f"{lx:.1f},{ly:.1f},{lz:.1f},{angle:.1f},"
              f"{rotated_int},{success_int},ETX\n")
    
    ser.write(packet.encode('utf-8'))
    print(f"\n[통합 패킷 전송] {packet.strip()}")
    
if __name__ == "__main__":
    # 3. 상자 규격 데이터 (ID 매칭)
    BOX_DATA = {
        1: {"label": "A_1", "w": 140.0, "h": 50.0,  "d": 100.0, "weight": 1.0},
        2: {"label": "B_3", "w": 120.0, "h": 53.0,  "d": 110.0, "weight": 1.0},
        3: {"label": "B_8", "w": 150.0, "h": 100.0, "d": 100.0, "weight": 1.0}
    }

    # 4. 카메라 캘리브레이션 로드
    try:
        with open('camera_calibration.pkl', 'rb') as f:
            calib = pickle.load(f)
    except FileNotFoundError:
        print("에러: camera_calibration.pkl 파일이 없습니다.")
        exit()

    # 카메라 초기화
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    # 5. 적재 시스템 초기화
    bin_system = LiveBin(200, 200, 200, max_weight=100)

    print("\n[시스템 가동] 'A'키를 눌러 파지 및 적재를 확정하세요.")

    try:
        while True:
            # 6. ArUco 실행
            result = live_aruco_detection(calib, picam2)

            if result:
                marker_id, matched_angle, pick_coords, up_axis = result
                
                if marker_id in BOX_DATA:
                    spec = BOX_DATA[marker_id]
                    label = spec["label"]
                    if up_axis == 0:   # X축이 수직 (Side면이 보임)
                        #w_top, d_top, h_now = spec['h'], spec['d'], spec['w']
                        w_top, d_top, h_now = spec['h'], spec['w'], spec['d']
                    elif up_axis == 1: # Y축이 수직 (Front면이 보임)
                        
                        w_top, d_top, h_now = spec['h'], spec['d'], spec['w']
                    else:              # Z축이 수직 (Top면이 보임)
                        
                        w_top, d_top, h_now = spec['w'], spec['d'], spec['h']

                    # 7. 적재 위치 계산 (py3Dbp)
                    # 이제 py3Dbp는 상자가 '놓여 있는 면'을 기준으로 계산함
                    new_item = Item(label, w_top, h_now, d_top, spec['weight'])
                    success, is_rotated = bin_system.place_item(new_item)

                    if success:
                        # 4. 그리퍼 목표 좌표 계산 (모서리 좌표 + 가로/2 + 세로/2)
                        # item.w와 item.d는 py3Dbp 내부에서 회전(is_rotated)까지 반영된 최종 가로/세로임
                        gripper_lx = new_item.x + (new_item.w / 2)
                        gripper_ly = new_item.y + (new_item.d / 2)
                        gripper_lz = new_item.z + new_item.h # 윗면 높이
        
                        load_coords = [gripper_lx, gripper_ly, gripper_lz]
                        
                        # 8. 프롬프트 확인
                        print("\n" + "="*50)
                        print(f"[데이터 확정] ID: {marker_id} ({label})")
                        print(f"[파지 좌표] X:{pick_coords[0]:.1f}, Y:{pick_coords[1]:.1f}, Z:{pick_coords[2]:.1f}")
                        print(f"[적재 좌표] X:{load_coords[0]:.1f}, Y:{load_coords[1]:.1f}, Z:{load_coords[2]:.1f}")
                        print(f"[회전 각도] {matched_angle:.2f} deg")
                        print("="*50)

                        # 9. 데이터 UART 전송
                        send_combined_packet(label, pick_coords, load_coords, matched_angle, is_rotated, success)
                        
                        bin_system.print_state()
                    else:
                        print(f"\n >>> [알림] {label} 적재 공간 부족")
            
            user_input = input("\n다음 상자를 인식하시겠습니까? (y/q): ")
            if user_input.lower() == 'q':
                break

    finally:
        picam2.stop()
        if ser.is_open:
            ser.close()
        print("\n시스템을 종료합니다.")
