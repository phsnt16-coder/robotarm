import cv2
import numpy as np
import pickle
import os
import serial
from picamera2 import Picamera2

# 1. 분리된 모듈 임포트
from ArUco import live_aruco_detection  # [ID, 각도, [X, Y, Z]] 반환
from py3Dbp import Item, LiveBin        # 적재 알고리즘

# 2. UART 통신 설정 (라즈베리 파이 5 기본 포트)
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

def send_combined_packet(label, pick_coords, load_coords, angle):
    """ 
    파지 좌표(Pick)와 적재 좌표(Load)를 하나의 패킷으로 통합 전송 
    포맷: STX,라벨,PickX,PickY,PickZ,LoadX,LoadY,LoadZ,각도,ETX
    """
    px, py, pz = pick_coords
    lx, ly, lz = load_coords
    
    packet = (f"STX,{label},{px:.1f},{py:.1f},{pz:.1f},"
              f"{lx:.1f},{ly:.1f},{lz:.1f},{angle:.1f},ETX\n")
    
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
            # 6. ArUco 엔진 실행
            result = live_aruco_detection(calib, picam2)

            if result:
                marker_id, matched_angle, pick_coords = result
                
                if marker_id in BOX_DATA:
                    spec = BOX_DATA[marker_id]
                    label = spec["label"]

                    # 7. 적재 위치 계산 (py3Dbp)
                    new_item = Item(label, spec['w'], spec['h'], spec['d'], spec['weight'])
                    success, is_rotated = bin_system.place_item(new_item)

                    if success:
                        load_coords = [new_item.x, new_item.y, new_item.z]
                        
                        # --- 프롬프트 출력 강화 ---
                        print("\n" + "="*50)
                        print(f"[데이터 확정] ID: {marker_id} ({label})")
                        print(f"[파지 좌표] X:{pick_coords[0]:.1f}, Y:{pick_coords[1]:.1f}, Z:{pick_coords[2]:.1f}")
                        print(f"[적재 좌표] X:{load_coords[0]:.1f}, Y:{load_coords[1]:.1f}, Z:{load_coords[2]:.1f}")
                        print(f"[회전 각도] {matched_angle:.2f} deg")
                        print("="*50)

                        # 8. 파지+적재 통합 데이터 UART 전송
                        send_combined_packet(label, pick_coords, load_coords, matched_angle)
                        
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
