import multiprocessing as mp
import cv2
import numpy as np
import pickle
import time
from picamera2 import Picamera2
from ultralytics import YOLO

# 분리된 모듈 임포트
from py3Dbp import Item, LiveBin
from vision_utils import get_short_edge_center

if __name__ == "__main__":
    # 1. 멀티프로세싱 및 통신 설정
    command_q = mp.Queue()

    # 2. 아이템 규격 데이터 (실제 측정 데이터로 수정 가능)
    # 단위: mm (Bin 크기와 단위를 맞춰야 함)
    BOX_DATA = {
        "A_1": {"w": 140.0, "h": 50.0,  "d": 100.0, "weight": 1.0},
        "B_3": {"w": 120.0, "h": 53.0,  "d": 110.0, "weight": 1.0},
        "B_8":  {"w": 150.0, "h": 100.0, "d": 100.0, "weight": 1.0}
    }

    # 3. 카메라 설정 및 왜곡 보정 데이터 로드
    try:
        with open('camera_calibration.pkl', 'rb') as f:
            calib = pickle.load(f)
        mtx, dist = calib['camera_matrix'], calib['dist_coeffs']
    except FileNotFoundError:
        print("에러: camera_calibration.pkl 파일이 없습니다.")
        exit()

    # [최적화] 매 프레임 계산하지 않도록 루프 밖에서 미리 계산
    W_IMG, H_IMG = 640, 480
    newmtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (W_IMG, H_IMG), 0, (W_IMG, H_IMG))

    # 4. 모델 및 적재 시스템 초기화
    model = YOLO('best.pt')
    # 적재함 크기 설정 (예: 200x200x200 mm)
    bin_system = LiveBin(200, 200, 200, max_weight=100)

    # 5. 카메라 가동
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (W_IMG, H_IMG), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    print("\n[시스템 가동] 화면을 보며 'A'키를 누르면 적재가 확정됩니다. (종료: Q)")

    try:
        while True:
            # 프레임 획득 및 전처리
            frame_raw = picam2.capture_array()
            frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)
            
            # 왜곡 보정 (함수 대신 직접 연산하여 속도 향상)
            undistorted = cv2.undistort(frame, mtx, dist, None, newmtx)

            # YOLO 추론
            results = model(undistorted, stream=True, verbose=False)
            key = cv2.waitKey(1) & 0xFF

            for r in results:
                for box_yolo in r.boxes:
                    x1, y1, x2, y2 = map(int, box_yolo.xyxy[0])
                    conf = box_yolo.conf[0]
                    label = model.names[int(box_yolo.cls[0])]
                    
                    if label in BOX_DATA and conf > 0.5:
                        # 1. 파지점 검출 (ROI 추출)
                        roi = undistorted[y1:y2, x1:x2]
                        edge_info = get_short_edge_center(roi)
                        
                        if edge_info:
                            rel_center, box_pts = edge_info
                            abs_center = (rel_center[0] + x1, rel_center[1] + y1)
                            
                            # 시각화 (박스 윤곽선 및 파지 포인트)
                            cv2.drawContours(undistorted, [box_pts + [x1, y1]], 0, (0, 255, 0), 2)
                            cv2.circle(undistorted, abs_center, 5, (0, 0, 255), -1)

                            # 2. 적재 트리거 (A키 입력 시)
                            if key == ord('a'):
                                spec = BOX_DATA[label]
                                # Item 객체 생성
                                new_item = Item(label, spec['w'], spec['h'], spec['d'], spec['weight'])
                                
                                # 적재 알고리즘 실행
                                success, is_rotated = bin_system.place_item(new_item)
                                
                                if success:
                                    # 로봇 팔 프로세스로 보낼 데이터 패키징
                                    control_data = {
                                        "label": label,
                                        "pick_pixel": abs_center,          # 그리퍼가 내려갈 픽셀 위치
                                        "target_xyz": (new_item.x, new_item.y, new_item.z), # 적재함 내 좌표
                                        "is_rotated": is_rotated           # 그리퍼 90도 회전 여부
                                    }
                                    command_q.put(control_data)
                                    
                                    print(f"\n[성공] {label} 배치: ({new_item.x}, {new_item.y}, {new_item.z})")
                                    print(f"      회전 여부: {is_rotated}")
                                    bin_system.print_state()
                                else:
                                    print(f"\n[실패] {label}을 적재할 공간이 부족합니다.")

                        # 기본 YOLO 박스 표시
                        cv2.rectangle(undistorted, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(undistorted, f"{label} {conf:.2f}", (x1, y1-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # 화면 출력
            cv2.imshow("Robot Vision System", undistorted)
            
            if key == ord('q'):
                command_q.put("EXIT")
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print("\n시스템을 종료합니다.")
