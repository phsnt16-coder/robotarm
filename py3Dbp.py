import multiprocessing as mp
import serial
import time
import cv2
import numpy as np
import pickle
from picamera2 import Picamera2
from ultralytics import YOLO
import itertools

from py3Dbp import Item, LiveBin


# 비전 처리 보조 함수
def get_short_edge_center(roi):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    
    cnt = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(cnt)
    box = np.int0(cv2.boxPoints(rect))
    
    e1 = np.linalg.norm(box[0] - box[1])
    e2 = np.linalg.norm(box[1] - box[2])
    p1, p2 = (box[0], box[1]) if e1 < e2 else (box[1], box[2])
    
    center_edge = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
    return center_edge, box

if __name__ == "__main__": 
    # 1. 멀티프로세싱 설정
    command_q = mp.Queue()
    
    # 이전에 만든 모터 제어 프로세스가 있다면 여기서 시작
    # p_motor = mp.Process(target=motor_worker, args=(command_q,))
    # p_motor.start()

    # 2. 초기 설정 데이터
    BOX_DATA = {
        "small_box": {"w": 140.0, "h": 100.0, "d": 50.0, "weight": 1.0},
        "large_box": {"w": 120.0, "h": 110.0, "d": 53.0, "weight": 1.0},
        "long_box":  {"w": 150.0, "h": 100.0, "d": 100.0, "weight": 1.0}
    }

    with open('camera_calibration.pkl', 'rb') as f:
        calib = pickle.load(f)
    mtx, dist = calib['camera_matrix'], calib['dist_coeffs']
    
    model = YOLO('best.pt')
    bin_system = LiveBin(200, 200, 200, max_weight=100)

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    print("시스템 가동... (A: 적재 확정, Q: 종료)")

    try:
        while True:
            frame_raw = picam2.capture_array()
            frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]

            newmtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w,h))
            undistorted = cv2.undistort(frame, mtx, dist, None, newmtx)

            results = model(undistorted, stream=True, verbose=False)
            key = cv2.waitKey(1) & 0xFF

            for r in results:
                for box_yolo in r.boxes:
                    x1, y1, x2, y2 = map(int, box_yolo.xyxy[0])
                    label = model.names[int(box_yolo.cls[0])]
                    
                    if label in BOX_DATA:
                        roi = undistorted[y1:y2, x1:x2]
                        edge_info = get_short_edge_center(roi) if roi.size > 0 else None
                        
                        if edge_info:
                            rel_center, box_pts = edge_info
                            abs_center = (rel_center[0] + x1, rel_center[1] + y1)
                            
                            cv2.drawContours(undistorted, [box_pts + [x1, y1]], 0, (0, 255, 0), 2)
                            cv2.circle(undistorted, abs_center, 5, (0, 0, 255), -1)

                            # --- 적재 로직 처리 구간 ---
                            if key == ord('a'):
                                spec = BOX_DATA[label]
                                # 1. Item 객체 생성
                                new_item = Item(label, spec['w'], spec['h'], spec['d'], spec['weight'])
                                
                                # 2. 적재 알고리즘 실행 (성공 여부, 회전 여부 수신)
                                success, is_rotated = bin_system.place_item(new_item)
                                
                                if success:
                                    print(f"\n[성공] {label} 배정 완료")
                                    
                                    # 3. 큐 전송용 데이터 패키징
                                    control_data = {
                                        "label": label,
                                        "pick_pixel": abs_center,      # 카메라 상의 파지점
                                        "target_xyz": (new_item.x, new_item.y, new_item.z), # 적재 좌표
                                        "is_rotated": is_rotated       # 90도 회전 여부
                                    }
                                    
                                    # 4. 큐에 넣기 (소비자 프로세스로 전송)
                                    command_q.put(control_data)
                                    
                                    bin_system.print_state()
                                else:
                                    print(f"[실패] {label}을 넣을 공간이 없습니다.")
                            # -------------------------

                    cv2.rectangle(undistorted, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(undistorted, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.imshow("Robot Vision System", undistorted)
            if key == ord('q'): 
                command_q.put("EXIT")
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
