import multiprocessing as mp
import serial
import time
import cv2
import numpy as np
import pickle
from picamera2 import Picamera2
from ultralytics import YOLO
import itertools

# 1. 적재 알고리즘 클래스 (Item, LiveBin)

class Item:
    def __init__(self, name, w, h, d, weight=1):
        self.name = name
        self.w, self.h, self.d = w, h, d
        self.weight = weight
        self.x = self.y = self.z = None

    def set_position(self, x, y, z):
        self.x, self.y, self.z = x, y, z

class LiveBin:
    def __init__(self, W, H, D, max_weight=9999):
        self.W, self.H, self.D = W, H, D
        self.max_weight = max_weight
        self.current_weight = 0
        self.placed_items = []
        self.extreme_points = [(0, 0, 0)]

    def check_collision(self, item, x, y, z):
        for p in self.placed_items:
            if not (x + item.w <= p.x or p.x + p.w <= x or
                    y + item.d <= p.y or p.y + p.d <= y or
                    z + item.h <= p.z or p.z + p.h <= z):
                return True
        return False

    def fits_in_container(self, item, x, y, z):
        return (x + item.w <= self.W and y + item.d <= self.D and z + item.h <= self.H)

    def calculate_score(self, x, y, z):
        return z * 10000 + y * 100 + x

    def place_item(self, item):
        if self.current_weight + item.weight > self.max_weight:
            return False, False

        best_position = None
        best_score = float("inf")
        
        # Z축 회전(바닥면 회전) 2가지 케이스
        rotations = [
            (item.w, item.h, item.d, False), # 원본
            (item.d, item.h, item.w, True)   # 90도 회전
        ]
        
        for (rw, rh, rd, rotated_flag) in rotations:
            for (x, y, z) in self.extreme_points:
                temp_item = Item(item.name, rw, rh, rd, item.weight)

                if not self.fits_in_container(temp_item, x, y, z):
                    continue

                if self.check_collision(temp_item, x, y, z):
                    continue

                score = self.calculate_score(x, y, z)

                if score < best_score:
                    best_score = score
                    # 모든 정보를 포함하여 저장
                    best_position = (x, y, z, rw, rh, rd, rotated_flag)

        if best_position is None:
            return False, False

        # 저장한 7개의 변수를 정확히 언패킹
        x, y, z, rw, rh, rd, best_rotated = best_position
        item.w, item.h, item.d = rw, rh, rd
        item.set_position(x, y, z)

        self.placed_items.append(item)
        self.current_weight += item.weight
        self.update_extreme_points(item)

        return True, best_rotated

    # 다음 적재 지점을 계산
    def update_extreme_points(self, item):
        new_points = [
            (item.x + item.w, item.y, item.z),
            (item.x, item.y + item.d, item.z),
            (item.x, item.y, item.z + item.h),
        ]
        for p in new_points:
            # 컨테이너 범위를 벗어나지 않는 점만 추가
            if p[0] < self.W and p[1] < self.D and p[2] < self.H:
                if p not in self.extreme_points:
                    self.extreme_points.append(p)

    def print_state(self):
        print("\n[현재 적재 상태]")
        for i in self.placed_items:
            print(f"- {i.name}: 위치({i.x},{i.y},{i.z}) 크기({i.w},{i.h},{i.d})")
        print(f"총 무게: {self.current_weight}/{self.max_weight}")

# 2. 비전 처리 보조 함수 (Canny)

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


# 3. 메인 시스템 설정 및 실행

if __name__ == "__main__": 
    command_q = mp.Queue()
    
    # p_motor = mp.Process(target=motor_worker, args=(command_q,))
    # p_motor.start()

    BOX_DATA = {
        "small_box": {"w": 140.0, "h": 100.0, "d": 50.0, "weight": 1.0},
        "large_box": {"w": 120.0, "h": 110.0, "d": 53.0, "weight": 1.0},
        "long_box":  {"w": 150.0, "h": 100.0, "d": 100.0, "weight": 1.0}
    }

    with open('camera_calibration.pkl', 'rb') as f:
        calib = pickle.load(f)
    mtx, dist = calib['camera_matrix'], calib['dist_coeffs']
    #여기에 YOLO모델 
    model = YOLO('best.pt') 
    bin_system = LiveBin(20, 20, 20, max_weight=100)

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
                        # Canny로 정밀 파지점 계산
                        roi = undistorted[y1:y2, x1:x2]
                        edge_info = get_short_edge_center(roi) if roi.size > 0 else None
                        
                        if edge_info:
                            rel_center, box_pts = edge_info
                            abs_center = (rel_center[0] + x1, rel_center[1] + y1)
                            
                            # 시각화
                            cv2.drawContours(undistorted, [box_pts + [x1, y1]], 0, (0, 255, 0), 2)
                            cv2.circle(undistorted, abs_center, 5, (0, 0, 255), -1)

                            # 적재 트리거 (A키)
                            if key == ord('a'):
                                spec = BOX_DATA[label]
                                item = Item(label, spec['w'], spec['h'], spec['d'], spec['weight'])
                                if bin_system.place_item(item):
                                    print(f"\n[성공] {label} -> 위치({item.x}, {item.y}, {item.z})")
                                    print(f"-> 파지 픽셀 좌표: {abs_center}")
                                    bin_system.print_state()
                                else:
                                    print(f"[실패] {label} 적재 불가")

                    cv2.rectangle(undistorted, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(undistorted, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.imshow("Robot Vision System", undistorted)
            if key == ord('q'): break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
