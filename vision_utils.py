import cv2
import numpy as np

def get_short_edge_center(roi):
    """
    ROI 내에서 가장 큰 물체의 짧은 변 중앙점과 박스 포인트를 반환합니다.
    """
    if roi is None or roi.size == 0:
        return None
        
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    
    # 가장 큰 윤곽선 선택
    cnt = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    
    # 두 변의 길이 계산하여 짧은 변 찾기
    e1 = np.linalg.norm(box[0] - box[1])
    e2 = np.linalg.norm(box[1] - box[2])
    
    # 짧은 변의 양 끝점 선택
    p1, p2 = (box[0], box[1]) if e1 < e2 else (box[1], box[2])
    
    # 짧은 변의 중앙점 계산
    center_edge = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
    
    return center_edge, box

def undistort_frame(frame, mtx, dist):
    """
    카메라 왜곡을 보정합니다.
    """
    h, w = frame.shape[:2]
    newmtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
    return cv2.undistort(frame, mtx, dist, None, newmtx)
