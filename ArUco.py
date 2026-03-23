#copywrite by 로보고니
import cv2
import numpy as np
import os
import pickle
from picamera2 import Picamera2

def live_aruco_detection(calibration_data):
    camera_matrix = calibration_data['camera_matrix']
    dist_coeffs = calibration_data['dist_coeffs']
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    # 마커 크기 (미터 단위) - 실제 마커 한 변의 길이
    marker_size = 0.05 

    # 마커의 3D 좌표 정의 (OpenCV solvePnP 양식)
    # 마커의 중심을 (0,0,0)으로 잡는 기준 좌표
    obj_points = np.array([
        [-marker_size / 2,  marker_size / 2, 0], #좌상단        
        [ marker_size / 2,  marker_size / 2, 0], #우상단
        [ marker_size / 2, -marker_size / 2, 0], #좌하단
        [-marker_size / 2, -marker_size / 2, 0]  #우하단
    ], dtype=np.float32)

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    try:
        while True:
            frame_rgb = picam2.capture_array()
            if frame_rgb is None: continue
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            # 왜곡 보정
            h, w = frame.shape[:2]
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 0, (w,h))
            dst = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_mtx)

            # 마커 검출
            corners, ids, _ = detector.detectMarkers(dst)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(dst, corners, ids)
                
                for i in range(len(ids)):
                    # solvePnP를 사용하여 포즈(rvec, tvec) 계산
                    # 보정된 영상이므로 왜곡 계수는 0(np.zeros)으로 설정
                    _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], new_mtx, np.zeros(5))
                    
                    # 좌표축 그리기
                    cv2.drawFrameAxes(dst, new_mtx, np.zeros(5), rvec, tvec, marker_size/2)
                    
                    # 위치 좌표 추출 (tvec)
                    px, py, pz = tvec.flatten()
                    
                    # 화면 표시 (단위: mm)
                    text = f"ID:{ids[i][0]} POS: {px*1000:.1f}, {py*1000:.1f}, {pz*1000:.1f}mm"
                    cv2.putText(dst, text, (int(corners[i][0][0][0]),
                                            int(corners[i][0][0][1])-10),
                                            cv2.FONT_HERSHEY_SIMPLEX,
                                            0.4, (0, 255, 0), 2)

            cv2.imshow('ArUco Pose Estimation', dst)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

def main():
    pkl_path = 'camera_calibration.pkl'
    if os.path.exists(pkl_path):
        with open(pkl_path, 'rb') as f:
            data = pickle.load(f)
        live_aruco_detection(data)
    else:
        print("캘리브레이션 파일이 없습니다.")

if __name__ == "__main__":
    main()
