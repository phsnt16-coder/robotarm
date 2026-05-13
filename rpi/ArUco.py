import cv2
import numpy as np
import os
import pickle
from picamera2 import Picamera2

# 아루코 마커로 파지좌표 반환
def live_aruco_detection(calibration_data, picam2):
    camera_matrix = calibration_data['camera_matrix']
    dist_coeffs = calibration_data['dist_coeffs']
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    
    # cv2 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    marker_size = 0.05 # 50*50mm 마커 사용

    # 마커 중앙을 원점으로 설정
    obj_points = np.array([
        [-marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)

    # 최종 반환 데이터 리스트: [ID, 각도, [X, Y, Z],중심축]
    captured_list = None

    try:
        print("프로그램 실행 중: 'a' 키를 누르면 [ID, 각도, 좌표] 리스트를 반환하고 종료. 'q'는 취소.")
        while True: 
            frame_raw = picam2.capture_array()
            if frame_raw is None: continue
            frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)
            
            h, w = frame.shape[:2]
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 0, (w,h))
            dst = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_mtx)

            corners, ids, _ = detector.detectMarkers(dst)
            current_frame_results = [] 

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(dst, corners, ids)
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    # 위치/방향 벡터값 저장
                    _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], new_mtx, np.zeros(5))
                    
                    # 방향벡터를 행렬로 변환 
                    rmat, _ = cv2.Rodrigues(rvec)
                    
                    # 각도 계산 및 축 정의
                    sy = np.sqrt(rmat[0,0]**2 + rmat[1,0]**2)
                    roll = np.degrees(np.arctan2(rmat[2,1] , rmat[2,2]))
                    pitch = np.degrees(np.arctan2(-rmat[2,0], sy))
                    yaw = np.degrees(np.arctan2(rmat[1,0], rmat[0,0]))

                    x_axis, y_axis, z_axis = rmat[:, 0], rmat[:, 1], rmat[:, 2]

                    # ID별 z축으로 내릴 거리 설정 (Z-drop 오프셋)
                    if marker_id == 1: distance = -0.025
                    elif marker_id == 2: distance = -0.0265
                    elif marker_id == 3: distance = -0.050
                    else: distance = -0.010
                    
                    # Z-Drop 좌표 계산 (카메라 기준 좌표계)
                    z_dropped_point_3d = tvec.flatten() + (distance * z_axis)
                    # mm 단위로 변환하여 리스트화
                    px_z, py_z, pz_z = z_dropped_point_3d * 1000 
                    coords_mm = [round(px_z, 1), round(py_z, 1), round(pz_z, 1)]
                    
                    # 수직 방향 및 각도 판별
                    y_components = [abs(x_axis[1]), abs(y_axis[1]), abs(z_axis[1])]
                    up_axis_idx = np.argmax(y_components)
                    
                    # up_axis_idx=0(x축), 1(y축), 2(z축)       
                    if up_axis_idx == 0:
                        target_angle, angle_label = roll - 180.0, "ROLL"
                        face_dir = "SIDE (X-VERTICAL)"
                    elif up_axis_idx == 1:
                        target_angle, angle_label = pitch, "PITCH"
                        face_dir = "FRONT (Y-VERTICAL)"
                    else:
                        target_angle, angle_label = yaw, "YAW"
                        face_dir = "TOP (Z-VERTICAL)"

                    # 2D 시각화 좌표 계산 
                    pts_3d = np.array([[0, 0, distance]], dtype=np.float32)
                    pts_2d, _ = cv2.projectPoints(pts_3d, rvec, tvec, new_mtx, np.zeros(5))
                    proj_x, proj_y = int(pts_2d[0][0][0]), int(pts_2d[0][0][1])
                    
                    marker_corners = corners[i][0]
                    center_x, center_y = int(np.mean(marker_corners[:, 0])), int(np.mean(marker_corners[:, 1]))
                    base_x, base_y = int(marker_corners[0][0]), int(marker_corners[0][1])

                    # 화면 그리기 (Line, Circle, Text)
                    cv2.line(dst, (center_x, center_y), (proj_x, proj_y), (255, 255, 0), 2)
                    cv2.circle(dst, (proj_x, proj_y), 5, (0, 0, 255), -1)

                    cv2.putText(dst, f"ID:{marker_id} {face_dir}", (base_x, base_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                    cv2.putText(dst, f"MATCHED {angle_label}: {target_angle:.1f} deg", (base_x, base_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 2)
                    cv2.putText(dst, f"Z-Drop: {coords_mm[0]:.0f}, {coords_mm[1]:.0f}, {coords_mm[2]:.0f}mm", (base_x, base_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

                    # 리스트 형태로 데이터 저장: [ID, 각도, 좌표, 중심축]
                    current_frame_results.append([int(marker_id), round(target_angle, 2), coords_mm, up_axis_idx])

            cv2.imshow('Dynamic Axis Matching', dst)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('a'):
                if len(current_frame_results) > 0:
                    captured_list = current_frame_results[0] # 첫 번째 인식 데이터
                    break
                else:
                    print("인식된 마커가 없습니다.")
            elif key == ord('q'):
                break

    finally:
        
        cv2.destroyAllWindows()
        return captured_list # [ID, 각도, [X, Y, Z]] 반환
