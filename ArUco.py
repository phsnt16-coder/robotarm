import cv2
import numpy as np
import os
import pickle
from picamera2 import Picamera2

# 아루코 마커로 파지좌표 반환
def live_aruco_detection(calibration_data):
    camera_matrix = calibration_data['camera_matrix']
    dist_coeffs = calibration_data['dist_coeffs']
    # cv2 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    marker_size = 0.05 #50*50mm 마커 사용

    # 마커 중앙을 원점으로 설정
    obj_points = np.array([
        [-marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()

    # return할 데이터
    captured_data = None

    try:
        print("프로그램 실행 중:'a' 키를 누르면 좌표를 반환하고 종료. 'q'는 취소 및 종료.")
        while True: # 스트리밍 실행
            frame_rgb = picam2.capture_array()
            if frame_rgb is None: continue
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            h, w = frame.shape[:2]
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 0, (w,h))
            dst = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_mtx)

            corners, ids, _ = detector.detectMarkers(dst)

            current_frame_results = [] # 현재 화면에서 발견된 데이터 저장용

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(dst, corners, ids)
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    # 위치/방향 벡터값 저장
                    _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], new_mtx, np.zeros(5))
                    cv2.drawFrameAxes(dst, new_mtx, np.zeros(5), rvec, tvec, marker_size/2)
                    # 방향벡터를 행렬로 변환 
                    rmat, _ = cv2.Rodrigues(rvec)
                    
                    # 1. 각도 계산 및 축 정의
                    sy = np.sqrt(rmat[0,0]**2 + rmat[1,0]**2)
                    roll = np.degrees(np.arctan2(rmat[2,1] , rmat[2,2]))
                    pitch = np.degrees(np.arctan2(-rmat[2,0], sy))
                    yaw = np.degrees(np.arctan2(rmat[1,0], rmat[0,0]))

                    x_axis, y_axis, z_axis = rmat[:, 0], rmat[:, 1], rmat[:, 2]

                    # 2. ID별 z축으로 내릴 거리 설정
                    if marker_id == 1: distance = 0.025
                    elif marker_id == 2: distance = 0.0265
                    elif marker_id == 3: distance = 0.050
                    else: distance = 0.010
                    
                    # 3. Z-Drop 좌표 계산 (카메라 기준 좌표계)
                    z_dropped_point_3d = tvec.flatten() + (distance * z_axis)
                    px_z, py_z, pz_z = z_dropped_point_3d * 1000 # mm 단위
                    
                    # 4. 수직 방향 및 각도 판별
                    y_components = [abs(x_axis[1]), abs(y_axis[1]), abs(z_axis[1])]
                    up_axis_idx = np.argmax(y_components)

                    if up_axis_idx == 0:
                        target_angle, angle_label = roll + 180.0, "ROLL"
                        face_dir = "SIDE (X-VERTICAL)"
                    elif up_axis_idx == 1:
                        target_angle, angle_label = pitch, "PITCH"
                        face_dir = "FRONT (Y-VERTICAL)"
                    else:
                        target_angle, angle_label = yaw, "YAW"
                        face_dir = "TOP (Z-VERTICAL)"

                    # 5. 시각화 좌표
                    pts_3d = np.array([[0, 0, distance]], dtype=np.float32)
                    pts_2d, _ = cv2.projectPoints(pts_3d, rvec, tvec, new_mtx, np.zeros(5))
                    proj_x, proj_y = int(pts_2d[0][0][0]), int(pts_2d[0][0][1])
                    
                    marker_corners = corners[i][0]
                    center_x, center_y = int(np.mean(marker_corners[:, 0])), int(np.mean(marker_corners[:, 1]))
                    base_x, base_y = int(marker_corners[0][0]), int(marker_corners[0][1])

                    cv2.line(dst, (center_x, center_y), (proj_x, proj_y), (255, 255, 0), 2)
                    cv2.circle(dst, (proj_x, proj_y), 5, (0, 0, 255), -1)

                    # 화면 텍스트 표시
                    cv2.putText(dst, f"ID:{marker_id} {face_dir}", (base_x, base_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                    cv2.putText(dst, f"MATCHED {angle_label}: {target_angle:.1f} deg", (base_x, base_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 2)
                    cv2.putText(dst, f"Z-Drop: {px_z:.0f}, {py_z:.0f}, {pz_z:.0f}mm", (base_x, base_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

                    # 현재 인식된 데이터를 임시 저장
                    current_frame_results.append({
                        'id': marker_id,
                        'angle': target_angle,
                        'z_drop_mm': [px_z, py_z, pz_z]
                    })

            cv2.imshow('Dynamic Axis Matching', dst)
            
            key = cv2.waitKey(1) & 0xFF
            # 'a' 키를 누르면 가장 먼저 인식된 마커 데이터 리턴
            if key == ord('a'):
                if len(current_frame_results) > 0:
                    captured_data = current_frame_results[0] # 첫 번째 마커 데이터 선택
                    print("\n[DATA CAPTURED]")
                    print(f"Marker ID: {captured_data['id']}")
                    print(f"Matched Angle: {captured_data['angle']:.2f} deg")
                    print(f"Z-Drop (X, Y, Z): {captured_data['z_drop_mm']} mm")
                    break
                else:
                    print("인식된 마커가 없습니다.")
            
            elif key == ord('q'):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        return captured_data # 최종 데이터 반환 (없으면 None)

def main():
    pkl_path = 'camera_calibration.pkl'
    if os.path.exists(pkl_path):
        with open(pkl_path, 'rb') as f:
            data = pickle.load(f)
        
        # 함수로부터 리턴값을 받음
        result = live_aruco_detection(data)
        
        if result:
            print("\n--- Final Return Values ---")
            print(f"Angle: {result['angle']}")
            print(f"Coordinates: {result['z_drop_mm']}")
    else:
        print("캘리브레이션 파일이 없습니다.")

if __name__ == "__main__":
    main()
