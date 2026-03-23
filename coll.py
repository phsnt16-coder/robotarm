import cv2
import numpy as np
import os
import glob
import pickle
from picamera2 import Picamera2


def calibrate_camera():
    # 체커보드의 차원 정의
    CHECKERBOARD = (9,13)  # 체커보드 행과 열당 내부 코너 수
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # 각 체커보드 이미지에 대한 3D 점 벡터를 저장할 벡터 생성
    objpoints = []
    # 각 체커보드 이미지에 대한 2D 점 벡터를 저장할 벡터 생성
    imgpoints = [] 
    
    # 3D 점의 세계 좌표 정의
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    # 주어진 디렉터리에 저장된 개별 이미지의 경로 추출
    images = glob.glob('./checkerboards/*.png')
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray,
                                               CHECKERBOARD,
                                               cv2.CALIB_CB_ADAPTIVE_THRESH +
                                               cv2.CALIB_CB_FAST_CHECK +
                                               cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            
            # 코너 그리기 및 표시
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(0)
    
    cv2.destroyAllWindows()
    
    # 카메라 캘리브레이션 수행
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
                                                      gray.shape[::-1], None, None)
    
    # 결과 출력
    print("Camera matrix : \n")
    print(mtx)
    print("\ndist : \n")
    print(dist)
    print("\nrvecs : \n")
    print(rvecs)
    print("\ntvecs : \n")
    print(tvecs)
    
    # 캘리브레이션 결과를 파일로 저장
    calibration_data = {
        'camera_matrix': mtx,
        'dist_coeffs': dist,
        'rvecs': rvecs,
        'tvecs': tvecs
    }
    
    with open('camera_calibration.pkl', 'wb') as f:
        pickle.dump(calibration_data, f)
    
    return calibration_data

def live_video_correction(calibration_data):
    # 1. 캘리브레이션 데이터 추출
    mtx = calibration_data['camera_matrix']
    dist = calibration_data['dist_coeffs']
    
    # 2. Picamera2 설정 (라즈베리 파이 전용)
    picam2 = Picamera2()
    # 캘리브레이션 사진 찍을 때와 동일한 해상도와 화각 모드를 설정해야 합니다.
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    config["sensor_mode"] = 4  # 전체 화각 모드 강제 (사진 찍을 때 썼던 모드)
    picam2.configure(config)
    picam2.start()
    
    print("실시간 왜곡 보정 중... 'q'를 누르면 종료됩니다.")
    
    try:
        while True:
            # 프레임 읽기 (RGB -> BGR 변환)
            frame_rgb = picam2.capture_array()
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            h, w = frame.shape[:2]
            
            # 3. 최적의 카메라 행렬 계산
            # alpha=0: 모든 검은색 여백을 제거하고 유효한 픽셀만 남깁니다.
            # alpha=1: 왜곡 보정 시 소실되는 픽셀 없이 모든 픽셀을 유지합니다 (검은 여백 생김).
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
            
            # 4. 왜곡 보정 (Undistort)
            dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            
            # 5. ROI 자르기 (보정 후 생기는 불필요한 테두리 제거)
            x, y, w_roi, h_roi = roi
            if all(v >= 0 for v in [x, y, w_roi, h_roi]) and w_roi > 0 and h_roi > 0:
                dst = dst[y:y+h_roi, x:x+w_roi]
            
            # 6. 화면 비교를 위해 리사이즈 및 결합
            # 보정 후 잘려나간 dst를 다시 원본 크기에 맞춰야 비교가 쉽습니다.
            dst_resized = cv2.resize(dst, (640, 480))
            combined = np.hstack((frame, dst_resized))
            
            cv2.imshow('Original (Left) | Corrected (Right)', combined)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # 자원 해제
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 이미 캘리브레이션 파일이 있는지 확인
    if os.path.exists('camera_calibration.pkl'):
        print("Loading existing calibration data...")
        with open('camera_calibration.pkl', 'rb') as f:
            calibration_data = pickle.load(f)
    else:
        print("Performing new camera calibration...")
        calibration_data = calibrate_camera()
    
    # 실시간 비디오 보정 실행
    print("Starting live video correction...")
    live_video_correction(calibration_data)