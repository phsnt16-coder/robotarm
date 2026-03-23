#copywrite by 로보고니
import cv2
import datetime
import os
from picamera2 import Picamera2

# 1. 저장할 폴더가 없으면 생성
save_dir = "./checkerboards"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
    print(f"'{save_dir}' 폴더를 생성했습니다.")

# 2. Picamera2 객체 생성 및 설정
picam2 = Picamera2()
# 보정용 이미지는 왜곡이 없어야 하므로 적절한 해상도로 설정합니다.
config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

print("체커보드 캡처 시스템 가동")
print("- 'a' 키: 현재 화면 저장")
print("- 'q' 키: 종료")

try:
    while True:
        # 카메라로부터 프레임 읽기 (NumPy 배열)
        frame_rgb = picam2.capture_array()

        # [중요] 화면 표시 및 저장을 위해 RGB를 BGR로 변환 (색상 보정)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # 화면에 표시
        cv2.imshow("Calibration Capture", frame_bgr)

        # 키 입력 대기
        key = cv2.waitKey(1) & 0xFF

        # 'a' 키를 누르면 저장
        if key == ord('a'):
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{save_dir}/capture_{timestamp}.png"
            
            # BGR 상태의 프레임을 저장
            cv2.imwrite(filename, frame_bgr)
            print(f"저장 완료: {filename}")

        # 'q' 키를 누르면 종료
        elif key == ord('q'):
            break

except Exception as e:
    print(f"오류 발생: {e}")

finally:
    # 자원 해제
    picam2.stop()
    cv2.destroyAllWindows()
    print("프로그램을 종료합니다.")
