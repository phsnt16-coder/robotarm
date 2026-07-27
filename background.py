from picamera2 import Picamera2
import cv2
import time


WIDTH = 1280
HEIGHT = 720

ROI_X1 = 392
ROI_Y1 = 0
ROI_X2 = 914
ROI_Y2 = 587


def main() -> None:
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888",
        }
    )

    picam2.configure(config)
    picam2.start()

    try:
        # 자동 노출과 화이트밸런스가 안정될 시간
        time.sleep(3)

        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            display = frame.copy()

            cv2.rectangle(
                display,
                (ROI_X1, ROI_Y1),
                (ROI_X2 - 1, ROI_Y2 - 1),
                (0, 255, 0),
                3
            )

            cv2.putText(
                display,
                "S: Save Background   Q: Quit",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.imshow("Background Capture", display)
            cv2.imshow(
                "ROI Preview",
                frame[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                success = cv2.imwrite("background.png", frame)

                if success:
                    print("background.png 저장 완료")
                    print(f"크기: {frame.shape[1]} x {frame.shape[0]}")
                else:
                    print("background.png 저장 실패")

            elif key == ord("q"):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()