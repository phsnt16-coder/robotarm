from pathlib import Path
import json

import cv2
from picamera2 import Picamera2


WIDTH = 1280
HEIGHT = 720

ROI_CONFIG_PATH = Path("roi_config.json")


def nothing(value: int) -> None:
    pass


def load_roi() -> tuple[int, int, int, int]:
    """기존 ROI 설정이 있으면 불러오고, 없으면 기본값을 반환합니다."""
    default_roi = (220, 60, 1060, 620)

    if not ROI_CONFIG_PATH.exists():
        return default_roi

    try:
        with ROI_CONFIG_PATH.open("r", encoding="utf-8") as file:
            data = json.load(file)

        return (
            int(data["x1"]),
            int(data["y1"]),
            int(data["x2"]),
            int(data["y2"]),
        )

    except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError):
        print("기존 ROI 설정을 읽지 못해 기본값을 사용합니다.")
        return default_roi


def save_roi(x1: int, y1: int, x2: int, y2: int) -> None:
    data = {
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
    }

    with ROI_CONFIG_PATH.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=4)

    print("\nROI 저장 완료")
    print(f"ROI_X1 = {x1}")
    print(f"ROI_Y1 = {y1}")
    print(f"ROI_X2 = {x2}")
    print(f"ROI_Y2 = {y2}")
    print(f"ROI 크기 = {x2 - x1} × {y2 - y1} px")
    print(f"저장 경로 = {ROI_CONFIG_PATH.resolve()}")


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

    window_name = "ROI Adjustment"
    roi_window_name = "Selected ROI"

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)

    x1, y1, x2, y2 = load_roi()

    cv2.createTrackbar("X1", window_name, x1, WIDTH - 1, nothing)
    cv2.createTrackbar("Y1", window_name, y1, HEIGHT - 1, nothing)
    cv2.createTrackbar("X2", window_name, x2, WIDTH, nothing)
    cv2.createTrackbar("Y2", window_name, y2, HEIGHT, nothing)

    print("트랙바를 움직여 ROI를 조정하세요.")
    print("s: ROI 좌표 저장")
    print("q: 종료")

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            x1 = cv2.getTrackbarPos("X1", window_name)
            y1 = cv2.getTrackbarPos("Y1", window_name)
            x2 = cv2.getTrackbarPos("X2", window_name)
            y2 = cv2.getTrackbarPos("Y2", window_name)

            # 잘못된 좌표가 입력되지 않도록 보정
            x1 = max(0, min(x1, WIDTH - 2))
            y1 = max(0, min(y1, HEIGHT - 2))

            x2 = max(x1 + 1, min(x2, WIDTH))
            y2 = max(y1 + 1, min(y2, HEIGHT))

            display = frame.copy()

            # ROI 외부를 어둡게 표시
            darkened = (display * 0.30).astype("uint8")
            darkened[y1:y2, x1:x2] = display[y1:y2, x1:x2]
            display = darkened

            # ROI 테두리
            cv2.rectangle(
                display,
                (x1, y1),
                (x2 - 1, y2 - 1),
                (0, 255, 0),
                3,
            )

            roi_width = x2 - x1
            roi_height = y2 - y1

            cv2.putText(
                display,
                f"ROI: ({x1}, {y1}) - ({x2}, {y2})",
                (25, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                display,
                f"Size: {roi_width} x {roi_height} px",
                (25, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                display,
                "S: Save ROI   Q: Quit",
                (25, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            roi = frame[y1:y2, x1:x2]

            cv2.imshow(window_name, display)
            cv2.imshow(roi_window_name, roi)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                save_roi(x1, y1, x2, y2)

            elif key == ord("q"):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()