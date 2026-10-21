from pathlib import Path
import time

import cv2
import numpy as np
from picamera2 import Picamera2


# =========================================================
# BSJ_outline 모듈화(초음파센서 추가예정)
# =========================================================

WIDTH = 1280
HEIGHT = 720

BACKGROUND_PATH = Path("background.png")

ROI_X1 = 392
ROI_Y1 = 0
ROI_X2 = 914
ROI_Y2 = 587

ROI_WIDTH = ROI_X2 - ROI_X1
ROI_HEIGHT = ROI_Y2 - ROI_Y1

MM_PER_PIXEL = 0.287331

DIFF_THRESHOLD = 25
MIN_AREA = 6000
MAX_AREA = 250000
MIN_FILL_RATIO = 0.45
MIN_RECT_WIDTH = 30
MIN_RECT_HEIGHT = 30
PRINT_INTERVAL = 0.3


def normalize_rect(rect):
    """Normalize minAreaRect to long-side width, short-side height, angle [-90, 90)."""
    (center_x, center_y), (width_px, height_px), angle = rect

    if width_px < height_px:
        width_px, height_px = height_px, width_px
        angle += 90.0

    if angle >= 90.0:
        angle -= 180.0

    if angle < -90.0:
        angle += 180.0

    return (
        float(center_x),
        float(center_y),
        float(width_px),
        float(height_px),
        float(angle),
    )


def load_background():
    """Load background.png and return the full image and its ROI crop."""
    background = cv2.imread(str(BACKGROUND_PATH))

    if background is None:
        raise RuntimeError(
            "background.png를 읽지 못했습니다. "
            "빈 컨베이어 상태에서 배경 사진을 먼저 저장하세요."
        )

    if background.shape[:2] != (HEIGHT, WIDTH):
        print(
            f"[WARNING] 배경 크기 변경: "
            f"{background.shape[1]}x{background.shape[0]} "
            f"-> {WIDTH}x{HEIGHT}"
        )
        background = cv2.resize(
            background, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR
        )

    background_roi = background[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2].copy()
    if background_roi.size == 0:
        raise RuntimeError("ROI 설정이 잘못되었습니다.")

    return background, background_roi


def select_target_contour(contours):
    """Select the largest contour passing the original box-shape filters."""
    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA or area > MAX_AREA:
            continue

        rect = cv2.minAreaRect(contour)
        (_, _), (rect_width, rect_height), _ = rect

        if rect_width <= 0 or rect_height <= 0:
            continue

        long_side = max(rect_width, rect_height)
        short_side = min(rect_width, rect_height)

        if long_side < MIN_RECT_WIDTH or short_side < MIN_RECT_HEIGHT:
            continue

        rect_area = rect_width * rect_height
        if rect_area <= 0:
            continue

        fill_ratio = area / rect_area
        if fill_ratio < MIN_FILL_RATIO:
            continue

        candidates.append({
            "contour": contour,
            "area": area,
            "fill_ratio": fill_ratio,
            "rect": rect,
        })

    if not candidates:
        return None

    return max(candidates, key=lambda item: item["area"])


def create_camera():
    """Create a camera for standalone testing. In the main project, pass in its existing camera."""
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        buffer_count=4,
    )
    picam2.configure(config)
    picam2.start()
    return picam2


def live_outline_detection(picam2):
    """
    Run live box-outline detection using an already-started Picamera2 object.

    Controls:
        a : accept the currently detected box and return its data
        b : recapture background while the ROI is empty
        s : save the current frame, ROI, mask, and detection preview
        q : cancel and return None

    Return on 'a':
        [
            angle_deg,
            [x_mm, y_mm, z_mm],
            [long_side_mm, short_side_mm]
        ]

    Coordinates:
        Origin = center of the ROI.
        +X = right in the image.
        +Y = down in the image.
        X/Y are planar offsets in mm, not robot-base coordinates.
        Z is temporarily fixed at 0.0 until the sensor is connected.
    """
    background_full, background_roi = load_background()
    background_blur = cv2.GaussianBlur(background_roi, (5, 5), 0)

    minimal_close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    last_print_time = 0.0
    latest_result = None
    latest_frame = None
    latest_roi = None
    latest_raw_mask = None
    latest_detection_view = None

    print("[Outline] 외곽선 검출 시작")
    print(f"[Outline] ROI: X={ROI_X1}~{ROI_X2}, Y={ROI_Y1}~{ROI_Y2}")
    print(f"[Outline] ROI center: ({ROI_WIDTH / 2:.1f}, {ROI_HEIGHT / 2:.1f})")
    print("[Outline] a: 좌표 확정 / b: 배경 재저장 / s: 영상 저장 / q: 취소")

    # Allow exposure/white balance to settle, without initializing a second camera.
    time.sleep(0.5)

    try:
        while True:
            frame_rgb = picam2.capture_array()
            if frame_rgb is None:
                continue

            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            if frame.shape[:2] != (HEIGHT, WIDTH):
                frame = cv2.resize(
                    frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR
                )

            roi = frame[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2].copy()
            detection_view = roi.copy()
            full_view = frame.copy()

            roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)
            difference = cv2.absdiff(background_blur, roi_blur)
            gray_difference = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)

            _, raw_mask = cv2.threshold(
                gray_difference, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY
            )

            minimal_corrected_mask = cv2.morphologyEx(
                raw_mask,
                cv2.MORPH_CLOSE,
                minimal_close_kernel,
                iterations=1,
            )

            contours, _ = cv2.findContours(
                raw_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            target = select_target_contour(contours)
            latest_result = None

            if target is not None:
                target_contour = target["contour"]
                rect = cv2.minAreaRect(target_contour)
                center_x, center_y, width_px, height_px, angle = normalize_rect(rect)

                width_mm = width_px * MM_PER_PIXEL
                height_mm = height_px * MM_PER_PIXEL

                # Preserve ROI-center coordinates.
                # Positive X points right; positive Y points down in the image.
                x_mm = (center_x - ROI_WIDTH / 2.0) * MM_PER_PIXEL
                y_mm = (center_y - ROI_HEIGHT / 2.0) * MM_PER_PIXEL

                # Z축 센서가 아직 연결되지 않아 Z 좌표 검출은 비활성화
                z_mm = 0.0
                pick_coords = [round(x_mm, 2), round(y_mm, 2), z_mm]
                box_size = [round(width_mm, 2), round(height_mm, 2)]
                latest_result = [round(angle, 2), pick_coords, box_size]

                box_points = np.int32(cv2.boxPoints(rect))

                cv2.drawContours(
                    detection_view, [target_contour], -1, (255, 0, 0), 2
                )
                cv2.drawContours(
                    detection_view, [box_points], -1, (0, 255, 0), 3
                )
                cv2.circle(
                    detection_view,
                    (int(round(center_x)), int(round(center_y))),
                    6,
                    (0, 0, 255),
                    -1,
                )

                global_box_points = box_points.copy()
                global_box_points[:, 0] += ROI_X1
                global_box_points[:, 1] += ROI_Y1
                cv2.drawContours(full_view, [global_box_points], -1, (0, 255, 0), 3)

                global_center_x = center_x + ROI_X1
                global_center_y = center_y + ROI_Y1
                cv2.circle(
                    full_view,
                    (int(round(global_center_x)), int(round(global_center_y))),
                    6,
                    (0, 0, 255),
                    -1,
                )

                text_lines = [
                    f"ROI center px: ({center_x:.1f}, {center_y:.1f})",
                    f"ROI-center mm: X={x_mm:.1f}, Y={y_mm:.1f}",
                    f"Size: {width_mm:.1f} x {height_mm:.1f} mm",
                    f"Angle: {angle:.1f} deg",
                    f"Area: {target['area']:.0f}, Fill: {target['fill_ratio']:.2f}",
                    "Press 'a' to accept",
                ]

                for index, line in enumerate(text_lines):
                    cv2.putText(
                        detection_view,
                        line,
                        (10, 30 + index * 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (0, 255, 0),
                        2,
                    )

                current_time = time.time()
                if current_time - last_print_time >= PRINT_INTERVAL:
                    print(
                        "[DETECTED] "
                        f"pick={pick_coords}, size={box_size} mm, "
                        f"angle={angle:.2f} deg"
                    )
                    last_print_time = current_time
            else:
                cv2.putText(
                    detection_view,
                    "Box not detected",
                    (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                )

            cv2.rectangle(
                full_view,
                (ROI_X1, ROI_Y1),
                (ROI_X2 - 1, ROI_Y2 - 1),
                (0, 255, 255),
                2,
            )
            cv2.putText(
                full_view,
                f"ROI {ROI_WIDTH} x {ROI_HEIGHT}",
                (ROI_X1 + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )

            cv2.imshow("Full Frame", full_view)
            cv2.imshow("Detection", detection_view)
            cv2.imshow("Difference", difference)
            cv2.imshow("Gray Difference", gray_difference)
            cv2.imshow("Raw Mask", raw_mask)
            cv2.imshow("Minimal Corrected Mask", minimal_corrected_mask)

            latest_frame = frame
            latest_roi = roi
            latest_raw_mask = raw_mask
            latest_detection_view = detection_view

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                return None

            if key == ord("a"):
                if latest_result is not None:
                    print(f"[Outline] 선택 결과: {latest_result}")
                    return latest_result
                print("[Outline] 박스가 검출되지 않았습니다.")

            elif key == ord("b"):
                success = cv2.imwrite(str(BACKGROUND_PATH), frame)
                if success:
                    background_full = frame.copy()
                    background_roi = background_full[
                        ROI_Y1:ROI_Y2, ROI_X1:ROI_X2
                    ].copy()
                    background_blur = cv2.GaussianBlur(
                        background_roi, (5, 5), 0
                    )
                    print("[BACKGROUND] background.png 재저장 완료")
                else:
                    print("[ERROR] background.png 저장 실패")

            elif key == ord("s"):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"frame_{timestamp}.png", latest_frame)
                cv2.imwrite(f"roi_{timestamp}.png", latest_roi)
                cv2.imwrite(f"raw_mask_{timestamp}.png", latest_raw_mask)
                cv2.imwrite(
                    f"detection_{timestamp}.png", latest_detection_view
                )
                print(f"[SAVE] 영상 저장 완료: {timestamp}")

    finally:
        cv2.destroyAllWindows()


def main():
    """Standalone test entry point. The main project should call live_outline_detection(picam2)."""
    picam2 = create_camera()
    try:
        result = live_outline_detection(picam2)
        print("Result:", result)
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()
