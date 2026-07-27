from pathlib import Path
import time

import cv2
import numpy as np
from picamera2 import Picamera2


# =========================================================
# 카메라 설정
# =========================================================

WIDTH = 1280
HEIGHT = 720

BACKGROUND_PATH = Path("background.png")


# =========================================================
# 확정 ROI
# =========================================================

ROI_X1 = 392
ROI_Y1 = 0
ROI_X2 = 914
ROI_Y2 = 587

ROI_WIDTH = ROI_X2 - ROI_X1
ROI_HEIGHT = ROI_Y2 - ROI_Y1


# =========================================================
# 픽셀 → mm 변환값
# =========================================================

MM_PER_PIXEL = 0.287331


# =========================================================
# 검출 설정
# =========================================================

# 차영상 이진화 기준값
# 박스가 잘 안 보이면 낮추고, 노이즈가 많으면 높이세요.
DIFF_THRESHOLD = 25

# ROI 내 검출 대상 면적 범위
MIN_AREA = 6000
MAX_AREA = 250000

# contour가 사각형 영역을 얼마나 채우는지 판단
MIN_FILL_RATIO = 0.45

# 박스 가로·세로 크기 제한
MIN_RECT_WIDTH = 30
MIN_RECT_HEIGHT = 30

# 검출 결과 출력 주기
PRINT_INTERVAL = 0.3


def normalize_rect(rect):
    """
    cv2.minAreaRect() 결과를 정규화합니다.

    반환값:
        center_x
        center_y
        width_px  : 긴 변
        height_px : 짧은 변
        angle     : -90 ~ 90도
    """

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
    """
    background.png를 불러오고 카메라 해상도에 맞춥니다.
    """

    background = cv2.imread(str(BACKGROUND_PATH))

    if background is None:
        raise RuntimeError(
            "background.png를 읽지 못했습니다.\n"
            "빈 컨베이어 상태에서 배경 사진을 먼저 저장하세요."
        )

    if background.shape[:2] != (HEIGHT, WIDTH):
        print(
            f"[WARNING] 배경 크기 변경: "
            f"{background.shape[1]}x{background.shape[0]} "
            f"→ {WIDTH}x{HEIGHT}"
        )

        background = cv2.resize(
            background,
            (WIDTH, HEIGHT),
            interpolation=cv2.INTER_LINEAR,
        )

    background_roi = background[
        ROI_Y1:ROI_Y2,
        ROI_X1:ROI_X2
    ].copy()

    if background_roi.size == 0:
        raise RuntimeError("ROI 설정이 잘못되었습니다.")

    return background, background_roi


def create_camera():
    """
    Picamera2를 초기화합니다.
    """

    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888",
        },
        buffer_count=4,
    )

    picam2.configure(config)
    picam2.start()

    return picam2


def select_target_contour(contours):
    """
    contour 후보 중 박스일 가능성이 가장 높은 contour를 선택합니다.
    """

    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < MIN_AREA:
            continue

        if area > MAX_AREA:
            continue

        rect = cv2.minAreaRect(contour)
        (_, _), (rect_width, rect_height), _ = rect

        if rect_width <= 0 or rect_height <= 0:
            continue

        long_side = max(rect_width, rect_height)
        short_side = min(rect_width, rect_height)

        if long_side < MIN_RECT_WIDTH:
            continue

        if short_side < MIN_RECT_HEIGHT:
            continue

        rect_area = rect_width * rect_height

        if rect_area <= 0:
            continue

        fill_ratio = area / rect_area

        if fill_ratio < MIN_FILL_RATIO:
            continue

        candidates.append(
            {
                "contour": contour,
                "area": area,
                "fill_ratio": fill_ratio,
                "rect": rect,
            }
        )

    if not candidates:
        return None

    # 가장 큰 contour를 박스로 판단
    return max(
        candidates,
        key=lambda item: item["area"],
    )


def main():
    print("========================================")
    print("배경 차감 기반 박스 외곽선 검출")
    print("========================================")
    print(f"카메라 크기: {WIDTH} x {HEIGHT}")
    print(
        f"ROI: X={ROI_X1}~{ROI_X2}, "
        f"Y={ROI_Y1}~{ROI_Y2}"
    )
    print(f"ROI 크기: {ROI_WIDTH} x {ROI_HEIGHT}")
    print(f"MM_PER_PIXEL: {MM_PER_PIXEL}")
    print("종료: q")
    print("배경 재저장: b")
    print("현재 프레임 저장: s")
    print("========================================")

    background_full, background_roi = load_background()

    # 차영상 계산 전 작은 영상 노이즈만 완화
    background_blur = cv2.GaussianBlur(
        background_roi,
        (5, 5),
        0,
    )

    picam2 = create_camera()

    # 자동 노출 및 화이트밸런스 안정화 대기
    time.sleep(2)

    # Raw Mask가 정확하므로 강한 morphology는 사용하지 않음
    minimal_close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (3, 3),
    )

    last_print_time = 0.0

    try:
        while True:
            frame_rgb = picam2.capture_array()

            frame = cv2.cvtColor(
                frame_rgb,
                cv2.COLOR_RGB2BGR,
            )

            if frame.shape[:2] != (HEIGHT, WIDTH):
                frame = cv2.resize(
                    frame,
                    (WIDTH, HEIGHT),
                    interpolation=cv2.INTER_LINEAR,
                )

            roi = frame[
                ROI_Y1:ROI_Y2,
                ROI_X1:ROI_X2
            ].copy()

            detection_view = roi.copy()
            full_view = frame.copy()

            roi_blur = cv2.GaussianBlur(
                roi,
                (5, 5),
                0,
            )

            # =================================================
            # 1. 배경과 현재 ROI의 차이 계산
            # =================================================

            difference = cv2.absdiff(
                background_blur,
                roi_blur,
            )

            gray_difference = cv2.cvtColor(
                difference,
                cv2.COLOR_BGR2GRAY,
            )

            # =================================================
            # 2. Raw Mask 생성
            # =================================================

            _, raw_mask = cv2.threshold(
                gray_difference,
                DIFF_THRESHOLD,
                255,
                cv2.THRESH_BINARY,
            )

            # =================================================
            # 3. 최소한의 보정
            # =================================================
            # Raw Mask가 이미 정확하므로 3x3 CLOSE 1회만 적용
            # 실제 contour 검출은 raw_mask를 기준으로 실행

            minimal_corrected_mask = cv2.morphologyEx(
                raw_mask,
                cv2.MORPH_CLOSE,
                minimal_close_kernel,
                iterations=1,
            )

            # =================================================
            # 4. Raw Mask에서 외곽선 검출
            # =================================================

            contours, _ = cv2.findContours(
                raw_mask.copy(),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            target = select_target_contour(contours)

            if target is not None:
                target_contour = target["contour"]
                contour_area = target["area"]
                fill_ratio = target["fill_ratio"]

                # ConvexHull은 사용하지 않음
                # Raw Mask contour를 바로 minAreaRect에 사용
                rect = cv2.minAreaRect(target_contour)

                (
                    center_x,
                    center_y,
                    width_px,
                    height_px,
                    angle,
                ) = normalize_rect(rect)

                width_mm = width_px * MM_PER_PIXEL
                height_mm = height_px * MM_PER_PIXEL

                # ROI 내부 좌표
                roi_center_x = center_x
                roi_center_y = center_y

                # 전체 카메라 좌표
                global_center_x = center_x + ROI_X1
                global_center_y = center_y + ROI_Y1

                # ROI 왼쪽 위를 원점으로 한 물리 좌표
                roi_x_mm = center_x * MM_PER_PIXEL
                roi_y_mm = center_y * MM_PER_PIXEL

                box_points = cv2.boxPoints(rect)
                box_points = np.int32(box_points)

                # 원본 contour
                cv2.drawContours(
                    detection_view,
                    [target_contour],
                    -1,
                    (255, 0, 0),
                    2,
                )

                # 최종 회전 사각형
                cv2.drawContours(
                    detection_view,
                    [box_points],
                    -1,
                    (0, 255, 0),
                    3,
                )

                # 중심점
                cv2.circle(
                    detection_view,
                    (
                        int(round(center_x)),
                        int(round(center_y)),
                    ),
                    6,
                    (0, 0, 255),
                    -1,
                )

                # 전체 화면에도 검출 결과 표시
                global_box_points = box_points.copy()
                global_box_points[:, 0] += ROI_X1
                global_box_points[:, 1] += ROI_Y1

                cv2.drawContours(
                    full_view,
                    [global_box_points],
                    -1,
                    (0, 255, 0),
                    3,
                )

                cv2.circle(
                    full_view,
                    (
                        int(round(global_center_x)),
                        int(round(global_center_y)),
                    ),
                    6,
                    (0, 0, 255),
                    -1,
                )

                # 검출 정보 출력
                text_lines = [
                    (
                        f"ROI center: "
                        f"({roi_center_x:.1f}, {roi_center_y:.1f})"
                    ),
                    (
                        f"Global center: "
                        f"({global_center_x:.1f}, "
                        f"{global_center_y:.1f})"
                    ),
                    (
                        f"Size: "
                        f"{width_mm:.1f} x "
                        f"{height_mm:.1f} mm"
                    ),
                    f"Angle: {angle:.1f} deg",
                    (
                        f"Area: {contour_area:.0f}, "
                        f"Fill: {fill_ratio:.2f}"
                    ),
                ]

                for index, text in enumerate(text_lines):
                    cv2.putText(
                        detection_view,
                        text,
                        (10, 30 + index * 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (0, 255, 0),
                        2,
                    )

                current_time = time.time()

                if current_time - last_print_time >= PRINT_INTERVAL:
                    print(
                        "[DETECTED] "
                        f"ROI center=({roi_center_x:.1f}, "
                        f"{roi_center_y:.1f}), "
                        f"Global center=({global_center_x:.1f}, "
                        f"{global_center_y:.1f}), "
                        f"ROI mm=({roi_x_mm:.1f}, "
                        f"{roi_y_mm:.1f}), "
                        f"Size=({width_mm:.1f}, "
                        f"{height_mm:.1f}) mm, "
                        f"Angle={angle:.1f}, "
                        f"Area={contour_area:.0f}, "
                        f"Fill={fill_ratio:.2f}"
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

            # 전체 화면 ROI 표시
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

            # =================================================
            # 5. 화면 표시
            # =================================================

            cv2.imshow("Full Frame", full_view)
            cv2.imshow("Detection", detection_view)
            cv2.imshow("Difference", difference)
            cv2.imshow("Gray Difference", gray_difference)
            cv2.imshow("Raw Mask", raw_mask)
            cv2.imshow(
                "Minimal Corrected Mask",
                minimal_corrected_mask,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            # 빈 컨베이어 상태에서 b를 누르면 배경 재저장
            elif key == ord("b"):
                success = cv2.imwrite(
                    str(BACKGROUND_PATH),
                    frame,
                )

                if success:
                    background_full = frame.copy()

                    background_roi = background_full[
                        ROI_Y1:ROI_Y2,
                        ROI_X1:ROI_X2
                    ].copy()

                    background_blur = cv2.GaussianBlur(
                        background_roi,
                        (5, 5),
                        0,
                    )

                    print(
                        "[BACKGROUND] "
                        "background.png 재저장 완료"
                    )
                else:
                    print(
                        "[ERROR] "
                        "background.png 저장 실패"
                    )

            # 현재 검출 화면과 마스크 저장
            elif key == ord("s"):
                timestamp = time.strftime(
                    "%Y%m%d_%H%M%S"
                )

                cv2.imwrite(
                    f"frame_{timestamp}.png",
                    frame,
                )

                cv2.imwrite(
                    f"roi_{timestamp}.png",
                    roi,
                )

                cv2.imwrite(
                    f"raw_mask_{timestamp}.png",
                    raw_mask,
                )

                cv2.imwrite(
                    f"detection_{timestamp}.png",
                    detection_view,
                )

                print(
                    f"[SAVE] 영상 저장 완료: {timestamp}"
                )

    except KeyboardInterrupt:
        print("\n[INFO] 키보드 입력으로 종료합니다.")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print("[INFO] 카메라 및 OpenCV 창 종료 완료")


if __name__ == "__main__":
    main()