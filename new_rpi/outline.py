from pathlib import Path
import time

import cv2
import numpy as np
from picamera2 import Picamera2


# =========================================================
# 카메라 설정
# =========================================================

WIDTH = 3280
HEIGHT = 2464

BACKGROUND_PATH = Path("background.png")


# =========================================================
# 확정 ROI
# =========================================================

ROI_X1 = 982
ROI_Y1 = 515
ROI_X2 = 2009
ROI_Y2 = 1728

ROI_WIDTH = ROI_X2 - ROI_X1
ROI_HEIGHT = ROI_Y2 - ROI_Y1


# =========================================================
# 픽셀 → mm 변환값
# =========================================================

MM_PER_PIXEL = 0.159489



# =========================================================
# VL53L8CX 거리센서 (Raspberry Pi I2C-1: GPIO2 SDA, GPIO3 SCL)
# =========================================================
# VL53L8CX는 펌웨어 업로드가 필요한 ToF 센서입니다.
# 사용 패키지: circuitpython-vl53lxcx (import vl53lxcx) 및 adafruit-blinka.
# LPn은 모듈에서 HIGH 상태로 유지된다고 가정합니다.

DISTANCE_ENABLED = True
CONVEYOR_DISTANCE_MM = 300.0  # 센서에서 빈 컨베이어 표면까지의 기준 거리(mm)
DISTANCE_POLL_INTERVAL = 0.2  # 초


def fix_firmware_lookup():
    """Repair vl53lxcx firmware discovery for absolute firmware paths.

    Patch applies only within this Python process; installed package is unchanged.
    """
    import sys
    import vl53lxcx

    if getattr(vl53lxcx._find_file, "_outline_fw_fixed", False):
        return

    original_find_file = vl53lxcx._find_file
    # 예: /home/mbp/.local/lib/python3.11/site-packages/vl53lxcx
    # Python 버전에 맞는 실제 설치 경로를 자동으로 찾습니다.
    package_dir = Path(vl53lxcx.__file__).resolve().parent

    def fixed_find_file(name, req_size):
        requested = Path(name)
        candidates = []
        if requested.is_absolute():
            candidates.append(requested)
        candidates.append(package_dir / requested.name)
        if not requested.is_absolute():
            candidates.extend(
                Path(directory) / requested
                for directory in sys.path if directory
            )

        for candidate in candidates:
            if not candidate.is_file():
                continue
            actual_size = candidate.stat().st_size
            if actual_size != req_size:
                raise ValueError(
                    f"firmware file size incorrect: {actual_size} != {req_size}"
                )
            print(f"[TOF FW] 펌웨어 확인: {candidate.resolve()}")
            return str(candidate)

        return original_find_file(name, req_size)

    fixed_find_file._outline_fw_fixed = True
    vl53lxcx._find_file = fixed_find_file


class VL53L8CXDistanceSensor:
    """Installed circuitpython-vl53lxcx driver adapter (4x4 center zones).

    Assumes the sensor LPn pin is already held HIGH by the module/wiring.
    """

    def __init__(self):
        try:
            import board
            import busio
            from vl53lxcx import (
                VL53L8CX,
                RESOLUTION_4X4,
                DATA_DISTANCE_MM,
                DATA_TARGET_STATUS,
                STATUS_VALID,
            )
        except ImportError as exc:
            raise RuntimeError(
                "circuitpython-vl53lxcx 또는 adafruit-blinka import 실패: "
                f"{exc}"
            ) from exc

        self.sensor = None
        self.i2c = None
        self.valid_status = STATUS_VALID
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self.sensor = VL53L8CX(self.i2c)
            if not self.sensor.is_alive():
                raise RuntimeError(
                    "VL53L8CX가 I2C(기본 0x29)에서 응답하지 않습니다. "
                    "배선, I2C 설정 및 LPn HIGH 상태를 확인하세요."
                )
            # Work around broken absolute-path handling in vl53lxcx._find_file.
            fix_firmware_lookup()
            self.sensor.init()
            self.sensor.resolution = RESOLUTION_4X4
            self.sensor.ranging_freq = 5
            self.sensor.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})
        except Exception:
            self.close()
            raise

    def read_mm(self):
        """4x4 중앙 4개 zone의 유효 거리 중앙값(mm); 데이터 없으면 None."""
        if not self.sensor.check_data_ready():
            return None
        data = self.sensor.get_ranging_data()
        distances = data.distance_mm
        statuses = data.target_status
        center_indices = (5, 6, 9, 10)
        valid = [
            float(distances[i]) for i in center_indices
            if statuses[i] in (self.valid_status, 9) and distances[i] > 0
        ]
        return float(np.median(valid)) if valid else None

    def close(self):
        if self.sensor is not None:
            try:
                self.sensor.stop_ranging()
            except Exception:
                pass
        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except Exception:
                pass


def calculate_box_height(distance_mm):
    """센서-상자 표면 거리에서 상자 높이(mm)를 계산합니다."""
    if distance_mm is None:
        return None
    return max(0.0, CONVEYOR_DISTANCE_MM - float(distance_mm))


# =========================================================
# 검출 설정
# =========================================================

# 차영상 이진화 기준값
# 박스가 잘 안 보이면 낮추고, 노이즈가 많으면 높이세요.
DIFF_THRESHOLD = 25

# ROI 내 검출 대상 면적 범위
MIN_AREA = 6000
MAX_AREA = 1000000  # 8MP ROI: 기존 250000은 실제 상자(약 732459 px²)를 제외함

# contour가 사각형 영역을 얼마나 채우는지 판단
MIN_FILL_RATIO = 0.65

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
    """Filter ROI-border contours and prioritize the largest valid object."""
    candidates = []
    edge_margin = 15  # pixels within ROI boundary treated as background artifacts

    for contour in contours:
        area = cv2.contourArea(contour)
        if not (MIN_AREA <= area <= MAX_AREA):
            continue

        # Ignore contours touching the ROI boundary (e.g. conveyor edges).
        x, y, w, h = cv2.boundingRect(contour)
        if (
            x <= edge_margin
            or y <= edge_margin
            or x + w >= ROI_WIDTH - edge_margin
            or y + h >= ROI_HEIGHT - edge_margin
        ):
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

    # Prioritize the largest valid contour; use fill ratio only for equal areas.
    return max(candidates, key=lambda item: (item["area"], item["fill_ratio"]))


def live_outline_detection(picam2):
    """기존 카메라를 사용해 a 키 확정 시 (angle, pick_xyz, [w,d], h)를 반환. q는 None."""
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
    print(f"빈 컨베이어 기준 거리: {CONVEYOR_DISTANCE_MM:.1f} mm")
    print("확정: a / 종료: q")
    print("배경 재저장: b")
    print("현재 프레임 저장: s")
    print("진단 사진 저장: c (원본/ROI/Raw Mask/검출 결과)")
    print("========================================")

    background_full, background_roi = load_background()

    # 차영상 계산 전 작은 영상 노이즈만 완화
    background_blur = cv2.GaussianBlur(
        background_roi,
        (5, 5),
        0,
    )

    # 카메라는 main.py에서 초기화하여 전달받음 (중복 초기화 방지)

    # 자동 노출 및 화이트밸런스 안정화 대기
    time.sleep(2)

    # Raw Mask가 정확하므로 강한 morphology는 사용하지 않음
    minimal_close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (3, 3),
    )

    last_print_time = 0.0
    last_distance_poll = 0.0
    distance_mm = None
    distance_sensor = None
    latest_result = None
    if DISTANCE_ENABLED:
        try:
            distance_sensor = VL53L8CXDistanceSensor()
            print("[TOF] VL53L8CX 초기화 완료 (I2C-1)")
        except Exception as exc:
            print(f"[TOF ERROR] 거리센서 초기화 실패: {exc}")
            print("[TOF] 외곽선 검출은 계속 진행합니다.")



    try:
        while True:
            if distance_sensor is not None and time.monotonic() - last_distance_poll >= DISTANCE_POLL_INTERVAL:
                last_distance_poll = time.monotonic()
                try:
                    new_distance = distance_sensor.read_mm()
                    if new_distance is not None:
                        distance_mm = new_distance
                except Exception as exc:
                    print(f"[TOF ERROR] 거리 측정 실패: {exc}")
                    distance_mm = None

            box_height_mm = calculate_box_height(distance_mm)

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

            latest_result = None
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

                # ROI 중심 기준 픽업 좌표(mm). +X 오른쪽, +Y 아래쪽.
                pick_x = (center_x - ROI_WIDTH / 2.0) * MM_PER_PIXEL
                pick_y = (center_y - ROI_HEIGHT / 2.0) * MM_PER_PIXEL
                # 센서 거리 자체가 아니라 상자 상면의 컨베이어 기준 높이.
                # 유효한 높이 없으면 a 키로 확정할 수 없음.
                if box_height_mm is not None and box_height_mm > 0:
                    latest_result = (
                        round(float(angle), 2),
                        [round(float(pick_x), 2), round(float(pick_y), 2), round(float(box_height_mm), 2)],
                        [round(float(width_mm), 2), round(float(height_mm), 2)],
                        round(float(box_height_mm), 2),
                    )

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
                    f"ToF: {distance_mm:.1f} mm" if distance_mm is not None else "ToF: N/A",
                    f"Box height: {box_height_mm:.1f} mm" if box_height_mm is not None else "Box height: N/A",
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
                    tof_text = f"{distance_mm:.1f} mm" if distance_mm is not None else "N/A"
                    box_height_text = f"{box_height_mm:.1f} mm" if box_height_mm is not None else "N/A"
                    print(
                        "[DETECTED] "
                        f"ROI center=({roi_center_x:.1f}, {roi_center_y:.1f}), "
                        f"Global center=({global_center_x:.1f}, {global_center_y:.1f}), "
                        f"ROI mm=({roi_x_mm:.1f}, {roi_y_mm:.1f}), "
                        f"w={width_mm:.1f} mm, d={height_mm:.1f} mm, "
                        f"h={box_height_text}, "
                        f"Angle={angle:.1f}, ToF={tof_text}, "
                        f"Area={contour_area:.0f}, Fill={fill_ratio:.2f}"
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
                return None
            elif key == ord("a"):
                if latest_result is not None:
                    print(f"[Outline] 확정: {latest_result}")
                    return latest_result
                print("[Outline] 유효한 박스/ToF 높이가 없습니다. 센서와 검출을 확인하세요.")

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

            # 문제 분석용 스냅샷: 단일 시점의 모든 검출 단계를 저장
            elif key == ord("c"):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                output_dir = Path("captures") / timestamp
                output_dir.mkdir(parents=True, exist_ok=True)

                snapshots = {
                    "01_full_frame.png": frame,
                    "02_roi.png": roi,
                    "03_raw_mask.png": raw_mask,
                    "04_detection.png": detection_view,
                    "05_difference.png": difference,
                    "06_gray_difference.png": gray_difference,
                    "07_corrected_mask.png": minimal_corrected_mask,
                    "08_full_view.png": full_view,
                }
                failed_files = []
                for filename, snapshot in snapshots.items():
                    if not cv2.imwrite(str(output_dir / filename), snapshot):
                        failed_files.append(filename)

                # 비교용 4분할 이미지: 공유하기 쉬운 크기로 별도 저장
                panels = [
                    ("ROI", roi),
                    ("RAW MASK", raw_mask),
                    ("DETECTION", detection_view),
                    ("CORRECTED MASK", minimal_corrected_mask),
                ]
                panel_w, panel_h = 500, 590
                panel_images = []
                for label, source in panels:
                    if source.ndim == 2:
                        source = cv2.cvtColor(source, cv2.COLOR_GRAY2BGR)
                    display = cv2.resize(source, (panel_w, panel_h), interpolation=cv2.INTER_AREA)
                    cv2.rectangle(display, (0, 0), (panel_w, 42), (30, 30, 30), -1)
                    cv2.putText(display, label, (12, 29), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (255, 255, 255), 2)
                    panel_images.append(display)
                comparison = np.vstack([
                    np.hstack(panel_images[:2]),
                    np.hstack(panel_images[2:]),
                ])
                if not cv2.imwrite(str(output_dir / "00_comparison.png"), comparison):
                    failed_files.append("00_comparison.png")

                if target is not None:
                    debug_text = (
                        f"Selected contour area: {target['area']:.1f} px^2\n"
                        f"Selected fill ratio: {target['fill_ratio']:.3f}\n"
                    )
                else:
                    debug_text = "Selected contour: None\n"
                (output_dir / "diagnostics.txt").write_text(
                    f"Camera: {WIDTH}x{HEIGHT}\n"
                    f"ROI: ({ROI_X1}, {ROI_Y1}) - ({ROI_X2}, {ROI_Y2})\n"
                    f"MM_PER_PIXEL: {MM_PER_PIXEL}\n"
                    f"DIFF_THRESHOLD: {DIFF_THRESHOLD}\n"
                    f"MIN_AREA: {MIN_AREA}, MAX_AREA: {MAX_AREA}\n"
                    f"MIN_FILL_RATIO: {MIN_FILL_RATIO}\n"
                    f"Contours found: {len(contours)}\n"
                    + debug_text,
                    encoding="utf-8",
                )
                print(f"[CAPTURE] 진단 사진 저장 위치: {output_dir.resolve()}")
                if failed_files:
                    print(f"[CAPTURE WARNING] 저장 실패: {', '.join(failed_files)}")
                else:
                    print("[CAPTURE] 00_comparison.png 및 원본/마스크/검출 이미지 저장 완료")

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
        if distance_sensor is not None:
            try:
                distance_sensor.close()
            except Exception as exc:
                print(f"[TOF WARNING] 센서 종료 오류: {exc}")
        cv2.destroyAllWindows()
        print("[INFO] Outline 검출 창 종료 완료")


def main():
    """outline.py 단독 실행용 진입점."""
    camera = create_camera()
    try:
        result = live_outline_detection(camera)
        print("[Outline] 결과:", result)
    finally:
        camera.stop()


if __name__ == "__main__":
    main()


