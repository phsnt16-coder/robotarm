#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Place IK 168-row measured lookup, center coordinates in millimeters.

Supported measured Z planes: 50 mm (1F), 100 mm (2F).
No cross-Z interpolation/extrapolation. Unmeasured XY points permit preview-only IDW
inside the same plane's convex hull. This module never commands motors.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
from scipy.spatial import Delaunay, cKDTree

DATA_PATH = Path(__file__).resolve().parent / 'place_ik_mapping.csv'
ANGLE_COLS = ('Base_deg', '23_deg', '45_deg', '6_deg', '7_deg')
ANGLE_KEYS = ('deg1', 'deg23', 'deg45', 'deg6', 'deg7')
SUPPORTED_Z_MM = (50.0, 100.0)
Z_TOLERANCE_MM = 0.5
MAX_LOCAL_TRIANGLE_EDGE_MM = 35.0
K_NEAREST = 6


class PlaceIKError(ValueError):
    """Unsupported or unverified Place IK request."""


@dataclass(frozen=True)
class Node:
    point_id: str
    x_mm: float
    y_mm: float
    z_mm: float
    angles: Tuple[float, float, float, float, float]
    source: str
    base_status: str
    original_base_deg: float
    suspect: bool

    @property
    def measured_exact(self) -> bool:
        return self.base_status in ('실측', '원본')


@dataclass(frozen=True)
class PlaceIKResult:
    xyz_mm: Tuple[float, float, float]
    angles: Dict[str, float]
    classification: str
    contributors: Tuple[Tuple[str, float, str], ...]
    has_extrapolated_anchor: bool
    warnings: Tuple[str, ...]
    region: str
    nearest_distance_mm: float
    maximum_triangle_edge_mm: float

    @property
    def measured_exact(self) -> bool:
        return self.classification == 'measured_exact'


class PlaceIKLookup:
    def __init__(self, csv_path=DATA_PATH):
        self.csv_path = Path(csv_path)
        if not self.csv_path.is_file():
            raise FileNotFoundError(f'Place IK CSV가 없습니다: {self.csv_path}')
        self.nodes = {}
        self.groups = {50.0: [], 100.0: []}
        self.meshes = {}
        with self.csv_path.open(encoding='utf-8-sig', newline='') as fp:
            reader = csv.DictReader(fp)
            required = {'측정ID', '중앙X_cm', '중앙Y_cm', 'Z_cm', *ANGLE_COLS, 'Base_상태'}
            if not required.issubset(set(reader.fieldnames or ())):
                raise ValueError(f'CSV 필수 열 누락: {sorted(required - set(reader.fieldnames or ())) }')
            for row in reader:
                try:
                    point_id = row['측정ID'].strip()
                    x, y, z = (float(row[key]) * 10.0 for key in ('중앙X_cm', '중앙Y_cm', 'Z_cm'))
                    angles = tuple(float(row[key]) for key in ANGLE_COLS)
                    status = row['Base_상태'].strip()
                    raw_base = (row.get('Base_원본_deg') or '').strip()
                    original_base = float(raw_base) if raw_base else angles[0]
                    suspect = (row.get('오류의심') or '').strip().lower() in {'y', 'yes', 'true', '1', '예'}
                except (TypeError, ValueError, KeyError) as exc:
                    raise ValueError(f'CSV 데이터 형식 오류: {row}') from exc
                if not point_id or point_id in self.nodes:
                    raise ValueError(f'빈 ID 또는 중복 ID: {point_id!r}')
                if status not in {'실측', '원본', '선형보간', '기존 선형보간', '신규 선형보간', '보류'}:
                    raise ValueError(f'알 수 없는 Base 상태: {point_id}: {status}')
                if not all(isfinite(v) for v in (x, y, z, *angles, original_base)):
                    raise ValueError(f'비유한 측정값: {point_id}')
                if not all(0 <= a <= 220 for a in angles):
                    raise ValueError(f'관절각 허용 범위 밖: {point_id}')
                if z not in self.groups:
                    raise ValueError(f'지원하지 않는 Z 높이: {point_id}: {z}mm')
                if z == 100.0 and '2F' not in point_id.upper():
                    raise ValueError(f'Z=100mm 측정 ID에 2F 없음: {point_id}')
                if z == 50.0 and '2F' in point_id.upper():
                    raise ValueError(f'Z=50mm 측정 ID에 2F가 포함됨: {point_id}')
                region = '2F' if z == 100.0 else '1F'
                node = Node(point_id, x, y, z, angles, region, status, original_base, suspect)
                self.nodes[point_id] = node
                self.groups[z].append(node)
        if len(self.groups[50.0]) != 84 or len(self.groups[100.0]) != 84:
            raise ValueError('통합 LUT에 1F 84개, 2F 84개가 필요합니다.')
        for z, nodes in self.groups.items():
            xy = np.array([(n.x_mm, n.y_mm) for n in nodes], dtype=float)
            if len(set(map(tuple, xy))) != len(nodes):
                raise ValueError(f'{z:.0f}mm 평면에 중복 XY 좌표가 있습니다.')
            self.meshes[z] = (xy, Delaunay(xy), cKDTree(xy))

    def nearest_xy_by_plane(self, xyz_mm, limit=4):
        """Report XY neighbors at each measured Z, never imply Z is supported."""
        coords = tuple(float(v) for v in xyz_mm)
        if len(coords) != 3 or not all(isfinite(v) for v in coords):
            raise PlaceIKError('유효한 상자 중심 [X,Y,Z] mm가 필요합니다.')
        x, y, target_z = coords
        output = {}
        for z, nodes in self.groups.items():
            xy, _tri, tree = self.meshes[z]
            distances, indices = tree.query([x, y], k=min(limit, len(nodes)))
            output[f'{int(z)}mm'] = [
                {'id': nodes[int(idx)].point_id,
                 'xyz_mm': [nodes[int(idx)].x_mm, nodes[int(idx)].y_mm, z],
                 'xy_distance_mm': round(float(dist), 3),
                 'z_difference_mm': round(z-target_z, 3),
                 'status': nodes[int(idx)].base_status,
                 'suspect': bool(nodes[int(idx)].suspect)}
                for dist, idx in zip(np.atleast_1d(distances), np.atleast_1d(indices))
            ]
        return output

    @staticmethod
    def _parse_xyz(xyz_mm):
        try:
            coords = tuple(float(v) for v in xyz_mm)
        except (TypeError, ValueError) as exc:
            raise PlaceIKError('박스 중심 좌표 [X,Y,Z] mm 숫자 3개 필요') from exc
        if len(coords) != 3 or not all(isfinite(v) for v in coords):
            raise PlaceIKError('유한한 숫자 [X,Y,Z] mm 3개를 입력하세요.')
        z_matches = [plane for plane in SUPPORTED_Z_MM if abs(coords[2] - plane) <= Z_TOLERANCE_MM]
        if len(z_matches) != 1:
            raise PlaceIKError(f'Z={coords[2]:.1f}mm는 미측정 높이입니다. Z=50/100mm만 지원하며 Z 보간은 하지 않습니다.')
        return coords, z_matches[0]

    def solve(self, xyz_mm, *, require_measured=False, allow_sparse=False,
              allow_corrected=False, allow_suspect=False) -> PlaceIKResult:
        """Lookup measured XY or preview IDW on a single Z plane.

        require_measured=True blocks non-exact points and suspicious/corrected
        samples unless explicitly permitted. Motor motions are not executed.
        """
        (x, y, z), plane = self._parse_xyz(xyz_mm)
        nodes = self.groups[plane]
        xy, tri, tree = self.meshes[plane]
        distances, indexes = tree.query([x, y], k=min(K_NEAREST, len(nodes)))
        distances, indexes = np.atleast_1d(distances), np.atleast_1d(indexes)
        nearest = float(distances[0])
        region = '2F' if plane == 100.0 else '1F'
        if nearest <= 1e-5:
            node = nodes[int(indexes[0])]
            corrected = not node.measured_exact and node.base_status != '보류'
            suspect = node.suspect or node.base_status == '보류'
            if require_measured and corrected and not allow_corrected:
                raise PlaceIKError(f'{node.point_id}: Base 선형보간값입니다. 실측 검증 전 사용 제한 (allow_corrected 필요).')
            if require_measured and suspect and not allow_suspect:
                raise PlaceIKError(f'{node.point_id}: 오류의심/보정 보류 측정값입니다. 재검증 전 사용 제한 (allow_suspect 필요).')
            warnings = []
            if corrected:
                warnings.append(f'{node.point_id} Base 선형보간 {node.angles[0]:.2f}° (기존 {node.original_base_deg:.2f}°)')
            if suspect:
                warnings.append(f'{node.point_id}: 오류의심 또는 보정 보류 데이터입니다.')
            warnings.append('모서리에서 측정 후 중앙좌표로 옮긴 데이터의 TCP/실제 위치 정합은 별도 검증 필요.')
            classification = 'base_corrected_exact' if corrected else 'suspect_exact' if suspect else 'measured_exact'
            return PlaceIKResult((x, y, z), dict(zip(ANGLE_KEYS, node.angles)),
                                 classification, ((node.point_id, 1.0, region),), False,
                                 tuple(warnings), region, nearest, 0.0)
        if require_measured:
            raise PlaceIKError('입력 XY는 정확한 실측점이 아닙니다. 자동 구동용 보간을 허용하지 않습니다.')
        simplex = int(tri.find_simplex(np.array([[x, y]], dtype=float))[0])
        if simplex < 0:
            raise PlaceIKError('동일 Z 평면의 실측 영역 밖이므로 외삽을 허용하지 않습니다.')
        corners = xy[tri.simplices[simplex]]
        edges = [float(np.linalg.norm(corners[i] - corners[(i + 1) % 3])) for i in range(3)]
        max_edge = max(edges)
        sparse = max_edge > MAX_LOCAL_TRIANGLE_EDGE_MM + 1e-7
        if sparse and not allow_sparse:
            raise PlaceIKError(f'측정점 간격이 넓습니다 (최대 {max_edge:.1f}mm). allow_sparse=True는 미리보기용입니다.')
        weights = 1.0 / np.maximum(distances, 1e-9) ** 2
        weights /= weights.sum()
        values = np.sum(np.array([nodes[int(i)].angles for i in indexes]) * weights[:, None], axis=0)
        contributors = tuple((nodes[int(i)].point_id, round(float(w), 6), region) for i, w in zip(indexes, weights))
        warnings = ['동일 Z 평면의 IDW 보간 결과: 위치 정확도 및 충돌 검증 전 모터 구동 금지.']
        if sparse:
            warnings.append(f'희소 측정 구역 (삼각형 최대 변 {max_edge:.1f}mm).')
        if any(not nodes[int(i)].measured_exact for i in indexes):
            warnings.append('기여점에 Base 보간/보류값이 포함되어 있습니다.')
        if any(nodes[int(i)].suspect for i in indexes):
            warnings.append('기여점에 오류의심 측정값이 포함되어 있습니다.')
        return PlaceIKResult((x, y, z), {k: round(float(v), 4) for k, v in zip(ANGLE_KEYS, values)},
                             'sparse_interpolated' if sparse else 'interpolated',
                             contributors, False, tuple(warnings), region, nearest, max_edge)

    def motor_angles(self, xyz_mm, **kwargs):
        return self.solve(xyz_mm, **kwargs).angles


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('x', type=float, help='상자 중앙 X(mm)')
    ap.add_argument('y', type=float, help='상자 중앙 Y(mm)')
    ap.add_argument('z', type=float, help='상자 중앙 Z(mm): 50 또는 100')
    ap.add_argument('--csv', type=Path, default=DATA_PATH)
    ap.add_argument('--measured-only', action='store_true')
    ap.add_argument('--allow-corrected', action='store_true')
    ap.add_argument('--allow-suspect', action='store_true')
    ap.add_argument('--allow-sparse', action='store_true', help='희소구간 미리보기')
    args = ap.parse_args()
    try:
        result = PlaceIKLookup(args.csv).solve(
            (args.x, args.y, args.z), require_measured=args.measured_only,
            allow_corrected=args.allow_corrected, allow_suspect=args.allow_suspect,
            allow_sparse=args.allow_sparse)
    except (PlaceIKError, ValueError, FileNotFoundError) as exc:
        ap.exit(2, f'[PLACE IK 거부] {exc}\n')
    print('측정구역:', result.region, '분류:', result.classification)
    print('입력 중앙좌표 (mm):', result.xyz_mm)
    for key, val in result.angles.items():
        print(f'{key}: {val:.2f}°')
    print('참조점:', result.contributors)
    for msg in result.warnings:
        print('[주의]', msg)
    print('모터 명령은 전송하지 않았습니다.')


if __name__ == '__main__':
    main()
