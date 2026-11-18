"""Measured Pick IK: complete XY grid at a single CSV-defined Z plane, bilinear interpolation.
Outside measured domain: nearest measured XYZ (no extrapolation).
No motor commands; requested and reference coordinates are reported separately.
CSV angles are preserved exactly; no shoulder offset is applied.
"""
import csv
import math
from bisect import bisect_right
from pathlib import Path

ANGLE_KEYS = ('deg1', 'deg23', 'deg45', 'deg6', 'deg7')
DATA_PATH = Path(__file__).resolve().with_name('pick_ik_mapping.csv')


class PickIKLookup:
    def __init__(self, csv_path=DATA_PATH):
        self.nodes = {}
        z_values = set()
        with open(csv_path, encoding='utf-8-sig', newline='') as fp:
            for row in csv.DictReader(fp):
                x, y, z = (float(row[k]) for k in ('x_mm', 'y_mm', 'z_mm'))
                a = {k: float(row[k]) for k in ANGLE_KEYS}
                if not all(map(math.isfinite, (x, y, z))) or not all(math.isfinite(v) and 0 <= v <= 220 for v in a.values()):
                    raise ValueError(f'Invalid measurement: {row["id"]}')
                if (x, y) in self.nodes:
                    raise ValueError(f'Duplicate grid point: {x}, {y}')
                z_values.add(z)
                self.nodes[x, y] = (row['id'], a, row.get('review', ''))
        self.xs = sorted({x for x, y in self.nodes})
        self.ys = sorted({y for x, y in self.nodes})
        if len(z_values) != 1 or not self.xs or not self.ys:
            raise ValueError('Expected a nonempty grid at one measured Z plane')
        self.z = next(iter(z_values))
        if any((x, y) not in self.nodes for x in self.xs for y in self.ys):
            raise ValueError('Incomplete Pick IK grid')

    @staticmethod
    def _axis(v, grid):
        if v in grid:
            return [(v, 1.0)]
        i = bisect_right(grid, v)
        a, b = grid[i-1], grid[i]
        t = (v-a)/(b-a)
        return [(a, 1-t), (b, t)]

    def solve(self, xyz_mm, *, shoulder_mode="raw"):
        xyz = tuple(map(float, xyz_mm))
        if len(xyz) != 3 or not all(map(math.isfinite, xyz)):
            raise ValueError('Pick LUT requires finite XYZ (mm)')
        x, y, z = xyz
        inside = (self.xs[0] <= x <= self.xs[-1]
                  and self.ys[0] <= y <= self.ys[-1]
                  and abs(z-self.z) <= 1e-8)
        if shoulder_mode not in ('raw', 'logical'):
            raise ValueError('shoulder_mode must be raw or logical (legacy compatibility; both pass angles unchanged)')
        refs = []
        warnings = []
        a = {k: 0.0 for k in ANGLE_KEYS}
        reference_xyz = xyz
        distance = 0.0
        if not inside:
            xx, yy = min(self.nodes, key=lambda p: (
                math.dist(xyz, (p[0], p[1], self.z)), p[0], p[1]))
            reference_xyz = (xx, yy, self.z)
            distance = math.dist(xyz, reference_xyz)
            mid, angles, review = self.nodes[xx, yy]
            a = dict(angles)
            refs.append((mid, 1.0))
            classification = 'nearest_measured'
            warnings.append(
                f'Pick LUT outside measured domain: requested={xyz}, '
                f'nearest={reference_xyz}, distance={distance:.2f} mm. '
                'Angles reach the reference point, not the requested XYZ; '
                'physical reach and clearance are not verified.')
            if review and review != 'corner_unchecked':
                warnings.append(f'{mid}: local midpoint deviation flag: {review}')
        else:
            for xx, wx in self._axis(x, self.xs):
                for yy, wy in self._axis(y, self.ys):
                    mid, angles, review = self.nodes[xx, yy]
                    w = wx*wy
                    refs.append((mid, w))
                    if review and review != 'corner_unchecked':
                        warnings.append(f'{mid}: local midpoint deviation flag: {review}')
                    for k in ANGLE_KEYS:
                        a[k] += w*angles[k]
            classification = 'measured_exact' if len(refs)==1 else 'xy_bilinear'
        # Both legacy modes now pass measured angles unchanged.
        a = {k: round(v, 6) for k, v in a.items()}
        if not (0 <= a['deg23'] <= 200 and 10 <= a['deg45'] <= 220 and all(0 <= v <= 220 for v in a.values())):
            raise ValueError(f'Pick LUT output outside command range: {a}')
        return {'angles': a, 'classification': classification,
                'contributors': refs, 'warnings': warnings, 'xyz_mm': xyz,
                'reference_xyz_mm': reference_xyz, 'nearest_distance_mm': distance}


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('x', type=float); ap.add_argument('y', type=float); ap.add_argument('z', type=float)
    ap.add_argument('--shoulder', choices=['raw', 'logical'], default='raw')
    args = ap.parse_args()
    print(PickIKLookup().solve((args.x, args.y, args.z), shoulder_mode=args.shoulder))
