import itertools

class Item:
    def __init__(self, name, w, h, d, weight=1):
        self.name = name
        self.w, self.h, self.d = w, h, d
        self.weight = weight
        self.x = self.y = self.z = None
        self.is_rotated = False

    def set_position(self, x, y, z):
        self.x, self.y, self.z = x, y, z

class LiveBin:
    def __init__(self, W, H, D, max_weight=9999):
        self.W, self.H, self.D = W, H, D
        self.max_weight = max_weight
        self.current_weight = 0
        self.placed_items = []
        self.extreme_points = [(0, 0, 0)]

    def check_collision(self, item, x, y, z):
        for p in self.placed_items:
            if not (x + item.w <= p.x or p.x + p.w <= x or
                    y + item.d <= p.y or p.y + p.d <= y or
                    z + item.h <= p.z or p.z + p.h <= z):
                return True
        return False

    def fits_in_container(self, item, x, y, z):
        return (x + item.w <= self.W and y + item.d <= self.D and z + item.h <= self.H)

    def calculate_score(self, x, y, z):
        # 바닥(z)부터 채우고, 안쪽(y), 왼쪽(x) 순으로 채우는 BLF 알고리즘
        return z * 10000 + y * 100 + x

    def place_item(self, item):
        """
        아이템 배치 시 (성공여부, 회전여부)를 반환합니다.
        """
        # 1. 무게 제한 확인
        if self.current_weight + item.weight > self.max_weight:
            return False, False

        best_position = None
        best_score = float("inf")
        
        # 2. 회전 케이스 설정 (높이 h는 고정하고 바닥면 w, d만 90도 회전)
        rotations = [
            (item.w, item.h, item.d, False), # 기본 (회전 안함)
            (item.d, item.h, item.w, True)   # 90도 회전
        ]
        
        for (rw, rh, rd, rotated_flag) in rotations:
            for (x, y, z) in self.extreme_points:
                temp_item = Item(item.name, rw, rh, rd, item.weight)

                if not self.fits_in_container(temp_item, x, y, z):
                    continue

                if self.check_collision(temp_item, x, y, z):
                    continue

                score = self.calculate_score(x, y, z)
                if score < best_score:
                    best_score = score
                    best_position = (x, y, z, rw, rh, rd, rotated_flag)

        # 3. 배치 결과 처리
        if best_position is None:
            return False, False

        # 데이터 업데이트
        x, y, z, rw, rh, rd, is_rotated = best_position
        item.w, item.h, item.d = rw, rh, rd
        item.set_position(x, y, z)
        item.is_rotated = is_rotated

        self.placed_items.append(item)
        self.current_weight += item.weight
        self.update_extreme_points(item)

        return True, is_rotated

    def update_extreme_points(self, item):
        # 새로운 후보 좌표 생성 (적재된 물체의 모서리 지점들)
        new_points = [
            (item.x + item.w, item.y, item.z),
            (item.x, item.y + item.d, item.z),
            (item.x, item.y, item.z + item.h),
        ]
        for p in new_points:
            if p[0] < self.W and p[1] < self.D and p[2] < self.H:
                if p not in self.extreme_points:
                    self.extreme_points.append(p)

    def print_state(self):
        print("\n[현재 적재 상태]")
        for i in self.placed_items:
            print(f"- {i.name}: 위치({i.x},{i.y},{i.z}) 크기({i.w},{i.h},{i.d})")
        print(f"총 무게: {self.current_weight}/{self.max_weight}")
