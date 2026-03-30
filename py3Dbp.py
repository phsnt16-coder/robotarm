import itertools

# Item Class


class Item:
    def __init__(self, name, w, h, d, weight=1):
        self.name = name
        self.w = w
        self.h = h
        self.d = d
        self.weight = weight


        self.x = None
        self.y = None
        self.z = None


    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z




# -------------------------
# LiveBin Class
# -------------------------
class LiveBin:
    def __init__(self, W, H, D, max_weight=9999):
        self.W = W
        self.H = H
        self.D = D
        self.max_weight = max_weight


        self.current_weight = 0
        self.placed_items = []
        self.extreme_points = [(0, 0, 0)]




    # AABB 충돌 판정
    def check_collision(self, item, x, y, z):
        for placed in self.placed_items:
            if not (
                x + item.w <= placed.x or
                placed.x + placed.w <= x or
                y + item.d <= placed.y or
                placed.y + placed.d <= y or
                z + item.h <= placed.z or
                placed.z + placed.h <= z
            ):
                return True
        return False




    # 컨테이너 범위 검사
    def fits_in_container(self, item, x, y, z):
        return (
            x + item.w <= self.W and
            y + item.d <= self.D and
            z + item.h <= self.H
        )


# 안정성 검사: 바닥(z=0)에 닿아 있거나, 아래에 다른 아이템이 있는지 확인
    def is_stable(self, item, x, y, z):
        if z == 0:
            return True  # 바닥에 직접 놓이는 경우 안전함
        
        # 아이템의 바닥면 면적 계산
        item_bottom_area = item.w * item.d
        support_area = 0
        
        for placed in self.placed_items:
            # 배치된 아이템의 윗면이 현재 아이템의 바닥면 높이(z)와 일치하는지 확인
            if abs((placed.z + placed.h) - z) < 1e-5:
                # x, y 평면에서 겹치는 영역(서포트 영역) 계산
                inter_w = max(0, min(x + item.w, placed.x + placed.w) - max(x, placed.x))
                inter_d = max(0, min(y + item.d, placed.y + placed.d) - max(y, placed.y))
                support_area += inter_w * inter_d
        
        # 아래쪽 아이템들이 현재 아이템 바닥 면적의 50% 이상을 받쳐주고 있는가?   
        return support_area >= (item_bottom_area * 0.5)

    # BLF 점수
    def calculate_score(self, x, y, z):
        return z * 10000 + y * 100 + x




    # 아이템 배치
    def place_item(self, item):
        gap=1 #아이템간 간격

        if self.current_weight + item.weight > self.max_weight:
            return False


        best_position = None
        best_score = float("inf")


        rotations = set(itertools.permutations([item.w, item.h, item.d]))


        for (rw, rh, rd) in rotations:
            for (x, y, z) in self.extreme_points:


                temp_item = Item(item.name, rw+gap, rh+gap, rd, item.weight)


                if not self.fits_in_container(temp_item, x, y, z):
                    continue


                if self.check_collision(temp_item, x, y, z):
                    continue


                score = self.calculate_score(x, y, z)


                if score < best_score:
                    best_score = score
                    best_position = (x, y, z, rw, rh, rd)


        if best_position is None:
            return False


        x, y, z, rw, rh, rd = best_position
        item.w, item.h, item.d = rw, rh, rd
        item.set_position(x, y, z)


        self.placed_items.append(item)
        self.current_weight += item.weight


        self.update_extreme_points(item)


        return True




    # Extreme Point 업데이트
    def update_extreme_points(self, item):
        new_points = [
            (item.x + item.w, item.y, item.z),
            (item.x, item.y + item.d, item.z),
            (item.x, item.y, item.z + item.h),
        ]


        for p in new_points:
            if p not in self.extreme_points:
                self.extreme_points.append(p)




    def print_state(self):
        print("\n현재 적재 상태:")
        for item in self.placed_items:
            print(f"{item.name} → pos=({item.x},{item.y},{item.z}) "
                  f"size=({item.w},{item.h},{item.d})")
        print("총 적재 무게:", self.current_weight)






# 실시간 입력 루프
if __name__ == "__main__":


    bin = LiveBin(10, 10, 10, max_weight=100)


    print("=== 실시간 3D 적재 시스템 ===")
    print("입력 형식: name,width,height,depth,weight")
    print("종료: Q")


    while True:
        user_input = input("\nItem 입력: ")


        if user_input.upper() == "Q":
            break


        try:
            name, w, h, d, weight = user_input.split(",")
            item = Item(name.strip(),
                        float(w),
                        float(h),
                        float(d),
                        float(weight))


            success = bin.place_item(item)


            if success:
                print(f"[성공] {item.name} → ({item.x},{item.y},{item.z}) 배치됨")
            else:
                print(f"[실패] {item.name} 적재 불가")


            bin.print_state()


        except:
            print("입력 형식 오류. 예: A,5,5,5,1")
