import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


class BinVisualizer:
    def __init__(self, live_bin):
        self.live_bin = live_bin
        
    def draw_cube(self, ax, x, y, z, w, d, h, name):
        # 상자를 3D 공간에 그리는 함수 
        # 8개의 꼭짓점 좌표 생성
        X = [x, x + w, x + w, x, x, x + w, x + w, x]
        Y = [y, y, y + d, y + d, y, y, y + d, y + d]
        Z = [z, z, z, z, z + h, z + h, z + h, z + h]
        
        vertices = [
            [0, 1, 2, 3], [4, 5, 6, 7], # 아래, 위
            [0, 1, 5, 4], [2, 3, 7, 6], # 앞, 뒤
            [0, 3, 7, 4], [1, 2, 6, 5]  # 왼쪽, 오른쪽
        ]
        
        faces = []
        for face in vertices:
            faces.append([(X[i], Y[i], Z[i]) for i in face])
            
        # 무작위 색상
        color = np.random.rand(3,)
        poly = Poly3DCollection(faces, alpha=0.5, facecolors=color, edgecolors='black', linewidths=1)
        ax.add_collection3d(poly)
        
        # 상자 중앙에 이름 표시
        ax.text(x + w/2, y + d/2, z + h/2, name, color='black', ha='center', va='center', fontsize=9)

    def update_plot(self):
        #LiveBin의 적재 상태를 Matplotlib 3D로 시각화
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. 컨테이너 외곽선 
        ax.set_xlim([0, self.live_bin.W])
        ax.set_ylim([0, self.live_bin.D])
        ax.set_zlim([0, self.live_bin.H])
        
        ax.set_xlabel('Width (X)')
        ax.set_ylabel('Depth (Y)')
        ax.set_zlabel('Height (Z)')
        ax.set_title(f"3D Packing State (Weight: {self.live_bin.current_weight}/{self.live_bin.max_weight})")

        # 2. 배치된 모든 아이템 순회하며 그리기
        for item in self.live_bin.placed_items:
            if item.x is not None:
                self.draw_cube(ax, item.x, item.y, item.z, item.w, item.d, item.h, item.name)
                
        plt.show()