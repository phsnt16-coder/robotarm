import time
import serial
import numpy as np
from robot_ik import RobotIKController #역기구학 연산 모듈

def motor_control_receiver(q):
    """
    메인 프로세스로부터 큐를 통해 데이터를 전달받아 처리하는 프로세스 함수.
    """
    # 실제 로봇 연결 시 사용될 UART 설정 (필요할 때 주석 해제)
    # ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    
    # IK 컨트롤러 객체 생성
    ik_solver = RobotIKController()
    
    print("\n[수신 모듈] 가동 시작 - 데이터 대기 중...")
    
    while True:
        if not q.empty():
            # 큐에서 패킷 데이터 획득
            packet_data = q.get()
            print(f"\n[디버그용]현재 큐 데이터: {packet_data}")
            # 데이터 분해
            label = packet_data['label']
            px, py, pz = packet_data['pick']
            lx, ly, lz = packet_data['load']
            angle = packet_data['angle']
            
            # [확인용 출력] 이 부분이 잘 나오면 메모리 전송 성공입니다.
            print("\n" + "="*45)
            print(f"[수신 성공] 라벨: {label} | 각도: {angle:.1f}°")
            print(f" - Pick: [{px:.1f}, {py:.1f}, {pz:.1f}]")
            print(f" - Load: [{lx:.1f}, {ly:.1f}, {lz:.1f}]")
            print("="*45)
            
            # pick 데이터를 target_position 리스트로 설정(mm 변환)
            target_position = [px / 1000.0, py / 1000.0, pz / 1000.0]
            
            # 역기구학(IK) 계산 실행
            calculated_joints = ik_solver.calculate_ik(target_position)
            
            # 순기구학(FK) 검증 데이터 연산
            computed_pos = ik_solver.verify_fk(calculated_joints)
            
            # 라디안 데이터를 디그리(도) 단위 배열로 변환
            calculated_degrees = np.degrees(calculated_joints).tolist()
            
            # 계산 결과 출력 확인
            print(f"[IK 연산 결과]")
            print(f" - 목표 Pick 위치: {target_position}")
            print(f" - 계산된 도달 위치: {computed_pos}")
            print(f" - 계산된 관절 각도: {calculated_degrees}")
            print("="*45)
            
            # --- 여기에 나중에 실제 모터 제어 함수를 호출하세요 ---
            # execute_motor_move(packet_data) 
            
        time.sleep(0.01) # CPU 점유율 최적화
