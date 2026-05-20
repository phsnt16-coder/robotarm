# Raspberry Pi FT-SCS15 Robot Arm Controller

# 위치 범위 : 0 ~ 1023

# Raspberry Pi + FT-SCS15 기반 Pick & Place 제어 코드

import serial
import time

# =========================

# 시리얼 통신 설정

# =========================

PORT = '/dev/ttyACM0'
BAUDRATE = 1000000

# FT-SCS15 서보와 UART 연결

ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)

# =========================

# 서보 및 상태 정의

# =========================

SERVO_COUNT = 7

STATE_IDLE = 0
STATE_MOVE_GRASP = 1
STATE_VAC_ON = 2
STATE_LIFT = 3
STATE_MOVE_PLACE = 4
STATE_VAC_OFF = 5
STATE_DONE = 6
STATE_STOP = 7
STATE_HOME = 8
STATE_READY = 9

POSE_HOME = 0
POSE_READY = 1
POSE_LIFT = 2

# =========================

# 현재 위치 / 목표 위치

# =========================

# 현재 서보 위치 저장

current_pos = [512, 512, 512, 512, 512, 512, 0]

# 목표 서보 위치 저장

target_pos = [512, 512, 512, 512, 512, 512, 0]

# 서보 이동 속도

speed_val = [5, 5, 5, 5, 5, 5, 5]

# =========================

# 좌표 변수

# =========================

box_type = 0
rot_z = 0

# 물체 집는 좌표

gx = 0
gy = 0
gz = 0

# 물체 놓는 좌표

px = 0
py = 0
pz = 0

# 그리퍼 회전 각도

grasp_angle = 512

# 진공 그리퍼 상태

vacuum_state = False

# 현재 상태 머신 상태

state = STATE_IDLE

# 상태 시작 시간 저장

state_start_time = 0

# =========================

# 자세 테이블

# =========================

pose_table = {
POSE_HOME:  [512, 512, 512, 512, 512, 512, 0],
POSE_READY: [512, 450, 450, 400, 400, 512, 0],
POSE_LIFT:  [512, 420, 420, 350, 350, 512, 0]
}

# =========================

# 현재 시간(ms)

# =========================

def millis():
return int(time.monotonic() * 1000)

# =========================

# 체크섬 계산

# =========================

def checksum(data):
return (~sum(data)) & 0xFF

# =========================

# 서보 위치 전송 함수

# =========================

def write_position(servo_id, position):

```
# 하위 바이트 추출
pos_l = position & 0xFF

# 상위 바이트 추출
pos_h = (position >> 8) & 0xFF

# FT-SCS15 패킷 구성
packet = [
    0xFF,
    0xFF,
    servo_id,
    7,
    3,
    0x2A,
    pos_l,
    pos_h,
    0,
    0
]

# 체크섬 추가
packet.append(checksum(packet[2:]))

# UART 전송
ser.write(bytearray(packet))
```

# =========================

# 상태 변경 함수

# =========================

def set_state(new_state):
global state
global state_start_time

```
state = new_state
state_start_time = millis()
```

# =========================

# Pick & Place 데이터 설정

# =========================

def set_pick_place(box, rz, grasp_xyz, place_xyz, angle):

```
global box_type
global rot_z

global gx
global gy
global gz

global px
global py
global pz

global grasp_angle

box_type = box
rot_z = rz

gx, gy, gz = grasp_xyz
px, py, pz = place_xyz

grasp_angle = angle

set_state(STATE_IDLE)
```

# =========================

# 미리 정의된 자세 적용

# =========================

def apply_pose(pose_id):

```
if pose_id not in pose_table:
    return

for i in range(SERVO_COUNT):
    target_pos[i] = pose_table[pose_id][i]
```

# =========================

# 서보 속도 설정

# =========================

def apply_speed(servo_id, speed):

```
if 0 <= servo_id < SERVO_COUNT:
    speed_val[servo_id] = max(1, speed)
```

# =========================

# 개별 서보 이동

# =========================

def apply_move_servo(servo_id, position):

```
if 0 <= servo_id < SERVO_COUNT:
    target_pos[servo_id] = constrain(position, 0, 1023)
```

# =========================

# 전체 정지

# =========================

def stop_all():

```
global vacuum_state

for i in range(SERVO_COUNT):
    target_pos[i] = current_pos[i]

vacuum_state = False

set_state(STATE_STOP)
```

# =========================

# 홈 위치 이동

# =========================

def go_home():
set_state(STATE_HOME)

# =========================

# Ready 위치 이동

# =========================

def go_ready():
set_state(STATE_READY)

# =========================

# 진공 흡착 ON

# =========================

def vacuum_on():
global vacuum_state
vacuum_state = True

# =========================

# 진공 흡착 OFF

# =========================

def vacuum_off():
global vacuum_state
vacuum_state = False

# =========================

# Grasp 좌표 → 서보 목표값 변환

# =========================

def convert_grasp_xyz_to_target():

```
target_pos[0] = constrain(512 + gx, 0, 1023)
target_pos[1] = constrain(512 + gy, 0, 1023)
target_pos[2] = constrain(512 + gy, 0, 1023)
target_pos[3] = constrain(512 + gz, 0, 1023)
target_pos[4] = constrain(512 + gz, 0, 1023)
target_pos[5] = constrain(grasp_angle, 0, 1023)
```

# =========================

# Place 좌표 → 서보 목표값 변환

# =========================

def convert_place_xyz_to_target():

```
target_pos[0] = constrain(512 + px, 0, 1023)
target_pos[1] = constrain(512 + py, 0, 1023)
target_pos[2] = constrain(512 + py, 0, 1023)
target_pos[3] = constrain(512 + pz, 0, 1023)
target_pos[4] = constrain(512 + pz, 0, 1023)
target_pos[5] = constrain(grasp_angle, 0, 1023)
```

# =========================

# 서보 위치 업데이트

# =========================

def update_servo():

```
for i in range(SERVO_COUNT - 1):

    if current_pos[i] < target_pos[i]:
        current_pos[i] += speed_val[i]

        if current_pos[i] > target_pos[i]:
            current_pos[i] = target_pos[i]

    elif current_pos[i] > target_pos[i]:
        current_pos[i] -= speed_val[i]

        if current_pos[i] < target_pos[i]:
            current_pos[i] = target_pos[i]

    # FT-SCS15 서보 ID는 일반적으로 1번부터 사용
    write_position(i + 1, current_pos[i])
```

# =========================

# 진공 그리퍼 업데이트

# =========================

def update_vacuum():
set_vacuum_hardware(vacuum_state)

# =========================

# 실제 진공 하드웨어 제어

# =========================

def set_vacuum_hardware(on):

```
# GPIO 연결 시 구현
pass
```

# =========================

# 목표 도달 여부 확인

# =========================

def is_target_reached():

```
for i in range(SERVO_COUNT - 1):

    if current_pos[i] != target_pos[i]:
        return False

return True
```

# =========================

# 상태 머신 처리

# =========================

def process_state():

```
global vacuum_state

current_time = millis()

# 대기 상태
if state == STATE_IDLE:
    return

# 물체 위치 이동
elif state == STATE_MOVE_GRASP:

    convert_grasp_xyz_to_target()

    if is_target_reached():
        set_state(STATE_VAC_ON)

# 진공 흡착 ON
elif state == STATE_VAC_ON:

    vacuum_state = True

    if current_time - state_start_time >= 500:
        set_state(STATE_LIFT)

# 들어올리기
elif state == STATE_LIFT:

    apply_pose(POSE_LIFT)

    if is_target_reached():
        set_state(STATE_MOVE_PLACE)

# 목표 위치 이동
elif state == STATE_MOVE_PLACE:

    convert_place_xyz_to_target()

    if is_target_reached():
        set_state(STATE_VAC_OFF)

# 진공 OFF
elif state == STATE_VAC_OFF:

    vacuum_state = False

    if current_time - state_start_time >= 300:
        set_state(STATE_DONE)

# 작업 완료
elif state == STATE_DONE:

    apply_pose(POSE_READY)

    if is_target_reached():
        set_state(STATE_IDLE)

# 정지 상태
elif state == STATE_STOP:
    return

# 홈 이동
elif state == STATE_HOME:

    apply_pose(POSE_HOME)

    if is_target_reached():
        set_state(STATE_IDLE)

# Ready 이동
elif state == STATE_READY:

    apply_pose(POSE_READY)

    if is_target_reached():
        set_state(STATE_IDLE)
```

# =========================

# 작업 시작

# =========================

def start_task():
set_state(STATE_MOVE_GRASP)

# =========================

# 값 제한 함수

# =========================

def constrain(value, min_value, max_value):
return max(min_value, min(value, max_value))

# =========================

# 메인 루프

# =========================

def main_loop():

```
while True:
    process_state()
    update_servo()
    update_vacuum()
```

# =========================

# 프로그램 시작

# =========================

if **name** == '**main**':

```
# Pick & Place 좌표 설정
set_pick_place(
    box=1,
    rz=0,
    grasp_xyz=(100, -100, -200),
    place_xyz=(200, -150, -100),
    angle=512
)

# 작업 시작
start_task()

try:
    main_loop()

except KeyboardInterrupt:

    # 강제 종료 시 전체 정지
    stop_all()

    # 시리얼 포트 종료
    ser.close()
```
