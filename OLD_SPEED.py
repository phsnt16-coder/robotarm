from scservo_sdk import *

PORT = "COM6"
BAUD = 1000000

SERVO_ID = 4

portHandler = PortHandler(PORT)
packetHandler = scscl(portHandler)

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUD):
    print("Baudrate 실패")
    quit()

print("===== 구형 FT-SCS15 내부 파라미터 읽기 =====")

# 24 : Punch
punch, result, error = packetHandler.read2ByteTxRx(
    SERVO_ID,
    24
)

print(f"Punch (24) : {punch}")

# 44 : Running Time
running_time, result, error = packetHandler.read2ByteTxRx(
    SERVO_ID,
    44
)

print(f"Running Time (44) : {running_time}")

# 46 : Goal Velocity
goal_velocity, result, error = packetHandler.read2ByteTxRx(
    SERVO_ID,
    46
)

print(f"Goal Velocity (46) : {goal_velocity}")

# 58 : Present Velocity
present_velocity, result, error = packetHandler.read2ByteTxRx(
    SERVO_ID,
    58
)

print(f"Present Velocity (58) : {present_velocity}")

# 56 : Present Position
present_position, result, error = packetHandler.read2ByteTxRx(
    SERVO_ID,
    56
)

print(f"Present Position (56) : {present_position}")

# 63 : Temperature
temperature, result, error = packetHandler.read1ByteTxRx(
    SERVO_ID,
    63
)

print(f"Temperature (63) : {temperature}°C")

portHandler.closePort()

print("읽기 완료")