import time
import can
import struct
import numpy as np
import threading

# CANインターフェースの設定
CAN_INTERFACE = 'can0'  # 使用するCANインターフェース
CAN_BAUDRATE = 500000  # 通信速度 500kbps

# CANバスを初期化
bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', bitrate=CAN_BAUDRATE)

# 受信データ格納用変数
x_position = None
y_position = None
theta = None
pid_running = False  # PID制御が実行中かどうかのフラグ

# 位置情報のデコード
def decode_position(data):
    if len(data) != 8:
        return None, None
    x, y = struct.unpack('ff', data)
    return x, y

# 角度情報のデコード
def decode_angle(data):
    if len(data) < 4:
        return None
    theta = struct.unpack('f', data[:4])[0]
    return theta

# PID制御クラス
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

# CANメッセージ送信
def send_can_message(can_id, data):
    message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    try:
        bus.send(message)
    except Exception as e:
        print(f"Error sending message: {e}")

# 定期的に速度と角速度を送信する
def send_velocity_to_can(vx, vy, theta_dot):
    packed_data = struct.pack('ff', vx, vy)
    send_can_message(0x00, packed_data)
    packed_angle_data = struct.pack('ff', theta_dot, 0.0)
    send_can_message(0x01, packed_angle_data)

# 受信スレッド
def receive_can_messages():
    global x_position, y_position, theta
    while True:
        message = bus.recv()
        if message.arbitration_id == 0x02:
            x_position, y_position = decode_position(message.data)
        elif message.arbitration_id == 0x03:
            theta = decode_angle(message.data)

# PID制御
def run_pid_control(target_x_new, target_y_new, target_theta_new, target_time=None, dt=0.05):
    global x_position, y_position, theta, pid_running

    if pid_running:
        print("PID control interrupted. Switching to new target.")
    
    pid_running = True
    target_x, target_y, target_theta = target_x_new, target_y_new, target_theta_new
    pid_position = PIDController(Kp=0.8, Ki=0.00, Kd=0.0)
    pid_angle = PIDController(Kp=0.8, Ki=0.0, Kd=0.0)

    receive_thread = threading.Thread(target=receive_can_messages)
    receive_thread.daemon = True
    receive_thread.start()

    position_error_threshold = 5.0
    angle_error_threshold = 0.087
    velocity_threshold = 5.0
    theta_dot_threshold = 0.0087

    last_send_time = time.time()

    while True:
        if x_position is not None and y_position is not None and theta is not None:
            target_position = np.array([target_x, target_y])
            current_position = np.array([x_position, y_position])
            distance_to_target = np.linalg.norm(target_position - current_position)

            # 目標位置に到達するための時間を考慮して、移動速度を調整
            if target_time is not None and target_time > 0:
                average_velocity = distance_to_target / target_time
            else:
                average_velocity = 0.0  # target_timeがない場合は通常のPID制御を続ける

            # 位置制御：PIDで速度を計算
            velocity = np.zeros(2)
            velocity[0] = pid_position.update(target_position[0], current_position[0], dt)
            velocity[1] = pid_position.update(target_position[1], current_position[1], dt)

            # 速度調整
            velocity[0] *= average_velocity / np.linalg.norm(velocity)
            velocity[1] *= average_velocity / np.linalg.norm(velocity)

            modified_vx = velocity[0] * np.cos(theta) - velocity[1] * np.sin(theta)
            modified_vy = velocity[0] * np.sin(theta) + velocity[1] * np.cos(theta)
            velocity[0] = modified_vx
            velocity[1] = modified_vy

            # 角度制御：PIDで角速度を計算
            target_angle = target_theta
            angle_error = target_angle - theta
            theta_dot = pid_angle.update(target_angle, theta, dt)

            position_error = np.linalg.norm(target_position - current_position)
            print(f"Position Error: {position_error}, Angle Error: {angle_error}")

            send_velocity_to_can(velocity[0], velocity[1], theta_dot)

            current_time = time.time()
            send_interval = (current_time - last_send_time) * 1000
            last_send_time = current_time

            print(f"Send Interval: {send_interval:.2f} ms")

            if position_error < position_error_threshold and abs(angle_error) < angle_error_threshold:
                if np.linalg.norm([velocity[0], velocity[1]]) < velocity_threshold and abs(theta_dot) < theta_dot_threshold:
                    print("Target reached. Stopping PID control.")
                    send_velocity_to_can(0.0, 0.0, 0.0)
                    pid_running = False
                    break

            time.sleep(0.01)

# 呼び出し例
target_x = 10.0
target_y = 10.0
target_theta = np.pi / 2  # 目標角度
target_time = 10.0  # 目標時間
run_pid_control(target_x, target_y, target_theta, target_time)
