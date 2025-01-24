import time
import can
import struct
import numpy as np
import matplotlib.pyplot as plt
import csv

# CANインターフェースの設定
can_interface = 'socketcan'  # 使用するインターフェース名（例：socketcan）
channel = 'can0'  # CANバスのチャネル（例：can0）

# CAN bus設定
bus = can.interface.Bus(channel=channel, bustype=can_interface)

# CANメッセージを送信する関数
def send_can_message(can_id, data):
    # CANメッセージ作成
    message = can.Message(arbitration_id=can_id,
                          data=data,
                          is_extended_id=False)
    
    # メッセージ送信
    try:
        bus.send(message)
        print(f"Message sent: ID={hex(can_id)} Data={data.hex()}")
    except can.CanError:
        print("Error sending message")

# 定期的にメッセージを送信する関数
def send_velocity_to_can(vx, vy):
    # vx と vy を 4 バイトずつ結合して 8 バイトに
    packed_data = struct.pack('ff', vx, vy)  # 2つの float を 8 バイトに
    send_can_message(0x00, packed_data)  # 1つのメッセージとして送信

# PID制御クラス
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # 比例ゲイン
        self.Ki = Ki  # 積分ゲイン
        self.Kd = Kd  # 微分ゲイン
        self.previous_error = np.array([0.0, 0.0])  # 前回の誤差（float型に変更）
        self.integral = np.array([0.0, 0.0])  # 積分誤差（float型に変更）
    
    def update(self, target, current, dt):
        # 目標座標と現在座標の差
        error = target - current
        self.integral += error * dt  # 積分項
        derivative = (error - self.previous_error) / dt  # 微分項
        
        # PID出力（速度）
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.previous_error = error  # 前回の誤差を更新
        return output

# ロボットの移動シミュレーション
def simulate_motion(x0, y0, target_x, target_y, dt, total_time, Kp, Ki, Kd):
    pid = PIDController(Kp, Ki, Kd)
    
    # 初期座標（float64型に変更）
    position = np.array([x0, y0], dtype=np.float64)
    target = np.array([target_x, target_y], dtype=np.float64)
    
    # 時間軸と位置リスト
    time_steps = np.arange(0, total_time, dt)
    trajectory = []
    velocities = []
    
    previous_position = position.copy()
    
    for t in time_steps:
        # PID制御で出力された速度を使って位置を更新
        velocity = pid.update(target, position, dt)
        
        # 速度をCAN通信で送信
        send_velocity_to_can(velocity[0], velocity[1])
        
        # 速度に基づいて位置を更新
        position += velocity * dt
        
        # 速度計算（位置の変化）
        speed = (position - previous_position) / dt
        previous_position = position.copy()
        
        # 位置と速度を記録
        trajectory.append(position.copy())
        velocities.append(speed.copy())
        
        # 目標に到達したら停止
        if np.linalg.norm(target - position) < 1.0:
            break
    
    return np.array(trajectory), np.array(velocities), time_steps

# シミュレーションパラメータ
x0, y0 = 500, 500  # 開始座標
target_x, target_y = 1500, 1500  # 目標座標
dt = 0.1  # タイムステップ
total_time = 100  # シミュレーション時間
Kp, Ki, Kd = 0.1, 0.01, 0.05  # PID制御のゲイン

# シミュレーションの実行
trajectory, velocities, time_steps = simulate_motion(x0, y0, target_x, target_y, dt, total_time, Kp, Ki, Kd)

# CSVファイルに位置と速度を書き込む
csv_filename = "robot_trajectory.csv"
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time [s]", "X Position [mm]", "Y Position [mm]", "X Velocity [mm/s]", "Y Velocity [mm/s]"])
    for t, pos, vel in zip(time_steps[:len(trajectory)], trajectory, velocities):
        writer.writerow([t, pos[0], pos[1], vel[0], vel[1]])

print(f"Position and velocity data have been saved to {csv_filename}")

# グラフのプロット
plt.figure(figsize=(10, 6))

# 位置のグラフ
plt.subplot(2, 1, 1)
plt.plot(time_steps[:len(trajectory)], trajectory[:, 0], label="X Position", color='blue')
plt.plot(time_steps[:len(trajectory)], trajectory[:, 1], label="Y Position", color='red')
plt.axhline(y=target_x, color='green', linestyle='--', label="Target X")
plt.axhline(y=target_y, color='green', linestyle='--', label="Target Y")
plt.xlabel('Time [s]')
plt.ylabel('Position [mm]')
plt.title('Position vs Time')
plt.legend()
plt.grid(True)

# 速度のグラフ
plt.subplot(2, 1, 2)
plt.plot(time_steps[:len(velocities)], velocities[:, 0], label="X Velocity", color='blue')
plt.plot(time_steps[:len(velocities)], velocities[:, 1], label="Y Velocity", color='red')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [mm/s]')
plt.title('Velocity vs Time')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
