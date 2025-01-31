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

# 目標位置
target_x = 0.0  # 目標X座標
target_y = 500.0  # 目標Y座標
taget_theta = 0.0
target_theta = 3.14 * taget_theta / 180  # 目標角度（ラジアン）

# 位置情報のデコード
def decode_position(data):
    print(f"Received data: {data.hex()}")
    if len(data) != 8:
        print(f"Invalid data length: {len(data)} bytes")
        return None, None  # 長さが合わない場合は処理を中止
    x, y = struct.unpack('ff', data)
    return x, y

# 角度情報のデコード
def decode_angle(data):
    if len(data) < 4:
        print(f"Invalid data length: {len(data)} bytes")
        return None  # データが足りない場合はNoneを返す
    theta = struct.unpack('f', data[:4])[0]  # 4バイトをアンパック
    return theta

# PID制御クラス
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # 比例ゲイン
        self.Ki = Ki  # 積分ゲイン
        self.Kd = Kd  # 微分ゲイン
        self.previous_error = 0.0  # 前回の誤差
        self.integral = 0.0  # 積分誤差
    
    def update(self, target, current, dt):
        # 目標と現在値の差を計算
        error = target - current
        self.integral += error * dt  # 積分項
        derivative = (error - self.previous_error) / dt  # 微分項
        
        # PID出力
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # 中間値を表示
        print(f"Error: {error}, Integral: {self.integral}, Derivative: {derivative}, Output: {output}")
        
        self.previous_error = error  # 前回の誤差を更新
        return output

# CANメッセージを送信する関数
def send_can_message(can_id, data):
    message = can.Message(arbitration_id=can_id,
                          data=data,
                          is_extended_id=False)
    try:
        bus.send(message)
        print(f"Message sent: ID={hex(can_id)} Data={data.hex()}")
    except Exception as e:
        print(f"Error sending message: {e}")

# 定期的に速度と角速度を送信する関数
def send_velocity_to_can(vx, vy, theta_dot):
    # vx と vy を 4 バイトずつ結合して 8 バイトに
    packed_data = struct.pack('ff', vx, vy)  # 2つの float を 8 バイトに
    send_can_message(0x00, packed_data)  # 位置制御の速度送信

    # 角速度 theta_dot を送信
    packed_angle_data = struct.pack('ff', theta_dot, 0.0)  # 角速度を送信
    send_can_message(0x01, packed_angle_data)  # 角度制御の速度送信

# 受信スレッドの関数
def receive_can_messages():
    global x_position, y_position, theta

    while True:
        message = bus.recv()  # メッセージを受信

        if message.arbitration_id == 0x02:  # 位置情報
            x_position, y_position = decode_position(message.data)
            print(f"位置情報: x = {x_position}, y = {y_position}")

        elif message.arbitration_id == 0x03:  # 角度情報
            theta = decode_angle(message.data)
            print(f"角度情報: theta = {theta}")

# メインルーチン
def main(dt):
    global x_position, y_position, theta

    # 位置制御用PIDコントローラ
    pid_position = PIDController(Kp=0.8, Ki=0.00, Kd=0.0)  # 位置制御のPIDゲイン

    # 角度制御用PIDコントローラ
    pid_angle = PIDController(Kp=0.8, Ki=0.0, Kd=0.0)  # 角度制御のPIDゲイン

    print("CAN Busを監視中...")

    last_send_time = time.time()  # 送信時刻の初期化

    # 受信スレッドの開始
    receive_thread = threading.Thread(target=receive_can_messages)
    receive_thread.daemon = True  # メインプログラムが終了するとスレッドも終了するように設定
    receive_thread.start()

    # 終了判定用の閾値
    position_error_threshold = 5.0  # 位置誤差がこの閾値以下で終了
    angle_error_threshold = 0.087   # 角度誤差がこの閾値以下で終了
    velocity_threshold = 5.0       # 速度ベクトルがこの閾値以下で終了
    theta_dot_threshold = 0.0087      # 角速度がこの閾値以下で終了

    while True:
        # 目標位置と角度に向かってPID制御を行う
        if x_position is not None and y_position is not None and theta is not None:
            target_position = np.array([target_x, target_y])
            current_position = np.array([x_position, y_position])

            # 位置制御：PIDで速度を計算
            velocity = np.zeros(2)
            velocity[0] = pid_position.update(target_position[0], current_position[0], dt)  # x方向
            velocity[1] = pid_position.update(target_position[1], current_position[1], dt)  # y方向
            modified_vx = velocity[0] * np.cos(theta) - velocity[1] * np.sin(theta)
            modified_vy = velocity[0] * np.sin(theta) + velocity[1] * np.cos(theta)
            velocity[0] = modified_vx
            velocity[1] = modified_vy

            # 角度制御：PIDで角速度を計算
            target_angle = target_theta
            angle_error = target_angle - theta
            theta_dot = pid_angle.update(target_angle, theta, dt)  # 角速度

            # 位置エラーと角度エラーを表示
            position_error = np.linalg.norm(target_position - current_position)
            print(f"目標位置: ({target_x}, {target_y}), 現在位置: ({x_position}, {y_position}), 位置エラー: {position_error}")
            print(f"目標角度: {target_theta} rad, 現在角度: {theta} rad, 角度エラー: {angle_error} rad")

            # 送信する速度と角速度を表示
            print(f"送信する速度ベクトル: vx = {velocity[0]}, vy = {velocity[1]}, 角速度: {theta_dot}")

            # 速度と角速度をCANに送信
            send_velocity_to_can(velocity[0], velocity[1], theta_dot)

            # 現在の時刻を取得して送信間隔を計算
            current_time = time.time()
            send_interval = (current_time - last_send_time) * 1000  # ミリ秒に変換
            last_send_time = current_time  # 最後の送信時刻を更新

            print(f"速度ベクトル送信間隔: {send_interval:.2f} ms")

            # 終了条件のチェック
            if position_error < position_error_threshold and abs(angle_error) < angle_error_threshold:
                if np.linalg.norm([velocity[0], velocity[1]]) < velocity_threshold and abs(theta_dot) < theta_dot_threshold:
                    print("目標位置と目標角度に到達しました。PID制御を終了します。")
                    send_velocity_to_can(0.0, 0.0, 0.0)
                    break  # 制御ループを抜ける

if __name__ == "__main__":
    # 時間間隔を変更可能
    main(dt=0.05)  # dtは変更可能
