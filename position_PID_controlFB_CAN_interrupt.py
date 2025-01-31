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
target_y = 500.0   # 目標Y座標

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
        self.previous_error = np.array([0.0, 0.0])  # 前回の誤差
        self.integral = np.array([0.0, 0.0])  # 積分誤差
    
    def update(self, target, current, dt):
        # 目標座標と現在座標の差を計算
        error = target - current
        self.integral += error * dt  # 積分項
        derivative = (error - self.previous_error) / dt  # 微分項
        
        # PID出力（速度）
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        self.previous_error = error  # 前回の誤差を更新
        
        # 制御の詳細をデバッグとして出力
        print(f"PID Error: {error}, Integral: {self.integral}, Derivative: {derivative}, Output: {output}")

        time.sleep(dt)
        
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

# 定期的にメッセージを送信する関数
def send_velocity_to_can(vx, vy):
    # vx と vy を 4 バイトずつ結合して 8 バイトに
    packed_data = struct.pack('ff', vx, vy)  # 2つの float を 8 バイトに
    print(f"Sending velocity vector: vx = {vx}, vy = {vy}")  # 速度ベクトルを表示
    send_can_message(0x00, packed_data)  # 1つのメッセージとして送信

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

    pid = PIDController(Kp=0.8, Ki=0.00, Kd=0.0)  # PIDゲインの設定

    print("CAN Busを監視中...")

    last_send_time = time.time()  # 送信時刻の初期化

    # 受信スレッドの開始
    receive_thread = threading.Thread(target=receive_can_messages)
    receive_thread.daemon = True  # メインプログラムが終了するとスレッドも終了するように設定
    receive_thread.start()

    while True:
        # 目標位置に向かってPID制御を行う
        if x_position is not None and y_position is not None:
            target = np.array([target_x, target_y])
            current = np.array([x_position, y_position])

            # PID制御で計算された速度を取得
            velocity = pid.update(target, current, dt)

            # 速度ベクトルをCANに送信
            send_velocity_to_can(velocity[0], velocity[1])

            # 現在の時刻を取得して送信間隔を計算
            current_time = time.time()
            send_interval = (current_time - last_send_time) * 1000  # ミリ秒に変換
            last_send_time = current_time  # 最後の送信時刻を更新

            print(f"速度ベクトル送信間隔: {send_interval:.2f} ms")

if __name__ == "__main__":
    # 時間間隔を変更可能
    main(dt=0.05)  # dtは変更可能
