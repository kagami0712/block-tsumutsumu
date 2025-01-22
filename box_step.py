import time
import can
import struct
import math

# CANインターフェースの設定
can_interface = 'socketcan'  # 使用するインターフェース名（例：socketcan）
channel = 'can0'  # CANバスのチャネル（例：can0）

# CAN bus設定
bus = can.interface.Bus(channel=channel, bustype=can_interface)

# 初期値
vx = 200.0  # 速度X
vy = 0.0    # 速度Y
omg = 0.0   # 角速度
side_length = 1.0  # ひし形の1辺の長さ（仮設定）
speed = 200.0  # 進行速度

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
def periodic_send(can_id):
    directions = ['right', 'up', 'left', 'down']  # ひし形の移動方向
    direction_idx = 0  # 初期方向
    while True:
        # 移動方向の設定
        if directions[direction_idx] == 'right':
            vx = speed
            vy = 0.0
            omg = 0.0
        elif directions[direction_idx] == 'up':
            vx = 0.0
            vy = speed
            omg = 0.0
        elif directions[direction_idx] == 'left':
            vx = -speed
            vy = 0.0
            omg = 0.0
        elif directions[direction_idx] == 'down':
            vx = 0.0
            vy = -speed
            omg = 0.0

        # 定期的に速度ベクトルを送信
        if can_id == 0x00:
            # vx と vy を 4 バイトずつ結合して 8 バイトに
            packed_data = struct.pack('ff', vx, vy)  # 2つの float を 8 バイトに
            send_can_message(0x00, packed_data)  # 1つのメッセージとして送信
        elif can_id == 0x01:
            # omg のみ送信
            packed_data = struct.pack('f', omg)  # 1つの float を 4 バイトに
            send_can_message(0x01, packed_data)  # omgを送信

        # ひし形の頂点を1辺進む時間を決定（仮設定で1秒）
        time.sleep(2)

        # 次の方向へ
        direction_idx = (direction_idx + 1) % len(directions)

# CAN IDを指定して送信を開始
if __name__ == "__main__":
    can_id = 0x00  # vx, vyを送信する
    periodic_send(can_id)
