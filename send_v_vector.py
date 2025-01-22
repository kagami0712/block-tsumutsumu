import time
import can
import struct

# CANインターフェースの設定
can_interface = 'socketcan'  # 使用するインターフェース名（例：socketcan）
channel = 'can0'  # CANバスのチャネル（例：can0）

# CAN bus設定
bus = can.interface.Bus(channel=channel, bustype=can_interface)

# vx, vy, omgの値（例としてランダムに設定）
vx = 200.0
vy = 0.0
omg = 0.0

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
    while True:
        if can_id == 0x00:
            # vx と vy を 4 バイトずつ結合して 8 バイトに
            packed_data = struct.pack('ff', vx, vy)  # 2つの float を 8 バイトに
            print(len(packed_data))  # packed_dataの長さを確認
            send_can_message(0x00, packed_data)  # 1つのメッセージとして送信
        elif can_id == 0x01:
            # omg のみ送信
            packed_data = struct.pack('f', omg)  # 1つの float を 4 バイトに
            send_can_message(0x01, packed_data)  # omgを送信
        
        time.sleep(1)  # 1秒間隔で送信

# CAN IDを指定して送信を開始
if __name__ == "__main__":
    can_id = 0x00  # vx, vyを送信する
    periodic_send(can_id)
