import can
import struct

# CANインターフェースの設定
CAN_INTERFACE = 'can0'  # 使用するCANインターフェース
CAN_BAUDRATE = 500000  # 通信速度 500kbps

# CANバスを初期化
bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', bitrate=CAN_BAUDRATE)

# 受信データ格納用変数
x_position = None
y_position = None
theta = None

def decode_position(data):
    """
    位置情報 (x, y) をデコードして表示
    """
    x, y = struct.unpack('ff', data)  # 2つのfloatを解凍
    return x, y

def decode_angle(data):
    """
    角度情報 (theta) をデコードして表示
    """
    theta = struct.unpack('f', data[:8])[0]  # 角度のfloatを解凍
    return theta

def main():
    global x_position, y_position, theta

    print("CAN Busを監視中...")

    while True:
        # メッセージを受信
        message = bus.recv()

        if message.arbitration_id == 0x02:  # 位置情報
            x_position, y_position = decode_position(message.data)
            print(f"位置情報: x = {x_position}, y = {y_position}")

        elif message.arbitration_id == 0x03:  # 角度情報
            theta = decode_angle(message.data)
            print(f"角度情報: theta = {theta}")

if __name__ == "__main__":
    main()
