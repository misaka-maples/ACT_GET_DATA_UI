import socket
from time import sleep,time
import threading
import re

class PersistentClient:
    HEADER = b'&'
    FOOTER = b'^'
    ENCODING = 'utf-8'

    # HEARTBEAT_INTERVAL = 10  # 🟢 心跳间隔
    RECV_TIMEOUT = 2         # 🟢 读取超6时时间

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None  # 连接对象
        self.connected = False  # 连接状态

        # 建立初始连接
        self.connect()

        # 用于存储接收到的数据的缓冲区
        self.response_buffer = []
        # 用于同步访问缓冲区的条件变量
        self.response_cond = threading.Condition()

        # 启动心跳检测线程
        # threading.Thread(target=self._heartbeat, daemon=True).start()

        # 启动实时读取线程（独立线程，不影响写入）
        threading.Thread(target=self._receive_data, daemon=True).start()

    def _frame_data(self, data):
        """封装数据包（增加协议头和尾部）"""
        if not isinstance(data, bytes):
            data = data.encode(self.ENCODING)
        return self.HEADER + data + self.FOOTER

    def connect(self):
        """建立长连接"""
        if self.sock:
            self.sock.close()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)  # 连接超时

        try:
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(self.RECV_TIMEOUT)  # 🟢 设置 `recv()` 超时
            self.connected = True
            print("[INFO] 成功建立长连接")
        except (ConnectionRefusedError, TimeoutError) as e:
            print(f"[ERROR] 连接失败: {e}")
            self.connected = False

    def send_message(self, message):
        """发送数据（仅发送，写入线程）"""
        if not self.connected:
            print("[WARNING] 连接已断开，正在尝试重新连接...")
            self.connect()

        try:
            framed_data = self._frame_data(message)
            print(framed_data)
            self.sock.sendall(framed_data)
            print(f"[INFO] 成功发送 {len(framed_data)} 字节")
            return True
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"[ERROR] 连接断开: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"[ERROR] 未知错误: {e}")
            return False

    # def _receive_data(self):
    #     """实时接收数据（独立线程，不影响1写入）"""
    #     while True:
    #         if self.connected:
    #             try:
    #                 data = self.sock.recv(1024)
    #                 if data:
    #                     print(f"[INFO] 实时接收到数据: {data.decode(self.ENCODING)}")
    #             except socket.timeout:
    #                 continue  # 🟢 超时不报错，继续监听
    #             except (ConnectionResetError, BrokenPipeError):
    #                 print("[WARNING] 服务器断开连接，正在重连...")
    #                 self.connected = False
    #                 self.connect()
    def _receive_data(self):
        """实时接收数据（独立线程，不影响写入）"""
        while True:
            if self.connected:
                try:
                    data = self.sock.recv(1024)
                    if data:
                        decoded_data = data.decode(self.ENCODING)
                        print(f"[INFO] 实时接收到数据: {decoded_data}")
                        # 将数据添加到缓冲区，并通知等待的线程
                        with self.response_cond:
                            self.response_buffer.append(decoded_data)
                            self.response_cond.notify_all()
                except socket.timeout:
                    continue  # 超时后继续监听
                except (ConnectionResetError, BrokenPipeError):
                    print("[WARNING] 服务器断开连接，正在重连...")
                    self.connected = False
                    self.connect()


    def _heartbeat(self):
        """定期发送心跳包，防止服务器断开连接"""
        while True:
            sleep(self.HEARTBEAT_INTERVAL)
            if self.connected:
                try:
                    self.sock.sendall(b"&PING^")
                    print("[INFO] 发送心跳包")
                except (BrokenPipeError, ConnectionResetError):
                    print("[WARNING] 心跳检测失败，连接断开")
                    self.connected = False
                    self.connect()

    def close(self):
        """关闭连接"""
        if self.sock:
            self.sock.close()
            self.connected = False
            print("[INFO] 连接已关闭")

    def set_arm_position(self, value: list, model: str,robotnum:str) -> bool:
        """
        设置机械臂位置（关节模式或位姿模式）

        :param value: 目标位置列表（关节角或位姿坐标）
        :param model: 模式选择，可选 "joint"（关节） 或 "pose"（位姿）
        :return: 发送成功返回True，失败返回False
        """
        # 参数校验
        if not isinstance(value, list) or len(value) != 6:
            print("[ERROR] 输入必须是包含6个数值的列表")
            return False

        if model not in ["joint", "pose"]:
            print("[ERROR] 模式参数必须是 'joint' 或 'pose'")
            return False

        try:
            # 将数值列表转换为协议字符串
            value_str = ",".join(f"{x:.4f}" for x in value)

            if model == "joint":
                # 关节模式协议示例: &SET_JOINT,1.0000,2.0000,3.0000,4.0000,5.0000,6.0000^
                command = f"set,{robotnum},10,ACS,0,0,{value_str},0,10,10" #后两个10分别为加速度、减速度
            if model == "pose":
                # 位姿模式协议示例: &SET_POSE,10.0000,20.0000,30.0000,40.0000,50.0000,60.0000^
                command = f"set,{robotnum},10,PCS,0,0,{value_str},0,10,10"

            # 调用底层发送方法（会自动添加协议头尾）
            return self.send_message(command)

        except Exception as e:
            print(f"[ERROR] 设置位置失败: {e}")
            return False

    def get_arm_postion_joint(self,robotnum):
            """
            获取机械臂位姿：
            发送请求后等待缓冲区中出现响应数据，最多等待5秒。
            :return: 返回接收到的字符串响应数据，或None（超时）
            """
            message = f"get,{robotnum},ACS"
            self.send_message(message)
            start_time = time()

            # 等待响应数据到来
            with self.response_cond:
                while not self.response_buffer and (time() - start_time < 5):
                    remaining = 5 - (time() - start_time)
                    self.response_cond.wait(timeout=remaining)
                if self.response_buffer:
                    response = self.response_buffer.pop(0)  # 取出最早的响应
                else:
                    print("[ERROR] 等待响应超时")
                    response =None

            match = re.search(r'getPos:"([^"]+)"', response)
            if match:
                # 提取并解析六个浮动数据
                data_string = match.group(1)  # 获取 "2,0,0,0,-7.2092,133.368,500.813,-1.63063,-0.0261585,-1.57236,0"
                data_list = data_string.split(',')[4:10]  # 获取从第5到第10个数据（索引从0开始）
                # 将数据转换为浮动数并返回
                return [float(i) for i in data_list]
            else:
                print("[ERROR] 无法解析位置数据")
                return None
            # 返回接收到的响应字符串

    def get_arm_position_pose(self, robotnum):
        """
        获取机械臂位姿：
        发送请求后等待缓冲区中出现响应数据，最多等待5秒。
        :return: 返回接收到的字符串响应数据，或None（超时）
        """
        message = f"get,{robotnum},PCS"
        message = message.strip()
        self.send_message(message)
        start_time = time()

        # 等待响应数据到来
        with self.response_cond:
            while not self.response_buffer and (time() - start_time < 5):
                remaining = 5 - (time() - start_time)
                self.response_cond.wait(timeout=remaining)
            if self.response_buffer:
                response = self.response_buffer.pop(0)  # 取出最早的响应
            else:
                print("[ERROR] 等待响应超时")
                response =None

        match = re.search(r'getPos:"([^"]+)"', response)
        if match:
            # 提取并解析六个浮动数据
            data_string = match.group(1)  # 获取 "2,0,0,0,-7.2092,133.368,500.813,-1.63063,-0.0261585,-1.57236,0"
            data_list = data_string.split(',')[4:10]  # 获取从第5到第10个数据（索引从0开始）
            # 将数据转换为浮动数并返回
            return [float(i) for i in data_list]
        else:
            print("[ERROR] 无法解析位置数据")
            return None
        # 返回接收到的响应字符串


if __name__ == "__main__":
    client = PersistentClient('192.168.3.15', 8001)
    # messages = [ "get,2,ACS"]
    #
    #
    # for msg in messages:
    #     success = client.send_message(msg)
    #     print(f"发送状态: {'成功' if success else '失败'}")
    #     sleep(0.001)  # 等待间隔

    while True:
        try:
            message = input("> ")  # 🟢 从命令行获取输入

            if message.lower() == "exit":
                print("[INFO] 退出客户端...")
                client.close()
                break

            if message == "1,1":
                deta = client.get_arm_position_pose(1)
                print(deta)

            if message == "1,2":
                deta = client.get_arm_position_pose(2)
                print(deta)

            if message == "2":
                deta = client.set_arm_position([300.064,3.71456,230.247,3.14159,0.000167089,0],"pose",2)
            if message == "3":
                deta = client.set_arm_position([20,-486.1,430.1,1.66675,-0.0284042,1.48369], "pose", 1)
            if message.strip():  # 🟢 避免发送空消息
                success = client.send_message(message)
                print(f"发送状态: {'成功' if success else '失败'}")




        except KeyboardInterrupt:
            print("\n[INFO] 终止客户端...")
            client.close()
            break
