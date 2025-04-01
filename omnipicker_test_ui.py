import serial
import serial.tools.list_ports
import struct
import tkinter as tk
from tkinter import ttk

# 默认波特率
BAUD_RATE = 50000

# 设置 can1 参数
set_can1 = b'\x49\x3B\x42\x57\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'
# 开启 can0、1 通道
start_can = b'\x49\x3B\x44\x57\x01\x00\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'
# 设置 can0 参数
set_can0 = b'\x49\x3B\x42\x57\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'



def list_serial_ports():
    """列出所有可用串口"""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def open_serial(port, baudrate=BAUD_RATE):
    """打开串口"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"串口 {port} 已打开，波特率 {baudrate}")
        return ser
    except Exception as e:
        print(f"无法打开串口 {port}: {e}")
        return None


def send_data(ser, data):
    """发送数据到串口"""
    if ser and ser.is_open:
        ser.write(data)
        print(f"发送数据: {data.hex().upper()}")
    else:
        print("串口未打开，无法发送数据")


def read_data(ser):
    """读取串口返回数据"""
    if ser and ser.is_open:
        data = ser.read(32)  # 读取最大 32 字节
        if data:
            print(f"接收数据: {data.hex().upper()}")
            return data
        else:
            print("未收到数据")
    else:
        print("串口未打开")
    return None


class CANApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CAN 数据发送")
        self.center_window()
        self.ser = None  # 串口对象
        self.is_sending = False
        self.is_configured = False
        # 清除旧的样式
        style = ttk.Style()
        style.theme_use("clam")  # 选择一个兼容性较好的主题
        # 设定全局样式
        style = ttk.Style()
        style.configure("TCombobox", font=("Arial", 18))  # 调整 Combobox 本体字体
        # 让下拉菜单的字体更大
        self.root.option_add("*TCombobox*Listbox.font", ("Arial", 18))

        # 主框架
        self.main_frame = tk.Frame(root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=40, pady=40)

        # 串口选择框架
        self.port_frame = tk.Frame(self.main_frame)
        self.port_frame.pack(fill=tk.X, pady=20)

        self.port_label = tk.Label(self.port_frame, text="串口:", font=("Arial", 18), height=2)
        self.port_label.pack(side=tk.LEFT, padx=20)

        self.port_combobox = ttk.Combobox(self.port_frame, state="readonly", font=("Arial", 18))
        self.refresh_ports()
        self.port_combobox.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20, pady=10)

        self.refresh_button = tk.Button(self.port_frame, text="刷新", command=self.refresh_ports, font=("Arial", 18), height=2)
        self.refresh_button.pack(side=tk.RIGHT, padx=20)
        
        # 串口操作按钮
        self.open_button = tk.Button(self.main_frame, text="打开串口", command=self.open_serial_port, font=("Arial", 18), height=2)
        self.open_button.pack(fill=tk.X, pady=20)

        # 创建 Combobox，并添加选项
        self.can_combobox = ttk.Combobox(self.main_frame, state="readonly", values=["can0", "can1"],font=("Arial", 18), height=10)
        self.can_combobox.pack(fill=tk.X, pady=20)
        # 设置默认值（可选）
        self.can_combobox.current(0)  # 默认选择第一个选项

        # 滑动条
        self.slider = tk.Scale(self.main_frame, from_=0, to=1, orient="horizontal", resolution=0.01, label="调节 CAN 数据",
                            font=("Arial", 18), length=800)
        self.slider.pack(fill=tk.X, padx=40, pady=40)

        # 按钮框架
        self.button_frame = tk.Frame(self.main_frame)
        self.button_frame.pack(fill=tk.X, pady=20)

        self.start_button = tk.Button(self.button_frame, text="开始发送", command=self.start_sending, state=tk.DISABLED,
                                    font=("Arial", 18), height=2)
        self.start_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)

        self.stop_button = tk.Button(self.button_frame, text="停止发送", command=self.stop_sending, state=tk.DISABLED,
                                    font=("Arial", 18), height=2)
        self.stop_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)

        # 让控件根据窗口大小自动调整
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        
    def center_window(self, width=1280, height=900):
        """让窗口居中"""
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - width) // 2
        y = (screen_height - height) // 2
        self.root.geometry(f"{width}x{height}+{x}+{y}")
    def refresh_ports(self):
        # 获取包含 "ACM" 的串口
        ports = [port.device for port in serial.tools.list_ports.comports() if "ACM" in port.device]

        # 更新 Combobox 选项
        self.port_combobox["values"] = ports

        # 如果有可用的串口，则默认选中第一个
        if ports:
            self.port_combobox.current(0)
        else:
            self.port_combobox['values'] = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combobox.current(0)
    def open_serial_port(self):
        """打开串口"""
        selected_port = self.port_combobox.get()
        if selected_port:
            self.ser = open_serial(selected_port)
            if self.ser:
                self.start_button.config(state=tk.NORMAL)  # 启用开始按钮

    def start_sending(self):
        """开始发送 CAN 数据"""
        if self.ser:
            self.is_sending = True
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.send_can_data()

    def stop_sending(self):
        """停止发送"""
        self.is_sending = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def send_can_data(self):
        """发送 CAN 数据"""
        if self.ser and self.is_sending:
            if not self.is_configured:
                send_data(self.ser, set_can0)
                # time.sleep(0.1)
                read_data(self.ser)
                send_data(self.ser, set_can1)
                # time.sleep(0.1)
                read_data(self.ser)
                send_data(self.ser, start_can)
                # time.sleep(0.1)
                read_data(self.ser)
                self.is_configured = True
            selected_value = self.can_combobox.get()
            if selected_value == "can0":
                # 发送 can0 数据
                can_data = self.calculate_can_data(self.slider.get())
                self.send_can_message(self.ser, 0x001, can_data, 0x00)
            if selected_value == "can1":
                # 发送 can1 数据
                can_data = self.calculate_can_data(self.slider.get())
                self.send_can_message(self.ser, 0x001, can_data, 0x01)
            # 发送 CAN 数据
            # can_data = self.calculate_can_data(self.slider.get())
            # self.send_can_message(self.ser, 0x001, can_data, 0x01)  # 发送 CAN 数据
            read_data(self.ser)

            # 100ms 后继续发送
            self.root.after(50, self.send_can_data)

    def send_can_message(self, ser, can_id, data, channel):
        """
        发送 CAN 数据帧
        :param ser: 串口对象
        :param can_id: 4字节 CAN ID
        :param data: 发送数据，最大 64 字节
        :param channel: CAN 通道（0 或 1）
        """
        can_id_bytes = struct.pack(">I", can_id)  # CAN ID 转换成 4字节

        data_length = len(data)
        if data_length > 64:
            data = data[:64]  # 限制数据长度为 64 字节

        frame_header = b'\x5A'  # 帧头
        frame_info_1 = (data_length | channel << 7).to_bytes(1, 'big')  # CAN通道0, DLC数据长度
        frame_info_2 = b'\x00'  # 发送类型: 正常发送, 标准帧, 数据帧, 不加速
        frame_data = data.ljust(64, b'\x00')  # 数据填充到 64 字节
        frame_end = b'\xA5'  # 帧尾

        send_frame = frame_header + frame_info_1 + frame_info_2 + can_id_bytes + frame_data[:data_length] + frame_end
        print("发送 CAN 帧:", send_frame.hex().upper())
        send_data(ser, send_frame)

    def calculate_can_data(self, slider_value):
        """基于滑动条值计算 CAN 数据"""
        left_data = b'\x00\x00\xFF\xFF\xFF\xFF\x00\x00'
        right_data = b'\x00\xFF\xFF\xFF\xFF\xFF\x00\x00'

        new_data = bytearray()
        for i in range(len(left_data)):
            new_byte = int(left_data[i] + (right_data[i] - left_data[i]) * slider_value)
            new_data.append(new_byte)

        return bytes(new_data)

    def close(self):
        """关闭串口"""
        if self.ser:
            self.ser.close()


# 启动 GUI 应用
def main():
    root = tk.Tk()
    app = CANApp(root)

    root.protocol("WM_DELETE_WINDOW", lambda: (app.stop_sending(), app.close(), root.quit()))
    root.mainloop()


if __name__ == "__main__":
    main()
