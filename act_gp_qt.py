import json
import os
import time
from queue import Queue
from threading import Lock
from Robotic_Arm.rm_robot_interface import *
import math, h5py
import cv2
import numpy as np
from tqdm import tqdm
from pyorbbecsdk import *
from tcp_tx import PersistentClient
from utils import frame_to_bgr_image
import random
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QLineEdit,QPushButton, QLabel, QVBoxLayout, QFormLayout,QProgressBar,QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer,QThread, pyqtSignal, QMutex
import threading

frames_queue_lock = Lock()

# Configuration settings
MAX_DEVICES = 3
MAX_QUEUE_SIZE = 2
ESC_KEY = 27
save_points_dir = os.path.join(os.getcwd(), "point_clouds")
save_depth_image_dir = os.path.join(os.getcwd(), "depth_images")
save_color_image_dir = os.path.join(os.getcwd(), "color_images")

frames_queue: list[Queue] = [Queue() for _ in range(MAX_DEVICES)]
stop_processing = False
curr_device_cnt = 0

# Load config file for multiple devices
config_file_path = os.path.join(os.path.dirname(__file__), "../pyorbbecsdk/config/multi_device_sync_config.json")
multi_device_sync_config = {}
camera_names = ['top', 'right_wrist']
camera_sn = {
    'top':'CP1E54200056',#index=0
    'right_wrist':'CP1L44P0006E',#index=1
}
COMPLETED:bool=False
GPCONTROL_COMMAND:int=0
#是位置姿态不是关节值
zero_pos = [0, 0, 0, 0, 0, 0]
original_pos = [-0.242293, 0.055747, 0.692225, -1.569, -0.597, 1.472]#空中点位
final_pos =  [-0.300054, 0.215523, 0.493377, -2.749, -0.706, 1.804]#桌面点位
standard_start_pos = [-0.19953, 0.169551, 0.685523, -1.605, -0.672, 0.916]#桌面点位
test_pos =[-0.106262, 0.165886, 0.68286, 1.667, 0.764, -2.07]
test_pos_2 = [-0.136529, 0.041631, 0.681072, 1.808, 0.971, -1.668]
# original_pos=test_pos_2
# final_pos=test_pos_2
"""
███████████████████████████████████████
█  重要提示:  █未测试，仅完成
███████████████████████████████████████
"""
class GPCONTROL(QThread):
    error_signal = pyqtSignal(object)
    gp_control_state_signal = pyqtSignal(object)  # 反馈信号，用于 UI 连接
    def __init__(self, parent=None, DEFAULT_SERIAL_PORTS = ("/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2") ):
        super().__init__(parent)
        self.state_flag = 0  # 夹爪状态: 0=关, 1=半开, 2=开
        self.running = True  # 控制线程运行
        self.control_command = ""  # 当前控制命令
        self.DEFAULT_SERIAL_PORTS = DEFAULT_SERIAL_PORTS
        self.BAUD_RATE = 50000
        self.min_data = b'\x00\x00\xFF\xFF\xFF\xFF\x00\x00'
        self.max_data = b'\x00\xFF\xFF\xFF\xFF\xFF\x00\x00'
        self.ser = self.open_serial()
        self.is_sending = False
        self.is_configured = False  # 配置标志位
        set_can1 = b'\x49\x3B\x42\x57\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'
        start_can1 = b'\x49\x3B\x44\x57\x01\x00\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'
        self.send_data(set_can1)  # 发送配置指令
        self.read_data()
        self.send_data(start_can1)  # 启动 CAN 通道
        self.read_data()    
    def run(self):
        state =None
        while self.running:
            # print(f"control:{self.state_flag}")
            # 1. 根据当前的 `state_flag` 设定控制命令
            if self.state_flag == 0:
                self.control_command = "CLOSE_GRIPPER"
                state = self.close_gp()
                # print("start gp close gp command ")
            elif self.state_flag == 1:
                self.control_command = "HALF_OPEN_GRIPPER"
                state = self.open_half_gp()
            elif self.state_flag == 2:
                self.control_command = "OPEN_GRIPPER"
                state = self.open_all_gp()
            # 3. 强制获取返回帧
            if state is not None:
                feedback = state
                self.gp_control_state_signal.emit(feedback)  # 发送反馈信号
            # 4. 以 50Hz 频率运行（20ms 间隔）
            # time.sleep(0.02)
    def set_state_flag(self,value):
        """修改 self.state_flag"""
        self.state_flag = value
    def stop(self):
        """
        退出线程
        """
        self.running = False
        print("[INFO] Gripper thread stopping...")

    def open_serial(self):
        """尝试打开两个串口，如果都失败则报错"""
        for port in self.DEFAULT_SERIAL_PORTS:
            try:
                ser = serial.Serial(port, self.BAUD_RATE, timeout=1)
                print(f"串口 {port} 已打开，波特率 {self.BAUD_RATE}")
                return ser
            except Exception as e:
                print(f"无法打开串口 {port}: {e}")
        
        # If both attempts fail, raise an error
        print(f"无法打开任何串口: {', '.join(self.DEFAULT_SERIAL_PORTS)}")
    
    def send_data(self, data):
        """发送数据到串口"""
        ser=self.ser
        if ser and ser.is_open:
            ser.write(data)
            # print(f"发送数据: {data.hex()}")
        else:
            print("串口未打开，无法发送数据")


    def filter_can_data(self, data):
        """根据头（0x5A）和尾（0xA5）过滤数据"""
        valid_frames = []

        # 查找所有以 0x5A 开头并以 0xA5 结尾的数据帧
        start_idx = 0
        while start_idx < len(data):
            # 查找下一个0x5A
            start_idx = data.find(b'\x5A', start_idx)
            if start_idx == -1:  # 如果找不到0x5A，退出循环
                break

            # 查找下一个0xA5
            end_idx = data.find(b'\xA5', start_idx)
            if end_idx == -1:  # 如果找不到0xA5，退出循环
                break

            # 提取有效数据帧（包括0x5A和0xA5）
            frame = data[start_idx:end_idx + 1]

            # 确保数据帧长度合理（至少 8 字节）
            if len(frame) >= 8:
                valid_frames.append(frame)

            # 设置起始索引，继续查找下一个帧
            start_idx = end_idx + 1
        return valid_frames

    def read_data(self):
        """读取串口返回数据并过滤符合头尾要求的数据"""
        ser = self.ser
        if ser and ser.is_open:
            data = ser.read(32)  # 读取最大 64 字节
            if data:
                valid_frames = self.filter_can_data(data)
                if valid_frames:
                    back_data=0
                    for frame in valid_frames:
                        if frame[:2].hex()=='5aff':
                            # print("")
                            continue
                        else:
                            # print(f"接收符合条件的CAN数据: {frame.hex()}")
                            back_data=frame.hex()
                    
                    return valid_frames, back_data
                else:
                    pass
            else:
                print("未收到数据")
        else:
            print("串口未打开，无法读取数据")
        return None
    def send_can_data(self, can_id, data, channel):
        """
        发送 CAN 数据帧
        :param ser: 串口对象
        :param can_id: 4字节 CAN ID
        :param data: 发送数据，最大 64 字节
        """
        can_id_bytes = can_id  # CAN ID 转换成 4字节

        data_length = len(data)
        if data_length > 64:
            data = data[:64]  # 限制数据长度为 64 字节

        frame_header = b'\x5A'  # 帧头
        frame_info_1 = (data_length | channel << 7).to_bytes(1, 'big')  # CAN通道0, DLC数据长度
        frame_info_2 = b'\x00'  # 发送类型: 正常发送, 标准帧, 数据帧, 不加速
        frame_data = data.ljust(64, b'\x00')  # 数据填充到 64 字节
        frame_end = b'\xA5'  # 帧尾

        send_frame = frame_header + frame_info_1 + frame_info_2 + can_id_bytes + frame_data[:data_length] + frame_end
        # print("发送 CAN 帧:", send_frame.hex())
        self.send_data(send_frame)
        # _,data = self.read_data()
        # return data
    def open_half_gp(self):
        half_open_gp = b'\x00\x7f\xFF\xFF\xFF\xFF\x00\x00'
        while 1:
            self.send_can_data(b'\x00\x00\x00\x01', half_open_gp, 0x01)
            data = self.read_data() 
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x01', half_open_gp, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
        
    def open_all_gp(self):
        open_gp = b'\x00\xff\xFF\xFF\xFF\xFF\x00\x00'
        while 1:
            self.send_can_data(b'\x00\x00\x00\x01', open_gp, 0x01)
            data = self.read_data() 
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x01', open_gp, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
        
    def close_gp(self):
        close_gp = b'\x00\x00\xFF\xFF\xFF\xFF\x00\x00'
        while 1:
            self.send_can_data(b'\x00\x00\x00\x01', close_gp, 0x01)
            data = self.read_data() 
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x01', close_gp, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
        
    def control_gp(self, gpstate, gppos, gpforce):
        gpstate = gpstate.to_bytes(2, 'big')
        gppos = gppos.to_bytes(2, 'big')
        gpforce = gpforce.to_bytes(2, 'big')
        gpcontrol_data = b'\x00\x00' + gpstate + gppos + b'\x00\x00' + gpforce
        print(f"gpcontrol_data: {gpcontrol_data.hex()}")
            
        while 1:   
            self.send_can_data(b'\x00\x00\x00\x01', gpcontrol_data, 0x01)
            data = self.read_data()
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x01', gpcontrol_data, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
            # return data
    
    def close(self):
        if self.ser:
            self.ser.close()

class ROBOT:
    def __init__(self):
        self.joint_state_right=None
        self.Client = PersistentClient('192.168.3.15', 8001)
        
        self.rm_65_b_right_arm = (RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E))
        self.arm_ini = self.rm_65_b_right_arm.rm_create_robot_arm("192.168.1.18",8080, level=3)

    def get_state(self, model='joint'):#pose
        self.joint_state_right = self.rm_65_b_right_arm.rm_get_current_arm_state()
        return_action = self.joint_state_right[1][model]
        if model=='joint':
            action=self.Client.get_arm_postion_joint()
        elif model=='pose':
            action = self.Client.get_arm_position_pose()
        
        return action

class ACTION_PLAN(QThread):
    action_signal = pyqtSignal(object)
    complete_signal = pyqtSignal(object)
    def __init__(self,Robot:ROBOT):
        super().__init__()
        self.running = True  # 线程运行状态
        self.Robot=Robot
        self.velocity = 15
        self.gpstate:list
        self.start_point=None
        self.goal_point = None
        self.local_desktop_point = None
        self.loop_len=1
        self.complete =False
        self.random_pos=[]
    def val(self,start_point,goal_point,local_desktop_point):
        self.start_point=start_point
        self.goal_point = goal_point
        self.local_desktop_point = local_desktop_point
    def move(self,position,up=False):
        if up is False:
            # self.Robot.rm_65_b_right_arm.rm_movej_p(position,self.velocity,0,0,1)
            self.Robot.Client.set_arm_position(position,'pose')
        else:
            position_=position.copy()
            position_[1]=position_[1]-0.1
            # self.Robot.rm_65_b_right_arm.rm_movej_p(position_,self.velocity,0,0,1)
            self.Robot.Client.set_arm_position(position_,'pose')
    def run(self):
        while self.running:
            self.run_thread()
            self.back_thread()
    def run_thread(self):
        index=0
        if self.start_point is not  None or self.local_desktop_point is not None or self.goal_point is not None:           
            self.complete_signal.emit(False)
            self.move(self.start_point)
            self.action_signal.emit(2)
            if self.random_pos !=[]:
                self.move(self.random_pos,up=True)
                self.move(self.random_pos)
                self.action_signal.emit(0)
                self.move(self.random_pos,up=True)
            else:
                self.move(self.local_desktop_point,up=True)
                self.move(self.local_desktop_point)
                self.action_signal.emit(0)
                self.move(self.local_desktop_point,up=True)
            self.move(self.goal_point ,up=True)
            self.move(self.goal_point )
            self.action_signal.emit(1)
            self.move(self.goal_point ,up=True)
            self.move(self.start_point)
            self.complete_signal.emit(True)
            print("发送action_plan完成信号")
            time.sleep(2)
            index+=1
        else:
            raise ValueError (f"no point is set")
    def back_thread(self):
        COMPLETED=False
        self.complete_signal.emit(COMPLETED)
        self.move(self.goal_point ,up=True)
        self.action_signal.emit(2)
        self.move(self.goal_point)
        self.action_signal.emit(0)
        self.move(self.goal_point ,up=True)
        self.move(self.start_point)
        self.random_positon()
        # self.move(self.random_pos,up=True)
        # self.move(self.random_pos,up=False)
        # self.action_signal.emit(2)
        # self.move(self.random_pos,up=True)
        self.move(self.local_desktop_point,up=True)
        self.move(self.local_desktop_point,up=False)
        self.action_signal.emit(2)
        self.move(self.local_desktop_point,up=True)
        self.move(self.start_point)
    def random_positon(self):
        self.random_pos = self.local_desktop_point.copy()
        self.random_pos[0] = random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点数
        self.random_pos[2] = random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点
    def set_loop_len(self,value):
        self.loop_len=value
    def stop(self):
        """ 停止线程 """
        self.running = False
        
        self.Robot.rm_65_b_right_arm.rm_set_delete_current_trajectory()
        self.quit()
        self.wait()
class GENERATOR_HDF5:
    def __init__(self):
        pass
    def save_hdf5(self,data_dict,path_to_save_hdf5,episode_idx):
        os.makedirs(path_to_save_hdf5, exist_ok=True)
        self.dataset_path = os.path.join(path_to_save_hdf5, f'episode_{episode_idx}.hdf5')

        try:
            with h5py.File(self.dataset_path, 'w') as root:
                root.attrs['sim'] = True
                obs = root.create_group('observations')
                images_group = obs.create_group('images')
                gp = root.create_group('gp')

                # 创建每个相机的数据集并写入数据
                for cam_name in camera_names:
                    if f'/observations/images/{cam_name}' in data_dict:
                        try:
                            cam_data = np.array(data_dict[f'/observations/images/{cam_name}'])
                            print(f"Saving image for {cam_name}, shape: {cam_data.shape}")  # 打印图片数据的尺寸
                            images_group.create_dataset(
                                cam_name.split('/')[-1],
                                data=cam_data,
                                dtype='uint8',
                            )
                        except Exception as e:
                            print(f"Error saving image data for camera {cam_name}: {e}")

                # 写入 qpos 数据
                if '/observations/qpos' in data_dict:
                    if 'qpos' in obs:
                        print("Dataset 'qpos' already exists. Updating it.")
                        del obs['qpos']
                    qpos_data = np.array(data_dict['/observations/qpos'])
                    print(f"Saving qpos, shape: {qpos_data.shape}")
                    obs.create_dataset(
                        'qpos',
                        data=qpos_data,
                        dtype='float32'
                    )

                # 写入 action 数据
                if '/action' in data_dict:
                    if 'action' in root:
                        print("Dataset 'action' already exists. Updating it.")
                        del root['action']
                    action_data = np.array(data_dict['/action'])
                    print(f"Saving action, shape: {action_data.shape}")
                    root.create_dataset(
                        'action',
                        data=action_data,
                        dtype='float32'
                    )

                # 保存 gpstate, gppos, gpforce 数据
                if '/gp/gppos' in data_dict:
                    if 'gppos' in gp:
                        print("Dataset 'gppos' already exists. Updating it.")
                        del gp['gppos']
                    try:
                        gppos_data = np.array([int(x, 16) for x in data_dict['/gp/gppos']], dtype='int32')
                        print(f"Saving gppos, length: {len(gppos_data)}")
                        gp.create_dataset(
                            'gppos',
                            data=gppos_data
                        )
                    except Exception as e:
                        print(f"Error saving gppos data: {e}")
                        return

                if '/gp/gpstate' in data_dict:
                    if 'gpstate' in gp:
                        print("Dataset 'gpstate' already exists. Updating it.")
                        del gp['gpstate']
                    try:
                        gpstate_data = np.array([int(x, 16) for x in data_dict['/gp/gpstate']], dtype='int32')
                        print(f"Saving gpstate, length: {len(gpstate_data)}")
                        gp.create_dataset(
                            'gpstate',
                            data=gpstate_data
                        )
                    except Exception as e:
                        print(f"Error saving gpstate data: {e}")
                        return

                if '/gp/gpforce' in data_dict:
                    if 'gpforce' in gp:
                        print("Dataset 'gpforce' already exists. Updating it.")
                        del gp['gpforce']
                    try:
                        gpforce_data = np.array([int(x, 16) for x in data_dict['/gp/gpforce']], dtype='int32')
                        print(f"Saving gpforce, length: {len(gpforce_data)}")
                        gp.create_dataset(
                            'gpforce',
                            data=gpforce_data
                        )
                    except Exception as e:
                        print(f"Error saving gpforce data: {e}")
                        return

                # 强制刷新文件，确保数据写入
                root.flush()

        except Exception as e:
            print(f"Error during saving hdf5 file: {e}")
        
        print(f"Data saved to {self.dataset_path}")

class CAMERA():
    def __init__(self):
        ctx = Context()
        self.device_list = ctx.query_devices()
        self.curr_device_cnt = self.device_list.get_count()
        self.pipelines: list[Pipeline] = []
        self.configs: list[Config] = []
        self.serial_number_list:list[str] = ["" for _ in range(self.curr_device_cnt) ]
        self.color_frames_queue: dict[str, Queue] = {}
        self.depth_frames_queue: dict[str, Queue] = {}
        self.setup_cameras()
        self.start_streams()
        # self.frames:FrameSet=None
        print("相机初始化完成")
    def setup_cameras(self):
        self.read_config(config_file_path)

        if self.curr_device_cnt == 0:
            print("No device connected")
            return
        if self.curr_device_cnt > MAX_DEVICES:
            print("Too many devices connected")
            return

        for i in range(self.curr_device_cnt):
            device = self.device_list.get_device_by_index(i)
            serial_number = device.get_device_info().get_serial_number()
        
            # **初始化帧队列**
            self.color_frames_queue[serial_number] = Queue()
            self.depth_frames_queue[serial_number] = Queue()
            pipeline = Pipeline(device)
            config = Config()
            sync_config_json = multi_device_sync_config[serial_number]
            sync_config = device.get_multi_device_sync_config()
            sync_config.mode = self.sync_mode_from_str(sync_config_json["config"]["mode"])
            sync_config.color_delay_us = sync_config_json["config"]["color_delay_us"]
            sync_config.depth_delay_us = sync_config_json["config"]["depth_delay_us"]
            sync_config.trigger_out_enable = sync_config_json["config"]["trigger_out_enable"]
            sync_config.trigger_out_delay_us = sync_config_json["config"]["trigger_out_delay_us"]
            sync_config.frames_per_trigger = sync_config_json["config"]["frames_per_trigger"]
            device.set_multi_device_sync_config(sync_config)
            try:
                profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile: VideoStreamProfile = profile_list.get_default_video_stream_profile()
                config.enable_stream(color_profile)
                profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                depth_profile = profile_list.get_default_video_stream_profile()
                config.enable_stream(depth_profile)

                self.pipelines.append(pipeline)
                self.configs.append(config)
                self.serial_number_list[i] = serial_number
            except OBError as e:
                print(f"setup_cameras error:{e}")

    def start_streams(self):
        # print(type(self.pipelines),type(self.configs),self.configs,self.curr_device_cnt)
        index = 0
        print(self.serial_number_list)
        for index, (pipeline, config, serial) in enumerate(zip(self.pipelines, self.configs, self.serial_number_list)):
            pipeline.start(
                config,
                lambda frame_set, curr_serial=serial: self.on_new_frame_callback(
                    frame_set, curr_serial
                ),
            )
    def stop_straems(self):
        for pipeline in self.pipelines:
            pipeline.stop()
        self.pipelines=[]
        self.configs=[]
        print("device stoped")
    def on_new_frame_callback(self, frames: FrameSet, serial_number: str):
        global MAX_QUEUE_SIZE
        if serial_number not in self.color_frames_queue:
            print(f"⚠️ WARN: 未识别的相机序列号 {serial_number}，跳过帧处理")
            return

        # **获取帧**
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # **处理彩色帧**
        if color_frame is not None:
            if self.color_frames_queue[serial_number].qsize() >= MAX_QUEUE_SIZE:
                self.color_frames_queue[serial_number].get()
            self.color_frames_queue[serial_number].put(color_frame)

        # **处理深度帧**
        if depth_frame is not None:
            if self.depth_frames_queue[serial_number].qsize() >= MAX_QUEUE_SIZE:
                self.depth_frames_queue[serial_number].get()
            self.depth_frames_queue[serial_number].put(depth_frame)

    def rendering_frame(self, max_wait=5):
        
        image_list: dict[str, np.ndarray] = {}
        start_time = time.time()  # 记录开始时间
        color_width, color_height = None, None  # 用于存储最终拼接尺寸

        while len(image_list) != self.curr_device_cnt:  # 直接判断字典长度
            if time.time() - start_time > max_wait:  # **超时处理**
                print("⚠️ WARN: 渲染超时，部分相机未收到帧数据")
                break

            for serial_number in self.color_frames_queue.keys():
                color_frame = None
                depth_frame = None

                if not self.color_frames_queue[serial_number].empty():
                    color_frame = self.color_frames_queue[serial_number].get()
                if not self.depth_frames_queue[serial_number].empty():
                    depth_frame = self.depth_frames_queue[serial_number].get()

                if color_frame is None and depth_frame is None:
                    continue  # 跳过无数据的相机

                color_image = None
                depth_image = None

                if color_frame is not None:
                    color_width, color_height = color_frame.get_width(), color_frame.get_height()
                    color_image = frame_to_bgr_image(color_frame)

                if depth_frame is not None:
                    width, height = depth_frame.get_width(), depth_frame.get_height()
                    scale = depth_frame.get_depth_scale()
                    depth_format = depth_frame.get_format()
                    if depth_format != OBFormat.Y16:
                        print(f"⚠️ WARN: 相机 {serial_number} 深度格式非 Y16，跳过处理")
                        continue
                    depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
                    depth_data = (depth_data.astype(np.float32) * scale).astype(np.uint8)
                    depth_image = cv2.applyColorMap(cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX), cv2.COLORMAP_JET)

                if color_image is not None:
                    image_list[serial_number] = color_image  # 使用 `serial_number` 作为 key

            # **拼接所有图像**
            if len(image_list) == self.curr_device_cnt and color_width is not None and color_height is not None:
                result_image = np.hstack(list(image_list.values()))
                result_image = cv2.resize(result_image, (color_width, color_height))  # 统一尺寸
                break

        return image_list

    def rendering_frame_(self):
        global stop_rendering
        image_list: dict[int, np.ndarray] = {}
        color_width, color_height = 0, 0
        while len(image_list) < self.curr_device_cnt:
            all_color_frames = {}

            # 遍历所有设备，确保所有设备的 color 帧同步
            for i in range(self.curr_device_cnt):
                if not self.color_frames_queue[i].empty():
                    while not self.color_frames_queue[i].empty():
                        latest_color_frame = self.color_frames_queue[i].get()  # 丢弃旧帧，取最新
                    all_color_frames[i] = latest_color_frame  # 只存最新帧
            
            # 等待所有设备都拿到最新 color 帧
            if len(all_color_frames) == self.curr_device_cnt:
                for i in range(self.curr_device_cnt):
                    color_image = frame_to_bgr_image(all_color_frames[i])
                    color_width, color_height = all_color_frames[i].get_width(), all_color_frames[i].get_height()
                    image_list[i] = color_image

            # 确保 image_list 不会超出 curr_device_cnt
            if len(image_list) > self.curr_device_cnt:
                image_list.clear()

        # 确保所有图像大小一致
        target_size = (color_height, color_width)
        image_list = {k: cv2.resize(v, target_size) for k, v in image_list.items()}

        # 拼接时确保设备顺序一致
        if len(image_list) == self.curr_device_cnt:
            result_image = np.hstack([image_list[i] for i in sorted(image_list.keys())])

            return result_image
    def sync_mode_from_str(self, sync_mode_str: str) -> OBMultiDeviceSyncMode:

        # to lower case
        sync_mode_str = sync_mode_str.upper()
        if sync_mode_str == "FREE_RUN":
            return OBMultiDeviceSyncMode.FREE_RUN
        elif sync_mode_str == "STANDALONE":
            return OBMultiDeviceSyncMode.STANDALONE
        elif sync_mode_str == "PRIMARY":
            return OBMultiDeviceSyncMode.PRIMARY
        elif sync_mode_str == "SECONDARY":
            return OBMultiDeviceSyncMode.SECONDARY
        elif sync_mode_str == "SECONDARY_SYNCED":
            return OBMultiDeviceSyncMode.SECONDARY_SYNCED
        elif sync_mode_str == "SOFTWARE_TRIGGERING":
            return OBMultiDeviceSyncMode.SOFTWARE_TRIGGERING
        elif sync_mode_str == "HARDWARE_TRIGGERING":
            return OBMultiDeviceSyncMode.HARDWARE_TRIGGERING
        else:
            raise ValueError(f"Invalid sync mode: {sync_mode_str}")
    def read_config(self, config_file: str):
        global multi_device_sync_config
        with open(config_file, "r") as f:
            config = json.load(f)
        for device in config["devices"]:
            multi_device_sync_config[device["serial_number"]] = device
            print(f"Device {device['serial_number']}: {device['config']['mode']}")

class CAMERA_HOT_PLUG:
    def __init__(self):
        self.mutex = QMutex()
        self.ctx = Context()
        self.device_list = self.ctx.query_devices()
        self.curr_device_cnt = self.device_list.get_count()
        self.pipelines: list[Pipeline] = []
        self.configs: list[Config] = []
        self.serial_number_list: list[str] = ["" for _ in range(self.curr_device_cnt)]
        self.color_frames_queue: dict[str, Queue] = {}
        self.depth_frames_queue: dict[str, Queue] = {}
        self.setup_cameras()
        self.start_streams()
        print("相机初始化完成")
        self.monitor_thread = threading.Thread(target=self.monitor_devices, daemon=True)
        self.monitor_thread.start()

    def monitor_devices(self):
        while True:
            time.sleep(2)
            new_device_list = self.ctx.query_devices()
            new_device_cnt = new_device_list.get_count()
            if new_device_cnt != self.curr_device_cnt:
                print("设备变化检测到，重新初始化相机...")
                self.stop_streams()
                self.device_list = new_device_list
                self.curr_device_cnt = new_device_cnt
                self.setup_cameras()
                self.start_streams()

    def setup_cameras(self):
        self.read_config(config_file_path)

        if self.curr_device_cnt == 0:
            print("No device connected")
            return
        if self.curr_device_cnt > MAX_DEVICES:
            print("Too many devices connected")
            return

        for i in range(self.curr_device_cnt):
            device = self.device_list.get_device_by_index(i)
            serial_number = device.get_device_info().get_serial_number()
            self.color_frames_queue[serial_number] = Queue()
            self.depth_frames_queue[serial_number] = Queue()
            pipeline = Pipeline(device)
            config = Config()
            sync_config_json = multi_device_sync_config[serial_number]
            sync_config = device.get_multi_device_sync_config()
            sync_config.mode = self.sync_mode_from_str(sync_config_json["config"]["mode"])
            sync_config.color_delay_us = sync_config_json["config"]["color_delay_us"]
            sync_config.depth_delay_us = sync_config_json["config"]["depth_delay_us"]
            sync_config.trigger_out_enable = sync_config_json["config"]["trigger_out_enable"]
            sync_config.trigger_out_delay_us = sync_config_json["config"]["trigger_out_delay_us"]
            sync_config.frames_per_trigger = sync_config_json["config"]["frames_per_trigger"]
            device.set_multi_device_sync_config(sync_config)
            try:
                profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile: VideoStreamProfile = profile_list.get_default_video_stream_profile()
                config.enable_stream(color_profile)
                profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                depth_profile = profile_list.get_default_video_stream_profile()
                config.enable_stream(depth_profile)

                self.pipelines.append(pipeline)
                self.configs.append(config)
                self.serial_number_list[i] = serial_number
            except OBError as e:
                print(f"setup_cameras error:{e}")

    def start_streams(self):
        print(self.serial_number_list)
        for index, (pipeline, config, serial) in enumerate(zip(self.pipelines, self.configs, self.serial_number_list)):
            pipeline.start(
                config,
                lambda frame_set, curr_serial=serial: self.on_new_frame_callback(
                    frame_set, curr_serial
                ),
            )

    def stop_streams(self):
        self.mutex.lock()
        try:
            for pipeline in self.pipelines:
                pipeline.stop()
            self.pipelines = []
            self.configs = []
            print("device stopped")
        finally:
            self.mutex.unlock()

    def on_new_frame_callback(self, frames: FrameSet, serial_number: str):
        self.mutex.lock()
        try:
            global MAX_QUEUE_SIZE
            if serial_number not in self.color_frames_queue:
                print(f"⚠️ WARN: 未识别的相机序列号 {serial_number}，跳过帧处理")
                return
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is not None:
                if self.color_frames_queue[serial_number].qsize() >= MAX_QUEUE_SIZE:
                    self.color_frames_queue[serial_number].get()
                self.color_frames_queue[serial_number].put(color_frame)
            if depth_frame is not None:
                if self.depth_frames_queue[serial_number].qsize() >= MAX_QUEUE_SIZE:
                    self.depth_frames_queue[serial_number].get()
                self.depth_frames_queue[serial_number].put(depth_frame)
        finally:
            self.mutex.unlock()

    def rendering_frame(self, max_wait=5):
        image_dict: dict[str, np.ndarray] = {}
        start_time = time.time()
        color_width, color_height = None, None
        while len(image_dict) != self.curr_device_cnt:
            if time.time() - start_time > max_wait:
                print("⚠️ WARN: 渲染超时，部分相机未收到帧数据")
                break
            for serial_number in self.color_frames_queue.keys():
                color_frame = None
                if not self.color_frames_queue[serial_number].empty():
                    color_frame = self.color_frames_queue[serial_number].get()
                if color_frame is None:
                    continue
                color_width, color_height = color_frame.get_width(), color_frame.get_height()
                color_image = frame_to_bgr_image(color_frame)
                image_dict[serial_number] = color_image
        if len(image_dict) == self.curr_device_cnt and color_width is not None and color_height is not None:
            result_image = np.hstack(list(image_dict.values()))
            result_image = cv2.resize(result_image, (color_width, color_height))
            return result_image
        return image_dict
    def sync_mode_from_str(self, sync_mode_str: str) -> OBMultiDeviceSyncMode:

        # to lower case
        sync_mode_str = sync_mode_str.upper()
        if sync_mode_str == "FREE_RUN":
            return OBMultiDeviceSyncMode.FREE_RUN
        elif sync_mode_str == "STANDALONE":
            return OBMultiDeviceSyncMode.STANDALONE
        elif sync_mode_str == "PRIMARY":
            return OBMultiDeviceSyncMode.PRIMARY
        elif sync_mode_str == "SECONDARY":
            return OBMultiDeviceSyncMode.SECONDARY
        elif sync_mode_str == "SECONDARY_SYNCED":
            return OBMultiDeviceSyncMode.SECONDARY_SYNCED
        elif sync_mode_str == "SOFTWARE_TRIGGERING":
            return OBMultiDeviceSyncMode.SOFTWARE_TRIGGERING
        elif sync_mode_str == "HARDWARE_TRIGGERING":
            return OBMultiDeviceSyncMode.HARDWARE_TRIGGERING
        else:
            raise ValueError(f"Invalid sync mode: {sync_mode_str}")
    def read_config(self, config_file: str):
        global multi_device_sync_config
        with open(config_file, "r") as f:
            config = json.load(f)
        for device in config["devices"]:
            multi_device_sync_config[device["serial_number"]] = device
            print(f"Device {device['serial_number']}: {device['config']['mode']}")

class run_main_windows(QWidget):
    def __init__(self):
        super().__init__()
        self.create_widget()
        self.robot=ROBOT()
        self.camera=CAMERA_HOT_PLUG()
        self.gpcontrol = GPCONTROL()
        self.generator_hdf5=GENERATOR_HDF5()
        self.action_plan=ACTION_PLAN(self.robot)
        self.gpcontrol.gp_control_state_signal.connect(self.handle_feedback)  # 连接信号到槽函数
        self.gpcontrol.error_signal.connect(self.handle_error_signal)
        self.action_plan.action_signal.connect(self.handle_action_signal)
        self.action_plan.complete_signal.connect(self.handle_complete_signal)
        self.stop_render=True
        self.image:dict[str,np.array]={}
        self.images_dict = {cam_name: [] for cam_name in camera_names}  # 用于存储每个相机的图片
        self.qpos_list=[]
        self.action_list=[]
        self.gpstate_list=[]
        self.gppos_list=[]
        self.gpforce_list=[]
        self.gpstate=[]
        self.max_episode_len=500
        self.data_dict:dict[str,np.array]={}
        self.index=0
        self.index_length=10
        self.start_pos,self.local_desktop_pos,self.goal_pos=None,None,None
        

    def create_widget(self):
        self.setWindowTitle("ACT GET DATA")
        screen = app.primaryScreen()  # 获取主屏幕
        size = screen.geometry()  # 获取屏幕几何信息
        screen_width = size.width()
        screen_height = size.height()
        window_width = 1280
        window_height = 1080

        # 创建定时器
        self.camear_timer = QTimer()
        self.camear_timer.timeout.connect(self.updata_frame)
        self.task_timer = QTimer()
        self.task_timer.timeout.connect(self.updata_task)

        # 主布局管理器
        layout = QVBoxLayout()

        # 设置窗口位置和大小
        self.setGeometry(0, 0, window_width, window_height)
        self.move((screen_width - window_width) // 2, (screen_height - window_height) // 2)

        # 创建视频显示区域
        self.label = QLabel(' ', self)
        self.label.setFixedSize(window_width, int(window_height*0.7))
        layout.addWidget(self.label)

        # 创建标签显示欢迎信息
        # self.label_ = QLabel('Hello, PyQt5!', self)
        # layout.addWidget(self.label_)

        # 创建选择框与输入框布局
        form_layout = QFormLayout()
        self.combo_box = QComboBox()
        self.combo_box.addItems(["start", "local_desktop", "goal"])  # 添加选项
        form_layout.addRow("Select:", self.combo_box)

        self.input_box = QLineEdit()
        self.input_box.setPlaceholderText("Input position")
        form_layout.addRow("Data:", self.input_box)

        layout.addLayout(form_layout)

        # 设置按钮
        button_layout = QHBoxLayout()

        self.setting_btn = QPushButton("Set Point", self)
        self.setting_btn.clicked.connect(self.on_setting_btn_click)
        button_layout.addWidget(self.setting_btn)

        self.get_robot_state_btn = QPushButton("Get Robot State", self)
        self.get_robot_state_btn.clicked.connect(self.on_get_robot_state_btn_click)
        button_layout.addWidget(self.get_robot_state_btn)

        self.start_button = QPushButton("Start Camera", self)
        self.start_button.clicked.connect(self.start_camera)
        button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Close Camera", self)
        self.stop_button.clicked.connect(self.stop_camera)
        button_layout.addWidget(self.stop_button)

        layout.addLayout(button_layout)

        # Task-related buttons
        self.start_task = QPushButton("Start Task", self)
        self.start_task.clicked.connect(self.on_start_task_btn_click)
        layout.addWidget(self.start_task)

        # 进度条部分
        self.episode_index_box = QLineEdit()
        self.episode_index_box.setPlaceholderText("Set start episode_idx.., default: 0,10")
        layout.addWidget(QLabel("Index:"))
        layout.addWidget(self.episode_index_box)

        self.set_index = QPushButton("Set Index", self)
        self.set_index.clicked.connect(self.on_set_index_btn_click)
        layout.addWidget(self.set_index)
        self.stop_emergency=QPushButton("STOP",self)
        self.stop_emergency.clicked.connect(self.on_stop_emergency_btn_click)
        layout.addWidget(self.stop_emergency)
        # 结果显示
        self.result_label = QLabel("")
        layout.addWidget(self.result_label)

        # 进度条
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setValue(0)
        layout.addWidget(self.progress_bar)

        # 设置窗口的布局
        self.setLayout(layout)
        print("控件初始化完成")
    def on_setting_btn_click(self):
        # print("按钮被点击了!")
        selected_option = self.combo_box.currentText()  # 获取下拉框的当前选项
        input_text = self.input_box.text()  # 获取输入框的文本
        if selected_option =='start' and input_text !='':
            self.start_pos = json.loads(input_text)
        elif selected_option == 'local_desktop' and input_text !='':
            self.local_desktop_pos = json.loads(input_text)
        elif selected_option == 'goal' and input_text !='':
            self.goal_pos =  json.loads(input_text)
        else:
            self.label_.setText("input is none or pose error")
        self.result_label.setText(f"select: {selected_option},input: {input_text}")
    def on_get_robot_state_btn_click(self):
        self.gpcontrol.start()
        # self.joint_pos = self.robot.rm_65_b_right_arm.rm_get_current_arm_state()[1]['joint']
        self.joint_pos = self.robot.Client.get_arm_postion_joint()
        # self.pose = self.robot.rm_65_b_right_arm.rm_get_current_arm_state()[1]['pose']
        self.pose = self.robot.Client.get_arm_postion_pose()
        self.input_box.setText(json.dumps(self.pose))
        self.result_label.setText(json.dumps(self.robot.rm_65_b_right_arm.rm_get_current_arm_state()[0]))
    def on_start_task_btn_click(self):
        time.sleep(0.5)
        if self.start_pos is None:
            self.result_label.setText("start_pos is None")
        elif self.local_desktop_pos is None:
            self.result_label.setText("local_desktop_pos is None")
        elif self.goal_pos is None:
            self.result_label.setText("goal_pos is None")
        else:
            self.action_plan.val(self.start_pos,self.goal_pos,self.local_desktop_pos)
            self.action_plan.start()
            self.task_timer.start(30)  # 每 50ms 触发一次
            self.progress_value = 0  # 复位进度
    def on_stop_emergency_btn_click(self):
        self.robot.rm_65_b_right_arm.rm_set_arm_stop()
        self.robot.rm_65_b_right_arm.rm_clear_system_err()
        self.robot.rm_65_b_right_arm.rm_set_delete_current_trajectory()
        self.task_timer.stop()
        self.camear_timer.stop()
        self.gpcontrol.stop()
        self.action_plan.stop()
        pass
    def updata_task(self):
        start_time=time.time()
        self.progress_value += 1
        self.progress_bar.setValue(self.progress_value)
        angle_qpos=self.robot.get_state()
        radius_qpos = [math.radians(j) for j in angle_qpos]
        # radius_qpos.append(self.robot.rm_65_b_right_arm.rm_get_tool_voltage()[1])
        mid_time=time.time()
        if self.gpstate !=[]:
            gpdata=self.gpstate
            gpstate,gppos,gpforce = gpdata
            if not isinstance(gpstate, str):
                gpstate = str(gpstate)
            if not isinstance(gppos, str):
                gppos = str(gppos)
            if not isinstance(gpforce, str):
                gpforce = str(gpforce)
            radius_qpos.append(np.array(int(gpstate, 16), dtype='int32'))
            radius_qpos.append(np.array(int(gppos, 16), dtype='int32'))
            radius_qpos.append(np.array(int(gpforce, 16), dtype='int32'))
        self.qpos_list.append(radius_qpos)
        if self.image !={}:
            self.images_dict['top'].append(self.image['top'])
            self.images_dict['right_wrist'].append(self.image['right_wrist'])
        if COMPLETED==True and self.index >= self.index_length:
            print('task completed')
            self.action_plan.stop()
            self.task_timer.stop()
        elif COMPLETED == True:
            self.save_data()
            print("completed data collection")
        end_time=time.time()
        time_=end_time-start_time
        print(f"\033[31mend_time-start_time:{time_}\nmid_time:{end_time-mid_time}\nmid-start:{mid_time-start_time}\033[0m")
    def handle_feedback(self,feed_back):
        self.gpstate=feed_back
    def handle_action_signal(self,action_feed_back):
        self.gpcontrol.set_state_flag(action_feed_back)
    def handle_complete_signal(self,complete_feed_back):
        global COMPLETED
        COMPLETED=complete_feed_back
        print("接受完成信号")
    def handle_error_signal(self,error_feed_back):
        print(error_feed_back)
        self.result_label.setText(error_feed_back)
    def save_data(self):
        self.action_list = self.qpos_list
        if self.qpos_list is not None:
            # self.qpos_list = np.vstack([self.qpos_list[0], self.qpos_list])
            self.qpos_array = np.vstack([self.qpos_list[0], self.qpos_list])  # 存入一个新变量

        else:
            raise "qpos is none"
        self.data_dict = {
            '/observations/qpos': self.qpos_array[:self.max_episode_len],
            '/action': self.action_list[:self.max_episode_len],
        }
        for cam_name in camera_names:
            self.data_dict[f'/observations/images/{cam_name}'] = self.images_dict[cam_name][:self.max_episode_len]
        self.generator_hdf5.save_hdf5(self.data_dict,"./hdf5_file",self.index)
        self.index+=1
        if self.index>self.index_length:
            self.result_label.setText(f"task over please restart or close")
    def on_set_index_btn_click(self):
        index=self.episode_index_box.text()
        print(index)
        if not re.match(r"^\d+,\d+$", index):
            self.result_label.setText(f"index error")
        else:
            index_len = list(map(int, index.split(',')))
            self.index_length=index_len[1]
            self.index=index_len[0]
    def start_camera(self):
        """启动摄像头"""
        self.stop_render =False
        self.camear_timer.start()

    def updata_frame(self):
        """更新摄像头图像"""
        frame_data = self.camera.rendering_frame()
        serial_number_list = self.camera.serial_number_list

        # 判断 frame_data 的类型
        if isinstance(frame_data, dict):  # 多台摄像头返回字典 {str: np.ndarray}
            if not frame_data:  # 字典为空
                print("⚠️ WARN: 没有接收到任何摄像头图像")
                return
            if all(img.size == 0 for img in frame_data.values()):  # 所有相机的图像都是空的
                print("⚠️ WARN: 所有摄像头的图像数据为空")
                return
        elif isinstance(frame_data, np.ndarray):  # 只有一台相机
            if frame_data.size == 0:
                print("⚠️ WARN: 没有接收到任何摄像头图像")
                return
            # 只有一个摄像头时，将其存入字典，模拟多摄像头格式
            frame_data = {"0": frame_data}  
            serial_number_list = ["0"]
        else:
            print(f"⚠️ ERROR: 无效的 frame_data 类型: {type(frame_data)}")
            return

        num_images = len(frame_data)

        if serial_number_list:
            first_key = str(serial_number_list[0])
            if first_key not in frame_data:
                print(f"⚠️ WARN: 摄像头 {first_key} 的图像数据缺失")
                return

            # 获取第一台摄像头的图像
            result_image = frame_data[first_key]

            # 拼接其他摄像头图像
            for sn in serial_number_list[1:]:
                sn_str = str(sn)
                if sn_str in frame_data:
                    result_image = np.hstack((result_image, frame_data[sn_str]))

            # 显示合成后的图像
            self.display_image(result_image)

        # 存储图像到 self.image
        self.image['top'] = frame_data.get(str(serial_number_list[0]), None)
        self.image['right_wrist'] = frame_data.get(str(serial_number_list[1]), None) if num_images > 1 else None


    def display_image(self,result_image):
        
        # **显示图像**
        if self.stop_render is False:
            color_height, color_width, ch = result_image.shape
            # print(result_image.shape)
            qt_img = QImage(result_image, color_width, color_height, ch * color_width, QImage.Format_RGB888)
            qimage = qt_img.rgbSwapped()
            self.label.setPixmap(QPixmap.fromImage(qimage))
    def stop_camera(self):
        """关闭摄像头"""
        self.stop_render=True
        # self.timer.stop()
        self.label.clear()
        # self.camera.stop_straems()
    def closeEvent(self, event):
        """关闭窗口时释放摄像头"""
        self.stop_camera()
        self.action_plan.stop()
        self.camear_timer.stop()
        self.task_timer.stop()
        self.camera.stop_straems()
        self.gpcontrol.close()
        self.gpcontrol.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication([])
    # 创建并显示窗口
    window = run_main_windows()
    window.show()
    # 启动事件循环
    app.exec_()