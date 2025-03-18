import json
import os
import time
from queue import Queue
from threading import Lock
from tkinter import NO
from traceback import print_tb
from Robotic_Arm.rm_robot_interface import *
import math, h5py
import cv2
import numpy as np
from tqdm import tqdm
from pyorbbecsdk import *
# from deploy.remote_control import posRecorder
from utils import frame_to_bgr_image
from pynput.keyboard import Listener, Key
from pynput import keyboard
import random
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QLineEdit,QPushButton, QLabel, QVBoxLayout,QProgressBar
from PyQt5.QtGui import QScreen
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer,QThread, pyqtSignal
from collections import deque
from concurrent.futures import ThreadPoolExecutor

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
GPCONTROL_COMMAND:int=1
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
    gp_control_state_signal = pyqtSignal(object)  # 反馈信号，用于 UI 连接
    def __init__(self, parent=None, DEFAULT_SERIAL_PORT = "/dev/ttyACM1"):
        super().__init__(parent)
        global GPCONTROL_COMMAND
        print("global GPCONTROL_COMMAND",GPCONTROL_COMMAND)
        self.state_flag = GPCONTROL_COMMAND  # 夹爪状态: 0=关, 1=半开, 2=开
        self.running = True  # 控制线程运行
        self.control_command = ""  # 当前控制命令
        self.DEFAULT_SERIAL_PORT = DEFAULT_SERIAL_PORT
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
        """打开串口"""
        port = self.DEFAULT_SERIAL_PORT
        baudrate = self.BAUD_RATE
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            print(f"串口 {port} 已打开，波特率 {baudrate}")
            return ser
        except Exception as e:
            print(f"无法打开串口 {port}: {e}")
            return None
    
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

    # def get_gp_state(self):
    #     data = self.read_data() 
    #     if data is not None:
    #         _, gpdata = data
    #         while gpdata == 0:
    #             self.send_can_data(b'\x00\x00\x00\x01', half_open_gp, 0x01)
    #             data = self.read_data()
    #             if data is not None:
    #                 _, gpdata = data
    #         gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
    #         return [gpstate,gppos,gpforce]
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
        self.rm_65_b_right_arm = (RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E))
        self.arm_ini = self.rm_65_b_right_arm.rm_create_robot_arm("192.168.1.18",8080, level=3)
        # print(self.arm_ini)
        # self.robot_controller = RoboticArm("192.168.1.18", 8080, 3)
    def get_state(self, model='joint'):#pose
        self.joint_state_right = self.rm_65_b_right_arm.rm_get_current_arm_state()
        # print(f"get state test", self.joint_state_right)
        return_action = self.joint_state_right[1][model]
        # print(return_action)
        return return_action

class ACTION_PLAN(QThread):
    action_signal = pyqtSignal(object)
    complete_signal = pyqtSignal(object)
    def __init__(self,Robot:ROBOT):
        super().__init__()
        global GPCONTROL_COMMAND
        print(f"action_plan global gpcontrol command {GPCONTROL_COMMAND}")
        self.running = True  # 线程运行状态
        self.Robot=Robot
        self.velocity = 15
        self.gpstate:list
        self.start_point=None
        self.goal_point = None
        self.local_desktop_point = None
        self.loop_len=1
    def val(self,start_point,goal_point,local_desktop_point):
        self.start_point=start_point
        self.goal_point = goal_point
        self.local_desktop_point = local_desktop_point
    def move(self,position,up=False):
        if up is False:
            self.Robot.rm_65_b_right_arm.rm_movej_p(position,self.velocity,0,0,1)
        else:
            position_=position.copy()
            position_[1]=position_[1]-0.1
            self.Robot.rm_65_b_right_arm.rm_movej_p(position_,self.velocity,0,0,1)
    def run(self):
        while self.running:
            # print(self.running)
            self.run_thread()
            self.back_thread()
        
    def run_thread(self):
        index=0
        # print(self.start_point,self.local_desktop_point,self.local_desktop_point)
      
        # print("GPCONTROL_COMMAND",GPCONTROL_COMMAND)
        if self.start_point is not  None or self.local_desktop_point is not None or self.goal_point is not None:           
            self.move(self.start_point)
            GPCONTROL_COMMAND=2
            self.action_signal.emit(GPCONTROL_COMMAND)
            # state = self.gpcontrol.open_all_gp()
            # self.gpstate.append(state)
            self.move(self.local_desktop_point,up=True)
            self.move(self.local_desktop_point)
            # self.gpcontrol.close_gp()
            GPCONTROL_COMMAND=0
            # time.sleep(10)
            self.action_signal.emit(GPCONTROL_COMMAND)
            self.move(self.local_desktop_point,up=True)
            self.move(self.goal_point ,up=True)
            self.move(self.goal_point )
            GPCONTROL_COMMAND=1
            self.action_signal.emit(GPCONTROL_COMMAND)
            # self.gpcontrol.open_all_gp()
            self.move(self.goal_point ,up=True)
            self.move(self.start_point)
            COMPLETED=True
            self.complete_signal.emit(COMPLETED)
            print("发送完成信号")
            time.sleep(2)
            index+=1
        else:
            raise ValueError (f"no point is set")
    def back_thread(self):
        self.move(self.goal_point ,up=True)
        GPCONTROL_COMMAND=2
        self.action_signal.emit(GPCONTROL_COMMAND)
        self.move(self.goal_point)
        GPCONTROL_COMMAND=0
        self.action_signal.emit(GPCONTROL_COMMAND)
        self.move(self.goal_point ,up=True)
        self.move(self.start_point)
        self.move(self.local_desktop_point,up=True)
        self.move(self.local_desktop_point,up=False)
        GPCONTROL_COMMAND=2
        self.action_signal.emit(GPCONTROL_COMMAND)
        self.move(self.local_desktop_point,up=True)
        self.move(self.start_point)
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

class CAMEAR():
    def __init__(self):
        ctx = Context()
        self.device_list = ctx.query_devices()
        self.curr_device_cnt = self.device_list.get_count()
        self.pipelines: list[Pipeline] = []
        self.configs: list[Config] = []
        self.serial_number_list:list[str] = ["" for _ in range(self.curr_device_cnt) ]
        self.color_frames_queue: list[Queue] = [Queue() for _ in range(self.curr_device_cnt)]
        self.depth_frames_queue: list[Queue] = [Queue() for _ in range(self.curr_device_cnt)]
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
            pipeline = Pipeline(device)
            config = Config()
            serial_number = device.get_device_info().get_serial_number()
            self.serial_number_list[i] = serial_number
            sync_config_json = multi_device_sync_config[serial_number]
            sync_config = device.get_multi_device_sync_config()
            sync_config.mode = self.sync_mode_from_str(sync_config_json["config"]["mode"])
            sync_config.color_delay_us = sync_config_json["config"]["color_delay_us"]
            sync_config.depth_delay_us = sync_config_json["config"]["depth_delay_us"]
            sync_config.trigger_out_enable = sync_config_json["config"]["trigger_out_enable"]
            sync_config.trigger_out_delay_us = sync_config_json["config"]["trigger_out_delay_us"]
            sync_config.frames_per_trigger = sync_config_json["config"]["frames_per_trigger"]
            # print(f"Device {serial_number} sync config: {sync_config}")
            device.set_multi_device_sync_config(sync_config)
            try:
                profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile:VideoStreamProfile = profile_list.get_default_video_stream_profile()
                config.enable_stream(color_profile)
                profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                depth_profile = profile_list.get_default_video_stream_profile()
                config.enable_stream(depth_profile)
                self.pipelines.append(pipeline)
                self.configs.append(config)
            except OBError as e :
                print(f"setup_cameras error:{e}")

    def start_streams(self):
        # print(type(self.pipelines),type(self.configs),self.configs,self.curr_device_cnt)
        index = 0
        # print(self.serial_number_list)
        for pipeline, config in zip(self.pipelines,self.configs):
            # print(f"index{index}")
            pipeline.start(
                config,
                lambda frame_set, curr_index=self.serial_number_list[index]:self.on_new_frame_callback(
                    frame_set, curr_index
                ),
            )
            index+=1
    def stop_straems(self):
        for pipeline in self.pipelines:
            pipeline.stop()
        self.pipelines=[]
        self.configs=[]
        print("device stoped")
    def on_new_frame_callback(self,frames: FrameSet, serial_number: str):
        global MAX_QUEUE_SIZE
        # self.frames=frames
        # print(serial_number)
        if serial_number == camera_sn['right_wrist']:
            index=0
        elif serial_number == camera_sn['top']:
            index=1
        elif serial_number == camera_sn['left_wrist']:
            index=2
        else:
            # print(serial_number)
            raise ValueError("变量错误：on frame callback 相机sn未在camera sn中")
        # print(frames.get_data())
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is not None:
            if self.color_frames_queue[index].qsize() >= MAX_QUEUE_SIZE:
                self.color_frames_queue[index].get()
            self.color_frames_queue[index].put(color_frame)
        if depth_frame is not None:
            # print(self.depth_frames_queue)
            if self.depth_frames_queue[index].qsize() >= MAX_QUEUE_SIZE:
                self.depth_frames_queue[index].get()
            self.depth_frames_queue[index].put(depth_frame)
    
    def rendering_frame(self):
        image_list: dict[int, np.ndarray] = {}
        while len(image_list)!=self.curr_device_cnt:  # 直接判断字典长度
            for i in range(self.curr_device_cnt):
                # print(self.curr_device_cnt)
                color_frame = None
                depth_frame = None
                if not self.color_frames_queue[i].empty():
                    color_frame = self.color_frames_queue[i].get()
                # else:
                #     print("color_frames_queue is empty")
                if not self.depth_frames_queue[i].empty():
                    depth_frame = self.depth_frames_queue[i].get()
                # else:
                #     print("depth_frames_queue is empty")
                if color_frame is None and depth_frame is None:
                    # print("color_frame is None and depth_frame is None")
                    continue
                
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
                        print("depth format is not Y16")
                        continue
                    depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
                    depth_data = (depth_data.astype(np.float32) * scale).astype(np.uint8)
                    depth_image = cv2.applyColorMap(cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX), cv2.COLORMAP_JET)

                if color_image is not None:
                    image_list[i] = color_image

            # 确保 `image_list` 不会超过 `curr_device_cnt`
            if len(image_list) > self.curr_device_cnt:
                image_list.clear()
                continue
            # 拼接所有图像
            if len(image_list) == self.curr_device_cnt:
                result_image = np.hstack(list(image_list.values()))
                result_image = cv2.resize(result_image, (color_width, color_height))  # 保持一致的尺寸
                break
            # print(image_list)
        return image_list
   
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
        self.camera=CAMEAR()
        self.gpcontrol = GPCONTROL()
        self.generator_hdf5=GENERATOR_HDF5()
        self.action_plan=ACTION_PLAN(self.robot)
        self.gpcontrol.gp_control_state_signal.connect(self.handle_feedback)  # 连接信号到槽函数
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
        screen_width=size.width()
        screen_height=size.height()
        window_width=1920
        window_height=640

        # 创建定时器
        self.camear_timer = QTimer()
        self.camear_timer.timeout.connect(self.updata_camera)
        self.task_timer=QTimer()
        self.task_timer.timeout.connect(self.updata_task)

        # 布局管理器# 创建垂直布局
        layout = QVBoxLayout()
        self.setGeometry(1080, 1920, window_width, window_height)
        self.move((screen_width - window_width) // 2, (screen_height - window_height) // 2)

        # 创建 QLabel 组件用于显示视频
        self.label = QLabel(' ',self)
        self.label.setFixedSize(1920, 720)
        layout.addWidget(self.label)

        self.label_ = QLabel('Hello, PyQt5!', self)
        layout.addWidget(self.label_)

        # 创建下拉框（QComboBox）
        self.combo_box = QComboBox()
        self.combo_box.addItems(["start", "local_desktop", "goal"])  # 添加选项
        layout.addWidget(QLabel("select:"))  # 标签
        layout.addWidget(self.combo_box)

        # 创建输入框（QLineEdit）
        self.input_box = QLineEdit()
        self.input_box.setPlaceholderText("txt..")  # 设置占位符
        layout.addWidget(QLabel("data:"))  # 标签
        layout.addWidget(self.input_box)

        # setting按钮
        self.setting_btn = QPushButton("set_point", self)
        self.setting_btn.resize(self.setting_btn.sizeHint())  # 自动调整按钮大小
        self.setting_btn.clicked.connect(self.on_setting_btn_click)  # 连接按钮的点击事件
        layout.addWidget(self.setting_btn)

        #get robot state
        self.get_robot_state_btn = QPushButton("get robot state", self)
        self.get_robot_state_btn.resize(self.get_robot_state_btn.sizeHint())  # 自动调整按钮大小
        self.get_robot_state_btn.clicked.connect(self.on_get_robot_state_btn_click)  # 连接按钮的点击事件
        layout.addWidget(self.get_robot_state_btn)

        # 创建按钮
        self.start_button = QPushButton("start camera", self)
        self.stop_button = QPushButton("close camera", self)
        # 绑定按钮事件
        self.start_button.clicked.connect(self.start_camera)
        self.stop_button.clicked.connect(self.stop_camera)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        # 创建按钮
        self.start_task = QPushButton("start task", self)
        self.start_task.clicked.connect(self.on_start_task_btn_click)
        layout.addWidget(self.start_task)
        # 创建输入框（QLineEdit）
        self.episode_index_box = QLineEdit()
        self.episode_index_box.setPlaceholderText("set start episode_idx..,example:0,10")  # 设置占位符
        layout.addWidget(QLabel("index:"))  # 标签
        layout.addWidget(self.episode_index_box)
        # 创建按钮
        self.set_index = QPushButton("set index", self)
        self.set_index.clicked.connect(self.on_set_index_btn_click)
        layout.addWidget(self.set_index)
        # 创建标签显示结果
        self.result_label = QLabel("")
        layout.addWidget(self.result_label)
        # 创建进度条
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setValue(0)  # 初始值
        layout.addWidget(self.progress_bar)
        # 设置窗口的布局
        self.setLayout(layout)
        print("控件初始化完成")
    def on_setting_btn_click(self):
        # print("按钮被点击了!")
        selected_option = self.combo_box.currentText()  # 获取下拉框的当前选项
        input_text = self.input_box.text()  # 获取输入框的文本
        
        if selected_option =='start' and input_text !='':
            # print(input_text)
            self.start_pos = json.loads(input_text)

        elif selected_option == 'local_desktop' and input_text !='':
            self.local_desktop_pos = json.loads(input_text)

        elif selected_option == 'goal' and input_text !='':
            self.goal_pos =  json.loads(input_text)
        else:
            self.label_.setText("input is none or pose error")
        # print(self.start_pos,type(input_text))
        # self.action_plan.val(self.start_pos,self.goal_pos,self.local_desktop_pos)
        self.result_label.setText(f"select: {selected_option},input: {input_text}")
    def on_get_robot_state_btn_click(self):
        self.gpcontrol.start()
        # state = (1,{'joint':[0,1,0],'pose':[0,0,0]})
        self.joint_pos = self.robot.rm_65_b_right_arm.rm_get_current_arm_state()[1]['joint']
        self.pose = self.robot.rm_65_b_right_arm.rm_get_current_arm_state()[1]['pose']
        data = json.dumps(self.pose)
        self.input_box.setText(data)
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
      
    def updata_task(self):
        start_time=time.time()
        self.progress_value += 1
        self.progress_bar.setValue(self.progress_value)
        angle_qpos=self.robot.get_state()
        radius_qpos = [math.radians(j) for j in angle_qpos]
        radius_qpos.append(self.robot.rm_65_b_right_arm.rm_get_tool_voltage()[1])
        mid_time=time.time()
        # _,gpdata=self.gpcontrol().read_data()
        if self.gpstate !=[]:
            gpdata=self.gpstate
            # print(gpdata,type(gpdata))
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
            print("处理完成信号")
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

        # print(f"GPCONTROL_COMMAND{GPCONTROL_COMMAND}")
    def handle_complete_signal(self,complete_feed_back):
        global COMPLETED
        COMPLETED=complete_feed_back
        print("接受完成信号")
    def save_data(self):
        self.action_list = self.qpos_list
        if self.qpos_list is not None:
            self.qpos_list = np.vstack([self.qpos_list[0], self.qpos_list])
        else:
            raise "qpos is none"
        self.data_dict = {
            '/observations/qpos': self.qpos_list[:self.max_episode_len],
            '/action': self.action_list[:self.max_episode_len],
            # '/gp/gpstate': self.gpstate_list[:self.max_episode_len],
            # '/gp/gppos': self.gppos_list[:self.max_episode_len],
            # '/gp/gpforce': self.gpforce_list[:self.max_episode_len],            
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
        
        
        # self.camera.setup_cameras()
        # self.camera.start_streams()
    def updata_camera(self):
        image_list=self.camera.rendering_frame()
        # 获取字典的长度
        # print(image_list)
        if image_list !={}:
            num_images = len(image_list)

            # 如果字典中有多个图像，进行水平拼接
            if num_images > 1:
                result_image = image_list[0]
                for i in range(1, num_images):
                    result_image = np.hstack((result_image, image_list[i]))

                # 确保拼接后的图像大小不超过 1080x720
                result_image = cv2.resize(result_image, (min(result_image.shape[1], 1920), min(result_image.shape[0], 720)))
            else:
                # 如果字典中只有一张图像，直接使用该图像
                result_image = image_list[0]
                result_image = cv2.resize(result_image, (min(result_image.shape[1], 1920), min(result_image.shape[0], 720)))
            self.image['top']=image_list[0]
            self.image['right_wrist']=image_list[1]
            color_height, color_width, ch = result_image.shape
            # print(self.image)
            # # 转换为 QImage
            if self.stop_render is False:
                qt_img = QImage(result_image, color_width, color_height, ch*color_width, QImage.Format_RGB888)
                # 在 QLabel 上显示
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