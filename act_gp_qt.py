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
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QLineEdit,QPushButton, QLabel, QVBoxLayout, QFormLayout,QProgressBar,QHBoxLayout,QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer,QThread, pyqtSignal, QMutex
import threading



frames_queue_lock = Lock()

# Configuration settings
MAX_DEVICES = 3
MAX_QUEUE_SIZE = 2
save_points_dir = os.path.join(os.getcwd(), "point_clouds")
save_depth_image_dir = os.path.join(os.getcwd(), "depth_images")
save_color_image_dir = os.path.join(os.getcwd(), "color_images")
action_play:bool = False
# Load config file for multiple devices
config_file_path = os.path.join(os.path.dirname(__file__), "../config/multi_device_sync_config.json")
multi_device_sync_config = {}
camera_names = ['left_wrist','top','right_wrist']
save_signal = False
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
        self.task_complete = False
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
            data = ser.read(64)  # 读取最大 64 字节
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
    def __init__(self,robot_num=1):
        self.joint_state_right=None
        self.robot_num=robot_num
        if self.robot_num ==1:
            self.Client = PersistentClient('192.168.2.14', 8001)
        elif self.robot_num==2:
            self.Client = PersistentClient('192.168.3.15', 8002)
        # self.Client.set_close(robot_num)
        # self.Client.set_clear(robot_num)
        # self.Client.set_open(robot_num)
        # self.rm_65_b_right_arm = (RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E))
        # self.arm_ini = self.rm_65_b_right_arm.rm_create_robot_arm("192.168.1.18",8080, level=3)

    def get_state(self, model='joint',):#pose
        # self.joint_state_right = self.rm_65_b_right_arm.rm_get_current_arm_state()
        # return_action = self.joint_state_right[1][model]
        if model=='joint':
            action=self.Client.get_arm_position_joint(robotnum=self.robot_num)
        elif model=='pose':
            action = self.Client.get_arm_position_pose(robotnum=self.robot_num)
        
        return action
    def set_state(self, action, model='joint'):
        # self.rm_65_b_right_arm.rm_set_arm_position(action, model)
        if model=='joint':
            self.Client.set_arm_position(action,'joint',self.robot_num)
        elif model=='pose':
            self.Client.set_arm_position(action,'pose',self.robot_num)
        else:
            raise ValueError(f"model {model} is not support")
    def enable_power(self):
        self.Client.set_close(self.robot_num)
        time.sleep(1)
        self.Client.set_clear(self.robot_num)
        self.Client.set_open(self.robot_num)
    def stop(self):
        self.Client.set_stop(self.robot_num)
        self.Client.set_reset(self.robot_num)
    
        # self.rm_65_b_right_arm.rm_set_stop()

class ACTION_PLAN(QThread):
    action_signal = pyqtSignal(object)
    complete_signal = pyqtSignal(object)
    traja_reverse_signal = pyqtSignal(object)
    close_signal = pyqtSignal(object)
    def __init__(self):
        super().__init__()
        self.running = True  # 线程运行状态
        self.Robot=ROBOT(robot_num=1)
        self.velocity = 15
        self.gpstate:list
        self.point=None
        self.goal_point = None
        self.local_desktop_point = None
        self.loop_len=1
        self.complete =False
        self.points = [
                    #    [-66.5918, -480.683, 341.961, 2.36635, -0.0480989, 1.43767],#第一版
                    #    [-134.102, -541.192, 2.5797, 2.36486, 0.0714842, 1.43293],
                    #    [-143.225, -602.043, -69.8797, 2.34887, 0.0679714, 1.51996],
                    #    [-143.773, -654.774, -133.036, 2.34891, 0.0677073, 1.52],
                    #    [-122.714, -653.022, -132.788, 2.34881, 0.0675977, 1.52006],
                    #    [-66.5918, -480.683, 341.961, 2.36635, -0.0480989, 1.43767],#第二版
                    #    [-95.0238, -588.402, 124.545, 2.35329, -0.0531227, 1.43677],
                    #    [-109.613, -579.638, -56.7167, 1.94239, -0.225811, 1.37319],
                    #    [-139.921, -544.635, -183.422, 1.97974, 0.0461738, 1.43546],
                    #    [-137.118, -601.257, -208.017, 1.98002, 0.0460499, 1.4354],
                    #    [-121.42, -599.741, -209.687, 1.98009, 0.0459801, 1.43546], 
                    [-127.182, -740.602, -84.8895, 2.09826, -0.0342467, 1.66033],#第三版
                    [-122.838, -800.168, -196.384, 2.12972, 0.0466091, 1.43007],
                    [-126.263, -801.374, -296.735, 2.12947, 0.0464736, 1.42999],
                    [-110.854, -801.413, -296.686, 2.12938, 0.0463978, 1.43004],
                    

                       ]  # 存储所有点位
    def val(self, point):
        self.point = point
  
    def move(self,position,up=False):
        if up is False:
            # self.Robot.rm_65_b_right_arm.rm_movej_p(position,self.velocity,0,0,1)
            self.Robot.set_state(position,'pose')
        else:
            position_=position.copy()
            position_[1]=position_[1]-0.1
            # self.Robot.rm_65_b_right_arm.rm_movej_p(position_,self.velocity,0,0,1)
            self.Robot.set_state(position_,'pose')
    def run(self):
        global action_play
        while self.running:
            # print(action_play)
            if action_play :
                self.move(self.point)
                # action_play = False
            else:
                if save_signal == False:
                    self.traja() 
                    self.traja_reverse()
                if not self.running:
                    break
    def traja(self):
        if not hasattr(self, "points") or not self.points:
            raise ValueError("请先设置至少一个点位")

        self.complete_signal.emit(False)
        for idx, point in enumerate(self.points):
            
            if self.is_close(self.Robot.get_state(model='pose'), point, tolerance=0.1):
                continue
            if point == self.points[3]:
                self.close_signal.emit(True)
            if point is not None:
                # 判断是否是后三个点
                if idx >= len(self.points) - 2:
                    self.move(point)  
                else:
                    rand_point = self.random_positon(point)
                    print(rand_point)
                    self.move(rand_point)

            if not self.running:
                self.stop()
        time.sleep(1)
        # self.complete_signal.emit(True)
        self.close_signal.emit(False)
        print("发送 action_plan 完成信号")
        # time.sleep(2)

    def traja_reverse(self):
        if not hasattr(self, "points") or not self.points:
            raise ValueError("请先设置至少一个点位")

        self.traja_reverse_signal.emit(True)

        # 逆序遍历 `self.points`，让机器人按原轨迹返回
        for idx, point in enumerate(reversed(self.points)):
            if self.is_close(self.Robot.get_state(model='pose'), point, tolerance=0.1):
                continue
            
            if point is not None:
                self.move(point)
            if not self.running:
                self.stop()
        self.traja_reverse_signal.emit(False)
        self.complete_signal.emit(True)

        print("发送 action_plan 逆向完成信号")
        time.sleep(2)

    def is_close(self, actual, target, tolerance=0.1):
        """
        判断两个列表的每个元素是否在允许误差范围内
        :param actual: 实际值列表（如当前机械臂状态）
        :param target: 目标值列表
        :param tolerance: 允许的最大误差（绝对值）
        :return: 所有元素均满足误差要求返回True，否则False
        """
        # 处理None和长度检查
        if actual is None or target is None:
            return False
        if len(actual) != len(target):
            return False
        
        # 逐个元素比较误差
        for a, t in zip(actual, target):
            # print(abs(a - t))
            if abs(a - t) > tolerance:
                return False
        return True

    def random_positon(self,point,a=25,b=30):
        random_pos = point.copy()
        random_pos[0] += random.uniform(a,b)  # 生成 1.5 到 3.5 之间的随机浮点数
        random_pos[1] += random.uniform(a,b)  # 生成 1.5 到 3.5 之间的随机浮点数
        random_pos[2] += random.uniform(a,b)  # 生成 1.5 到 3.5 之间的随机浮点
        # random_pos[3] += random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点数
        # random_pos[4] += random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点数
        # random_pos[5] += random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点
        return random_pos
    def set_loop_len(self,value):
        self.loop_len=value
    def stop(self):
        """ 停止线程 """
        self.running = False
        
        # self.Robot.rm_65_b_right_arm.rm_set_delete_current_trajectory()
        self.quit()
        # self.wait()

class GENERATOR_HDF5:
    def __init__(self):
        pass
    def save_hdf5(self,data_dict,path_to_save_hdf5,episode_idx,compressed=True):
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
                                compression="gzip", 
                                compression_opts= 4 if compressed else 0
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
        self.running_device = True
        self.change_signal = False
        print("相机初始化完成")
        self.monitor_thread = threading.Thread(target=self.monitor_devices, daemon=True)
        self.monitor_thread.start()
        
    def monitor_devices(self):
        while self.running_device:
            time.sleep(2)
            new_device_list = self.ctx.query_devices()
            new_device_cnt = new_device_list.get_count()
            if  new_device_cnt!= self.curr_device_cnt:
                print("设备变化检测到，重新初始化相机...")
                self.change_signal = True
                self.stop_streams()
                self.device_list = new_device_list
                self.curr_device_cnt = new_device_cnt
                self.setup_cameras()
                self.start_streams()
                self.change_signal = False
    def stop(self):
        self.running_device=False
        self.monitor_thread.join()
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
        print("stop camera -ing")

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

        return image_dict,color_width, color_height
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
        self.robot=ROBOT(robot_num=1)
        self.camera=CAMERA_HOT_PLUG()
        self.gpcontrol = GPCONTROL()
        self.generator_hdf5=GENERATOR_HDF5()
        self.action_plan=ACTION_PLAN()
        self.gpcontrol.gp_control_state_signal.connect(self.handle_feedback)  # 连接信号到槽函数
        self.gpcontrol.error_signal.connect(self.handle_error_signal)
        self.action_plan.action_signal.connect(self.handle_action_signal)
        self.action_plan.complete_signal.connect(self.handle_complete_signal)
        self.action_plan.traja_reverse_signal.connect(self.handle_traja_reverse_signal)
        self.action_plan.close_signal.connect(self.handle_close_signal)
        self.stop_render=True
        self.image:dict[str,np.array]={}
        self.images_dict = {cam_name: [] for cam_name in camera_names}  # 用于存储每个相机的图片
        self.qpos_list=[]
        self.action_list=[]
        self.gpstate_list=[]
        self.gppos_list=[]
        self.gpforce_list=[]
        self.gpstate=[]
        self.max_episode_len=5000

        self.start_index = 0    
        self.index=self.start_index
        self.task_complete_step = 0
        self.end_index=50
        self.start_pos,self.local_desktop_pos,self.goal_pos=[-9.19798, -84.536, 631.215, 1.42741, -0.0901151, 2.83646],[-98.2562, -0.131311, 30.7511, 3.35265, -26.1317, 72.2164],[-99.3707, -82.4347, 65.0209, 8.46747, -53.2068, 69.0324]
        self.complete_sign = False
        self.traja_reverse_signal = False
        self.close_signal = False
        self.save_signal = False
        self.create_widget()

    def create_widget(self):
        self.setWindowTitle("ACT GET DATA")
        screen = app.primaryScreen()  # 获取主屏幕
        size = screen.geometry()  # 获取屏幕几何信息
        screen_width = size.width()
        screen_height = size.height()
        window_width = 1920
        window_height = 1080

        # 创建定时器
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.updata_frame)
        self.task_timer = QTimer()
        self.task_timer.timeout.connect(self.updata_collect_task)

        # 主布局管理器
        layout = QVBoxLayout()

        # 设置窗口位置和大小
        self.setGeometry(0, 0, window_width, window_height)
        self.move((screen_width - window_width) // 2, (screen_height - window_height) // 2)

        # 创建视频显示区域
        self.label = QLabel(' ', self)
        self.label.setFixedSize(window_width, int(window_height*0.7))
        # self.label.setScaledContents(True)  # 让 QLabel 随窗口大小调整图像
        layout.addWidget(self.label)

        # 创建选择框与输入框布局
        form_layout = QFormLayout()
        # combo_box_layout = QHBoxLayout()

        # self.combo_box = QComboBox()
        # self.combo_box.addItems(["Point 1", "Point 2", "Point 3"])  # 添加选项
        # form_layout.addRow("Select:", self.combo_box)

        self.input_box = QLineEdit()
        self.input_box.setPlaceholderText("Input position")
        form_layout.addRow("Data:", self.input_box)
        layout.addLayout(form_layout)

        # 设置按钮
        button_layout = QHBoxLayout()
        # self.add_point_btn = QPushButton("Add Point", self)
        # self.add_point_btn.clicked.connect(self.on_add_point_btn_click)
        # button_layout.addWidget(self.add_point_btn)
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

        self.change_mode_btn = QPushButton("修改位置开关", self)
        self.change_mode_btn.clicked.connect(self.on_push_change_mode_btn_click)
        button_layout.addWidget(self.change_mode_btn)

        self.enable_btn = QPushButton("使能", self)
        self.enable_btn.clicked.connect(self.on_push_enable_btn_click)
        button_layout.addWidget(self.enable_btn)
        
        self.enable_power_btn = QPushButton("上电", self)
        self.enable_power_btn.clicked.connect(self.on_push_enable_power_btn_click)
        button_layout.addWidget(self.enable_power_btn)

        layout.addLayout(button_layout)
        # Task-related buttons
        self.start_task = QPushButton("Start Task", self)
        self.start_task.clicked.connect(self.on_start_task_btn_click)
        layout.addWidget(self.start_task)
        self.stop_task = QPushButton("Stop Task", self)
        self.stop_task.clicked.connect(self.on_stop_task_btn_click)
        layout.addWidget(self.stop_task)

        # 进度条部分
        self.episode_index_box = QLineEdit()
        self.episode_index_box.setPlaceholderText(f"Set start episode_idx.., default: {self.start_index },{self.end_index}")
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
        self.progress_bar.setRange(0, 500)
        layout.addWidget(self.progress_bar)


        # 设置窗口的布局
        self.setLayout(layout)

        print("控件初始化完成")

    # def on_add_point_btn_click(self):
    #     point_count = self.combo_box.count() + 1  # 计算新点的索引
    #     self.combo_box.addItem(f"Point {point_count}")  # 添加新点
    #     print(f"添加新的点位: Point {point_count}")
    def on_push_change_mode_btn_click(self):
        global action_play
        if action_play == False:
            action_play = True
            self.change_mode_btn.setText("action_play:True")  # 直接修改文本
        else:
            action_play = False
            self.change_mode_btn.setText("action_play:False")
        print(action_play)
    def on_push_enable_power_btn_click(self):
        self.robot.enable_power()
    def on_push_enable_btn_click(self):
        self.action_plan.start()
    def on_setting_btn_click(self):
        # global action_play
        # 读取 QLineEdit 输入的点位数据
        input_text = self.input_box.text().strip()
        try:
            # 将输入文本转换为列表，例如 "[1.0, 2.0, 3.0]"
            position = eval(input_text)  # 直接解析为 Python 列表
            if not isinstance(position, list) or len(position) < 3:
                raise ValueError  # 确保是合法的坐标格式
        except:
            QMessageBox.warning(self, "输入错误", "请输入正确的坐标格式，如：[1.0, 2.0, 3.0]")
            return
        
        self.action_plan.point = position  # 更新指定索引的点位
        print(position)
        # action_play = True
    def on_get_robot_state_btn_click(self):
        self.pose = self.robot.get_state('pose')
        self.input_box.setText(json.dumps(self.pose))
    def on_start_task_btn_click(self):
        global action_play
        if self.action_plan.points is None:
            self.result_label.setText("points is None")
        # elif self.local_desktop_pos is None:
        #     self.result_label.setText("local_desktop_pos is None")
        # elif self.goal_pos is None:
        #     self.result_label.setText("goal_pos is None")
        else:
            # self.action_plan.val(self.start_pos,self.goal_pos,self.local_desktop_pos)
            self.gpcontrol.start()
            if self.action_plan.isRunning():
                print("Task already running")
            else:
                self.action_plan.start()
            action_play = False
            self.progress_value = 0  # 复位进度
            self.task_timer.start(10)
            self.result_label.setText("task start")
            time.sleep(1)
    def on_stop_task_btn_click(self):
        self.robot.stop()
        self.task_timer.stop()
        # self.camera_timer.stop()
        self.gpcontrol.stop()
        self.action_plan.stop()
        self.result_label.setText("task stop")
    def on_stop_emergency_btn_click(self):
        self.task_timer.stop()
        self.camera_timer.stop()
        self.gpcontrol.stop()
        self.action_plan.stop()
    def updata_collect_task(self):
        # print(self.traja_reverse_signal)
        # if not self.traja_reverse_signal:
        start_time = time.time()
        if not False:
            self.progress_value += 1
            # print(f"当前进度: {self.progress_value}")
            self.progress_bar.setValue(self.progress_value)
            # start = time.time()

            angle_qpos=self.robot.get_state()
            # if self.robot.get_state('pose'):
            #     self.robot.set_state(self.robot.get_state('pose'))
            # print(time.time()-start)
            radius_qpos = [math.radians(j) for j in angle_qpos]
            # 处理 gpstate
            # print(self.gpstate)
            if self.gpstate:
                gpstate, gppos, gpforce = map(lambda x: str(x) if not isinstance(x, str) else x, self.gpstate)
                radius_qpos.extend([int(gpstate, 16), int(gppos, 16), int(gpforce, 16)])
            else:
                raise ValueError("error in gpstate")
            # 记录 qpos 数据
            self.qpos_list.append(radius_qpos)
            # 记录图像数据
            if self.image:
                for camera_name in camera_names:
                    self.images_dict[camera_name].append(self.image.get(camera_name))

            if self.close_signal:
                time.sleep(0.1)
            else:
                time.sleep(0.1)
            if self.traja_reverse_signal is True and self.task_complete_step == 0:
                self.task_complete_step = self.progress_value
        end_time = time.time()
        print(f"一帧时间：{end_time - start_time}")
            # 任务完成检查
        # if self.progress_value:
            # if self.progress_value >= 100:
            #     self.complete_sign = True
            #     self.task_complete_step = self.progress_value/2
            #     print(self.task_complete_step)
            #     self.result_label.setText("task complete")
        if self.complete_sign:
            print(self.index - self.start_index)
            if self.index >= self.end_index :
                print('task completed')
                self.save_data()
                self.action_plan.stop() 
                self.task_timer.stop()
                self.complete_sign=False
            else:
                self.save_data()
                self.complete_sign=False
                print("completed data collection")

    def handle_feedback(self,feed_back):
        self.gpstate=feed_back
        # print(self.gpstate)
    def handle_traja_reverse_signal(self,traja_feed_back):
        self.traja_reverse_signal = traja_feed_back
        print(f"traja_reverse_signal :{traja_feed_back}")
    def handle_action_signal(self,action_feed_back):
        self.gpcontrol.set_state_flag(action_feed_back)
    def handle_complete_signal(self,complete_feed_back):
        self.complete_sign =complete_feed_back
        print(f"complete_sign :{complete_feed_back}")
    def handle_error_signal(self,error_feed_back):
        print(error_feed_back)
        self.result_label.setText(error_feed_back)
    def handle_close_signal(self,close_signal):
        self.close_signal = close_signal
    # def save_data(self):

    #     try:
    #         global save_signal
    #         data_dict:dict[str,np.array]={}
    #         data_dict_add:dict[str,np.array]={}
    #         save_signal = True
    #         self.action_list = self.qpos_list
    #         self.max_episode_len=self.progress_value
    #         if self.qpos_list is not None:
    #             # self.qpos_list = np.vstack([self.qpos_list[0], self.qpos_list])
    #             self.qpos_array = np.vstack([self.qpos_list[0], self.qpos_list])  # 存入一个新变量
    #         else:
    #             raise "qpos is none"
    #         data_dict = {
    #             '/observations/qpos': self.qpos_array[:self.max_episode_len],
    #             '/action': self.action_list[:self.max_episode_len],
    #         }
            
    #         data_dict_add = {
    #             '/observations/qpos': self.qpos_array[:self.max_episode_len],
    #             '/action': self.action_list[:self.max_episode_len],
    #         }
    #         self.progress_value = 0  # 复位进度
    #         self.qpos_list.clear()
    #         self.action_list.clear()
    #         for cam_name in camera_names:
    #             data_dict[f'/observations/images/{cam_name}'] = self.images_dict[cam_name][:self.max_episode_len]
            
    #         self.generator_hdf5.save_hdf5(data_dict,"./hdf5_file",self.index)
    #         data_dict:dict[str,np.array]={}

    #         if self.task_complete_step != 0:
    #             for cam_name in camera_names:
    #                 images = self.images_dict[cam_name][:self.max_episode_len]
    #                 colored_images = []
    #                 square_size = 100

    #                 for i, img in enumerate(images):
    #                     img_copy = [row[:] for row in img]  # 深拷贝，防止改到原图

    #                     height = len(img_copy)
    #                     width = len(img_copy[0])
    #                     square_color = [0, 0, 255] if i < self.task_complete_step else [0, 255, 0]  # 蓝或绿（RGB）

    #                     # 左下角：行范围 [height - square_size, height)
    #                     for row in range(height - square_size, height):
    #                         for col in range(square_size):
    #                             if 0 <= row < height and 0 <= col < width:
    #                                 img_copy[row][col] = square_color

    #                     colored_images.append(img_copy)
    #                 self.images_dict[cam_name] = colored_images
    #                 data_dict_add[f'/observations/images/{cam_name}'] = self.images_dict[cam_name][:self.max_episode_len]
    #                 self.images_dict[cam_name].clear()
    #         self.generator_hdf5.save_hdf5(data_dict_add,"./hdf5_file_add",self.index)
    #         self.task_complete_step = 0
    #         data_dict_add:dict[str,np.array]={}

    #         self.index+=1
    #         if self.index>self.end_index:
    #             self.result_label.setText(f"task over please restart or close")
    #         save_signal = False
    #     except Exception as e:
    #         print(f"Save data error: {e}")
    #         self.result_label.setText(f"保存出错：{str(e)}")
    def save_data(self):
        try:
            global save_signal
            save_signal = True

            data_dict = {}
            data_dict_add = {}
            self.action_list = self.qpos_list
            self.max_episode_len = self.progress_value

            if self.qpos_list is not None:
                self.qpos_array = np.vstack([self.qpos_list[0], self.qpos_list])  # 一次性拼接
            else:
                raise ValueError("qpos_list is None")
            self.action_array = np.array(self.action_list)
            # 保存动作和关节状态
            data_dict = {
                '/observations/qpos': self.qpos_array[:self.max_episode_len],
                '/action': self.action_list[:self.max_episode_len],
            }

            # 保存图像
            for cam_name in camera_names:
                # 确保图像是np.array
                images_np = np.stack(self.images_dict[cam_name][:self.max_episode_len], axis=0)  # (N, H, W, C)
                data_dict[f'/observations/images/{cam_name}'] = images_np

            # 保存主文件
            self.generator_hdf5.save_hdf5(data_dict, "./hdf5_file", self.index)

            # 清理原始数据
            self.progress_value = 0
            self.qpos_list.clear()
            self.action_list.clear()

            # 如果需要带有色块的图像版本（标注 task_complete_step）
            if self.task_complete_step != 0:
                square_size = 100
                square_color_pre = np.array([0, 0, 255], dtype=np.uint8)   # 蓝
                square_color_post = np.array([0, 255, 0], dtype=np.uint8)  # 绿

                for cam_name in camera_names:
                    images = np.stack(self.images_dict[cam_name][:self.max_episode_len], axis=0)  # (N, H, W, C)
                    height, width = images.shape[1:3]

                    # 应用色块：直接修改图像像素
                    for i in range(len(images)):
                        color = square_color_pre if i < self.task_complete_step else square_color_post
                        images[i, height - square_size:, :square_size] = color

                    data_dict_add[f'/observations/images/{cam_name}'] = images

                    # 清理图像缓存
                    self.images_dict[cam_name].clear()

            # 保存带标注图像的版本
            if data_dict_add:
                data_dict_add['/observations/qpos'] = self.qpos_array[:self.max_episode_len]
                data_dict_add['/action'] = self.action_array[:self.max_episode_len]
                self.generator_hdf5.save_hdf5(data_dict_add, "./hdf5_file_add", self.index, compressed=False)

            # 更新索引和状态
            self.task_complete_step = 0
            self.index += 1

            if self.index > self.end_index:
                self.result_label.setText("task over please restart or close")

            save_signal = False

        except Exception as e:
            print(f"Save data error: {e}")
            self.result_label.setText(f"保存出错：{str(e)}")

    def on_set_index_btn_click(self):
        index=self.episode_index_box.text()
        # print(index)
        if not re.match(r"^\d+,\d+$", index):
            self.result_label.setText(f"index error")
        else:
            index_len = list(map(int, index.split(',')))
            self.end_index=index_len[1]
            self.index=index_len[0]
            self.start_index =index_len[0]

    def start_camera(self):
        """启动摄像头"""
        self.stop_render =False
        self.camera_timer.start(0)

    def updata_frame(self):
        """更新摄像头图像"""
        global multi_device_sync_config
        if self.camera.change_signal:
            self.camera = CAMERA_HOT_PLUG()
            frame_data, color_width, color_height = self.camera.rendering_frame()

        else:
            frame_data, color_width, color_height = self.camera.rendering_frame()
        # print(color_height,color_width)
        serial_number_list = self.camera.serial_number_list
        camera_index_map = {device['config']['camera_name']: serial_number_list.index(device["serial_number"]) for device in multi_device_sync_config.values() if device["serial_number"] in serial_number_list}
        # print(camera_index_map)
        if isinstance(frame_data, dict):  # 多台摄像头返回字典 {str: np.ndarray}
            if not frame_data:  # 字典为空
                print("⚠️ WARN: 没有接收到任何摄像头图像")
                return
            if all(img.size == 0 for img in frame_data.values()):  # 所有相机的图像都是空的
                print("⚠️ WARN: 所有摄像头的图像数据为空")
                return
            # print(f"⚠️ WARN: 多台摄像头，序列号列表: {serial_number_list}")
        elif isinstance(frame_data, np.ndarray):  # 只有一台相机
            if frame_data.size == 0:
                print("⚠️ WARN: 没有接收到任何摄像头图像")
                return
            # 只有一个摄像头时，将其存入字典，模拟多摄像头格式
            frame_data = {"0": frame_data}  
            serial_number_list = ["0"]
            print(f"⚠️ WARN: 只有一台摄像头，序列号为 {serial_number_list[0]}")
        else:
            print(f"⚠️ ERROR: 无效的 frame_data 类型: {type(frame_data)}")
            return
        # 初始化结果图像
        result_image = None
        for device in multi_device_sync_config.values():
            cam_name, sn = device['config']['camera_name'], device["serial_number"]
            # print(cam_name,sn)
            if sn in frame_data:
                # print(sn)
                img = frame_data[sn]
                if result_image is None:
                    result_image = img  # 第一个摄像头的图像
                else:
                    result_image = np.hstack((result_image, img))  # 按水平方向拼接图像

        if result_image is not None:
            self.display_image(result_image)
            # print(type(np.array(frame_data.get(str(serial_number_list[camera_index_map['top']]), None))))
            if self.task_timer.isActive():
                # print(f"camera_index_map: {camera_index_map}")
                for cam_name in camera_names:
                    # if cam_name == 'top':.
                    self.image[cam_name]=np.array(frame_data.get(str(serial_number_list[camera_index_map[cam_name]]), None))
                    # elif cam_name == 'right_wrist':
                    #     self.image[cam_name]=frame_data.get(str(serial_number_list[camera_index_map[cam_name]]), None)
                    # elif cam_name == 'left_wrist':
                    #     self.image[cam_name]=frame_data.get(str(serial_number_list[camera_index_map[cam_name]]), None)

    def display_image(self,result_image):
        # **显示图像**
        # print(f"display_result_image")
        if self.stop_render is False:
            color_height, color_width, ch = result_image.shape
            # print(result_image.shape)
            qt_img = QImage(result_image, color_width, color_height, ch * color_width, QImage.Format_RGB888)
            qimage = qt_img.rgbSwapped()
            self.label.setPixmap(QPixmap.fromImage(qimage))
    # def display_image(self, result_image):
    #     """显示图像，并适应 QLabel 大小"""
    #     if self.stop_render is False:
    #         color_height, color_width, ch = result_image.shape
    #         qt_img = QImage(result_image, color_width, color_height, ch * color_width, QImage.Format_RGB888)
    #         qt_img = qt_img.scaled(self.label.width(), self.label.height())  # 适应 QLabel 大小
    #         self.label.setPixmap(QPixmap.fromImage(qt_img))
    def stop_camera(self):
        """关闭摄像头"""
        self.stop_render=True
        # self.timer.stop()
        self.label.clear()
        # self.camera.stop_straems()
    def start_collect(self):
        """开始采集数据"""
        self.gpcontrol.start(10)
        self.task_timer.start(10)
        self.progress_value = 0  # 复位进度
    def stop_collect(self):
        """停止采集数据"""
 
        print(self.progress_value)
        self.save_data()
        self.task_timer.stop()
        self.result_label.setText("采集已停止")
       
    def closeEvent(self, event):
        """关闭窗口时释放摄像头"""
        print("关闭窗口")
        # self.robot.Client.close()
        self.action_plan.stop()
        self.camera.stop()
        # print("camera-stop")
        self.camera_timer.stop()
        self.stop_camera()
        # self.camera.stop_streams()
        # print("camera-timer-stop")
        self.task_timer.stop()
        self.gpcontrol.close()
        self.gpcontrol.stop()
        event.accept()

# if __name__ == '__main__':
app = QApplication([])
# 创建并显示窗口
window = run_main_windows()
window.show()
# 启动事件循环
app.exec_()