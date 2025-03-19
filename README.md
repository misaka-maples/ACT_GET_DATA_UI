# 机械臂多相机数据采集系统

## 简介
本项目实现了一个基于PyQt5的多线程控制系统，集成了机械臂控制、夹爪控制、多相机同步采集和HDF5数据存储功能。系统支持：
- 实时显示多个奥比中光深度相机的同步画面
- 机械臂轨迹规划与运动控制
- 电动夹爪状态监控与精准控制
- 多模态数据采集（图像/关节位置/夹爪状态）
- 自动化数据采集流程管理

## 安装依赖
```bash
pip install pyorbbecsdk pynput pyqt5 h5py opencv-python numpy pyserial
```

## 硬件配置要求
1. 机械臂: 支持RM系列机械臂（需提供IP地址）
2. 相机: 奥比中光Astra系列深度相机（至少2台，需配置同步线）
3. 夹爪: 支持CAN总线通信的电动夹爪
4. 同步控制器: 多设备同步触发装置

## 配置文件
在`config/multi_device_sync_config.json`中配置相机同步参数：
```json
{
  "devices": [
    {
      "serial_number": "相机1序列号",
      "config": {
        "mode": "PRIMARY",
        "color_delay_us": 0,
        "depth_delay_us": 0
      }
    },
    {
      "serial_number": "相机2序列号",
      "config": {
        "mode": "SECONDARY_SYNCED",
        "color_delay_us": 0,
        "depth_delay_us": 0
      }
    }
  ]
}
```

## 使用说明
1. 启动GUI:
```python
python main.py
```

2. 界面操作流程:
   - 连接设备: 依次点击"start camera"初始化相机，"get robot state"获取机械臂状态
   - 设置路径点:
     - 在下拉框选择点类型（start/local_desktop/goal）
     - 在输入框输入JSON格式的坐标（示例：[-0.2, 0.05, 0.69, -1.56, -0.59, 1.47]）
     - 点击"set_point"保存当前点
   - 数据采集:
     - 在index输入框设置采集次数（示例："0,10"表示从0开始采集10次）
     - 点击"start task"开始自动采集流程

3. 控制说明:
   - 机械臂运动速度: 默认15%（可在`ACTION_PLAN`类调整）
   - 夹爪控制逻辑:
     - 0: 闭合
     - 1: 半开
     - 2: 全开
   - 紧急停止: 关闭窗口或使用键盘ESC键

## 数据存储格式
采集数据以HDF5格式存储，结构如下：
```yaml
episode_{index}.hdf5
├── observations
│   ├── images
│   │   ├── top: uint8 array [NxHxWx3]  # 顶部相机RGB图像
│   │   └── right_wrist: uint8 array [NxHxWx3]  # 腕部相机RGB图像
│   └── qpos: float32 array [Nx7]  # 关节位置（6轴+工具电压）
├── action: float32 array [Nx7]  # 执行动作
└── gp
    ├── gpstate: int32 array [N]  # 夹爪状态
    ├── gppos: int32 array [N]    # 夹爪位置
    └── gpforce: int32 array [N]   # 夹爪力度
```

## 注意事项
1. 硬件连接:
   - 确保所有相机通过同步线正确连接
   - 机械臂控制需在同一个局域网段
   - 夹爪CAN总线需正确配置终端电阻

2. 同步问题处理:
   - 如果出现画面不同步，检查相机的供电稳定性
   - 确认同步线连接顺序（主相机->从相机）
   - 使用`get_device_info.py`工具验证相机识别

3. 安全规范:
   - 机械臂工作区域需保持清空
   - 急停按钮应设置在便于操作的位置
   - 首次运行建议在示教模式下进行

## 已知限制
1. 相机分辨率固定为640x480@30fps
2. 最大采集时长限制为500帧/次
3. 暂不支持动态调整运动轨迹
4. 需要手动设置机械臂初始位置

## 故障排除
Q: 相机无法初始化  
A: 检查USB3.0连接，确认SDK权限设置，尝试重新插拔相机

Q: 机械臂连接超时  
A: 确认IP地址正确，检查防火墙设置，验证网络ping测试

Q: 数据文件损坏  
A: 确保存储路径有足够空间，采集过程中不要强制终止程序

Q: 夹爪状态不更新  
A: 检查CAN总线终端电阻（需120Ω），验证电源供电稳定性
