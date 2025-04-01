from tcp_tx import PersistentClient
import time

Client = PersistentClient('192.168.2.14', 8001)
number = 2
Client.set_close(number)
Client.set_clear(number)
Client.set_open(number)
# Client.set_close(number)
# Client.set_clear(number)
# Client.set_open(number)
def is_close(actual, target, tolerance=0.1):
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
        if abs(a - t) > tolerance:
            return False
    return True

right_camera_pose = [-39.273, 737.31, -82.4644, -2.49483, 0.484904, -1.61246]
point_1_1  = [-59.2402, -416.372, 440.325, 2.15084, -0.0677886, 2.93576]
point_1_2  = [-40.2402, -416.372, 440.325, 2.15084, -0.0677886, 2.93576]
point_2_1 = [58.9871, 570.444, 591.066, -1.68646, 0.285616, -1.24174]
point_2_2 = [19.9871, 570.444, 591.066, -1.68646, 0.285616, -1.24174]
point_2_3 = [-30.0129, 570.444, 591.066, -1.68646, 0.285616, -1.24174]
i = 0
while True:
    try:
        # if i == 0:
        Client.set_arm_position(model='pose',robotnum= number,value=point_2_1)
        Client.set_arm_position(model='pose',robotnum= number,value=point_2_2)
        Client.set_arm_position(model='pose',robotnum= number,value=point_2_3)
            # i+= 1
        
        # if is_close(Client.get_arm_position_pose(number), point_2_2, tolerance=0.1):
        #     print(Client.get_arm_position_pose(number))
        #     print("机械臂到达目标位置")

        time.sleep(1)
        # print(Client.get_arm_position_pose(number))
    except Exception as e:
        print(e)
        print("连接失败")