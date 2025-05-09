class ROBOT:
    def __init__(self):
        self.joint_state_right=None
        self.Client = PersistentClient('192.168.3.15', 8001)
        # self.Client = PersistentClient('192.168.2.14', 8001)
        # self.Client.set_close(robot_num)
        # self.Client.set_clear(robot_num)
        # self.Client.set_open(robot_num)
        # self.rm_65_b_right_arm = (RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E))
        # self.arm_ini = self.rm_65_b_right_arm.rm_create_robot_arm("192.168.1.18",8080, level=3)

    def get_state(self, model='joint',robot_num=1):#pose
        # self.joint_state_right = self.rm_65_b_right_arm.rm_get_current_arm_state()
        # return_action = self.joint_state_right[1][model]
        if model=='joint':
            action=self.Client.get_arm_position_joint(robotnum=robot_num)
        elif model=='pose':
            action = self.Client.get_arm_position_pose(robotnum=robot_num)
        
        return action
    def set_state(self, action, model='joint',robot_num=1):

        if model=='joint':
            self.Client.set_arm_position(action,'joint',robot_num)
        elif model=='pose':
            self.Client.set_arm_position(action,'pose',robot_num)
        else:
            raise ValueError(f"model {model} is not support")
    def enable_power(self):
        self.Client.set_close(1)
        self.Client.set_clear(1) 
        self.Client.set_close(2)
        self.Client.set_clear(2)
        time.sleep(1)
        self.Client.set_open(2)
        self.Client.set_open(1)
    def stop(self,robot_num):
        self.Client.set_stop(robot_num)
        self.Client.set_reset(robot_num)
    
        # self.rm_65_b_right_arm.rm_set_stop()
