import time
import numpy as np
# 创建机械臂对象
import rospy
from dofbot_real import RealEnv


def linear_interpolation(src, tat, n=10):
    """简单的线性插值实现"""
    path = np.linspace(src, tat, num=n)
    return path

if __name__ == '__main__':
    # 调用realenv
    env = RealEnv()
    env.reset()


    # 可以实现简单状态机来实现分段控制
    # 状态机的中间路点
    points = [
        np.asarray([90., 90., 90., 90., 90.]), # 初始位置
        ...
    ]
    
    # target:(137,51,52,2,90,120)

    for i in range(len(points) - 1):
        # 取出路点并做路径规划得到路径
        path = ...
        for p in path:
            # 执行路径上各点
            # env.step(joint=...)可以控制关节
            # env.step(gripper=...)可以控制夹爪
            # 建议分开控制
            ...

