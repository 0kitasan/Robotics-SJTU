import time
import numpy as np
# 创建机械臂对象
import rospy
from dofbot_real import RealEnv


def linear_interpolation(src, tat, n=20):
    """
    简单的线性插值实现。
    :param src: 起始点 (numpy array)
    :param tat: 目标点 (numpy array)
    :param n: 插值点的数量
    :return: 包含从 src 到 tat 的路径点 (numpy array)
    """
    # np.linspace 会在起始点和目标点之间生成 n 个均匀间隔的点
    path = np.linspace(src, tat, num=n)
    return path

if __name__ == '__main__':
    # 调用realenv，初始化ROS节点并连接到机器人
    env = RealEnv()
    
    print("正在将机械臂重置到初始位置...")
    env.reset()
    print(f"重置完成。当前关节角度: {env.get_state()}")
    
    # --- 定义夹爪状态常量，使代码更易读 ---
    GRIPPER_OPEN = 120  # 夹爪张开的角度
    GRIPPER_CLOSED = 30 # 夹爪闭合的角度

    # --- 定义一个简单的抓取任务的状态机路点 ---
    # 状态机的中间路点，每个路点都是一个包含5个关节角度的numpy数组
    points = [
        np.asarray([90., 90., 90., 90., 90.]),      # 0. 初始位置 (Reset)
        np.asarray([90., 45., 90., 45., 90.]),      # 1. 抬起并前倾的“准备”位置
        np.asarray([137., 51., 52., 2., 90.])       # 2. 最终的抓取目标位置
    ]
    
    # 任务开始前，先将夹爪张开，准备抓取
    print(f"正在张开夹爪至角度 {GRIPPER_OPEN}...")
    env.step(gripper=GRIPPER_OPEN)
    time.sleep(1) # 等待夹爪动作完成

    # 遍历路点，从一个点移动到下一个点
    for i in range(len(points) - 1):
        # 取出当前路段的起始点和目标点
        start_point = points[i]
        target_point = points[i+1]
        
        print(f"\n--- 正在从路点 {i} 移动到路点 {i+1} ---")
        print(f"起始角度: {start_point}")
        print(f"目标角度: {target_point}")

        # 使用线性插值函数进行简单的路径规划，得到平滑的路径
        path = linear_interpolation(start_point, target_point)

        # 遍历路径上的每一个小步骤点
        for p in path:
            # 打印当前执行的步骤点（保留两位小数）
            print(f"  执行步骤: {np.round(p, 2)}")
            
            # 使用 env.step(joint=...) 控制关节移动到该步骤点
            # 这个函数是阻塞的，会等到动作完成后才返回
            env.step(joint=p)

    print("\n--- 已到达最终抓取位置 ---")

    # 到达目标点后，闭合夹爪，模拟抓取物体
    print(f"正在闭合夹爪至角度 {GRIPPER_CLOSED}...")
    env.step(gripper=GRIPPER_CLOSED)
    time.sleep(1) # 等待夹爪动作完成
    print("抓取完成！")

    # 任务完成，让机械臂按原路返回初始位置
    print("\n--- 正在返回初始位置 ---")
    # 创建一个从最后一个点返回到第一个点的路径
    return_path = linear_interpolation(points[-1], points[0])
    for p in return_path:
        print(f"  返回步骤: {np.round(p, 2)}")
        env.step(joint=p)
        
    print("\n任务全部完成！")
