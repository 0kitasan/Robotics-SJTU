import time
import numpy as np
# 创建机械臂对象
import rospy
from dofbot_real import RealEnv


def linear_interpolation(src, tat, n=10):
    """简单的线性插值实现"""
    path = np.linspace(src, tat, num=n)
    return path

# 2 号机器 关闭 gripper 150  开启 gripper 90

if __name__ == '__main__':
    # 调用realenv
    env = RealEnv()
    env.reset()
    print("等待初始化完成")
    time.sleep(1)

    # env.step(joint=np.asarray([90., 90., 90., 90., 90.]), gripper=90)

    # env.step(joint=np.asarray([90., 90., 90., 90., 90.]))
    # # env.step(joint=np.asarray([137., 51., 52., 2., 90.]))

    # 可以实现简单状态机来实现分段控制
    # 状态机的中间路点
    points = [
        np.asarray([90., 90., 90., 90., 90.]), # 初始位置
        np.asarray([137., 51., 52., 2., 90.]), # 第一个目标位置 修改
        np.asarray([40., 61., 42., 7., 90.]), # 第二个目标位置 修改
    ] 

    # 2 号机器 关闭 gripper 150  开启 gripper 90

    for i in range(len(points) - 1):
        # 取出路点并做路径规划得到路径
        path = linear_interpolation(points[i], points[i + 1], n=10)
        
        print(f"current i: {i}")

        for j,p in enumerate(path):
            # 建议分开控制
            print(f"{path}")

            length = len(path)

            print(f"length: {length}, current j: {j}")

            if i == 0:
                if j == length - 1: #  假设在第一个路径段打开夹爪，再关闭
                    env.step(joint=p, gripper=90) 
                    time.sleep(0.1)
                    env.step(joint=p, gripper=150)
                    print("进入抓取最后一段")

                else:
                    env.step(joint=p, gripper=90) 

                


            if i == 1:
                if j == length - 1:
                    env.step(joint=p, gripper=90) 
                    time.sleep(0.1)
                    # env.step(joint=p, gripper=150)
                    print("进入放置最后一段")
                else:
                    env.step(joint=p, gripper=150) 


