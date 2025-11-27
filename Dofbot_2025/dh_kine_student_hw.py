"""
机器人学课程 Dofbot 机械臂基于改进DH参数法的正 / 逆运动学建模
"""

# --------------------- 1. 导入常用库 ---------------------
import roboticstoolbox as rtb  # 机器人专用工具箱
import numpy as np  # 矩阵运算

import matplotlib.pyplot as plt
import time

# --------------------- 2. 常量定义 ---------------------
pi = 3.1415926  # 自己指定 π，方便后续打印保留 7 位小数
# 连杆长度（单位：m，与实物一致）
l1 = 0.1045  # 连杆1长度（基座→关节2）
l2 = 0.08285  # 连杆2长度（关节2→关节3）
l3 = 0.08285  # 连杆3长度（关节3→关节4）
l4 = 0.12842  # 连杆4长度（关节4→末端）

# ==============================================
# 用改进 DH 法建立机器人模型Demo
# ==============================================
# RevoluteMDH(a, alpha, d, offset)
# 默认 theta 为关节变量，因此只写常数项即可
DH_demo = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=l1),  # 关节1：绕 z 旋转，d 向上偏移 l1
        rtb.RevoluteMDH(
            alpha=-pi / 2, offset=-pi / 3
        ),  # 关节2：x 向下扭转 90°，初始偏置 -90°
        rtb.RevoluteMDH(a=l2, offset=pi / 6),  # 关节3：平移 l2
        rtb.RevoluteMDH(a=l3, offset=pi * 2 / 3),  # 关节4：平移 l3，初始偏置 +90°
        rtb.RevoluteMDH(alpha=pi / 2, d=l4),  # 关节5：x 向上扭转 90°，末端延伸 l4
    ],
    name="DH_demo",  # 给机器人起个名字，打印时更直观
)

# 打印标准 DH 参数表（alpha、a、d、theta、offset）
print("========== DH_demo机器人 DH 参数 ==========")
print(DH_demo)

# --------------------- 零位验证 ---------------------
fkine_input0 = [0, 0, 0, 0, 0]  # 全部关节置 0
fkine_result0 = DH_demo.fkine(fkine_input0)
print("\n零位正解齐次变换矩阵:")
print(fkine_result0)
DH_demo.plot(q=fkine_input0, block=True)  # 3D 可视化（阻塞模式）

# ==============================================
# 仿真任务0、 用改进 DH 法建立Dofbot机器人模型
# ==============================================
# RevoluteMDH(a, alpha, d, offset)
# 默认 theta 为关节变量，因此只写常数项即可
dofbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=l1),
        rtb.RevoluteMDH(alpha=-pi / 2),
        rtb.RevoluteMDH(a=l2),
        rtb.RevoluteMDH(a=l3),
        rtb.RevoluteMDH(alpha=pi / 2, d=l4),
    ],
    name="Dofbot",
)
# todo
dofbot = rtb.DHRobot(
    [
        # offset这里是偏移量 传入参数+偏移量
        rtb.RevoluteMDH(a=0, alpha=0, d=0.1045, offset=0),
        rtb.RevoluteMDH(a=0, alpha=-pi / 2, d=0, offset=-pi / 2),
        rtb.RevoluteMDH(a=0.08285, alpha=0, d=0, offset=0),
        rtb.RevoluteMDH(a=0.08285, alpha=0, d=0, offset=pi / 2),
        rtb.RevoluteMDH(a=0, alpha=pi / 2, d=0.12842, offset=0),
    ],
    name="Dofbot",
)
# 打印标准 DH 参数表（alpha、a、d、theta、offset）
print("========== Dofbot机器人 DH 参数 ==========")
print(dofbot)

# --------------------- 4. Part0 零位验证 ---------------------
fkine_input0 = [0, 0, 0, 0, 0]  # 全部关节置 0
fkine_result0 = dofbot.fkine(fkine_input0)
print("\n零位正解齐次变换矩阵:")
print(fkine_result0)
dofbot.plot(q=fkine_input0, block=True)  # 3D 可视化（阻塞模式）

# ==============================================
# 仿真任务1、 正运动学 —— 给出DH模型在以下 4 组关节角下的正运动学解
# ==============================================
poses = [
    [0.0, pi / 3, pi / 4, pi / 5, 0.0],  # demo
    [pi / 2, pi / 5, pi / 5, pi / 5, pi],  # 1
    [pi / 3, pi / 4, -pi / 3, -pi / 4, pi / 2],  # 2
    [-pi / 2, pi / 3, -2 * pi / 3, pi / 3, pi / 3],  # 3
]

# -------- 1.1 demo  pose ----------
q_demo = [0.0, pi / 3, pi / 4, pi / 5, 0.0]
T_demo = dofbot.fkine(q_demo)
print("\n========== Part1-0 (demo) 正解 ==========")
print(T_demo)
dofbot.plot(q=q_demo, block=True)

# -------- 1.2 pose 1 ----------

q_task1 = poses[1]
T_task1 = dofbot.fkine(q_task1)
print("\n========== Part1-1 正解 ==========")
print(T_task1)
dofbot.plot(q=q_task1, block=True)

# -------- 1.3 pose 2 ----------

q_task1 = poses[2]
T_task1 = dofbot.fkine(q_task1)
print("\n========== Part1-2 正解 ==========")
print(T_task1)
dofbot.plot(q=q_task1, block=True)

# -------- 1.4 pose 3 ----------

q_task1 = poses[3]
T_task1 = dofbot.fkine(q_task1)
print("\n========== Part1-3 正解 ==========")
print(T_task1)
dofbot.plot(q=q_task1, block=True)

# ==============================================
# 仿真任务2、 逆运动学 —— 给出DH模型在以下 4 组笛卡尔空间姿态下的逆运动学解
# ==============================================
targets = [
    # demo
    np.array(
        [
            [-1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, -0.1],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
    # 1
    np.array(
        [
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.1],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
    # 2
    np.array(
        [
            [np.cos(pi / 3), 0.0, -np.sin(pi / 3), 0.2],
            [0.0, 1.0, 0.0, 0.0],
            [np.sin(pi / 3), 0.0, np.cos(pi / 3), 0.2],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
    # 3
    np.array(
        [
            [-0.866, -0.25, -0.433, -0.03704],
            [0.5, -0.433, -0.75, -0.06415],
            [0.0, -0.866, 0.5, 0.3073],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ),
]

# -------- 2.1 demo 目标 ----------
T_des_demo = np.array(
    [
        [-1.0, 0.0, 0.0, 0.1],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, -0.1],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
q_ik_demo = dofbot.ik_LM(T_des_demo)[0]  # 取返回元组第 0 个元素
print("\n========== Part2-0 (demo) 逆解 ==========")
print("关节角（rad）：", np.array(q_ik_demo))
dofbot.plot(q=q_ik_demo, block=True)

# -------- 2.2 目标 1 ----------

q_ik_task2 = dofbot.ik_LM(targets[1])[0]
print("\n========== Part2-1 逆解 ==========")
print("关节角（rad）：", np.array(q_ik_task2))
dofbot.plot(q=q_ik_task2, block=True)

# -------- 2.3 目标 2 ----------

q_ik_task2 = dofbot.ik_LM(targets[2])[0]
print("\n========== Part2-2 逆解 ==========")
print("关节角（rad）：", np.array(q_ik_task2))
dofbot.plot(q=q_ik_task2, block=True)

# -------- 2.4 目标 3 ----------

q_ik_task2 = dofbot.ik_LM(targets[3])[0]
print("\n========== Part2-3 逆解 ==========")
print("关节角（rad）：", np.array(q_ik_task2))
dofbot.plot(q=q_ik_task2, block=True)

# ==============================================
# 仿真任务3、 工作空间可视化（≥500 点）
#     关节限位（°）→ 弧度
#     J1: [-180, 180]  J2~J5: [0, 180]
# ==============================================

# 定义关节角度范围和采样点数量

joint_limits = [
    [-pi, pi],  # J1
    [0, pi],  # J2
    [0, pi],  # J3
    [0, pi],  # J4
    [0, pi],  # J5
]

num_samples = 10000  # 总采样点数量

workspace_points = []  # 用于存储末端位置点的列表

print("\n========== 工作空间绘制 ==========")

# 进行随机采样
for _ in range(num_samples):
    # 在每个关节的限制范围内生成一个随机角度
    q_rand = [
        np.random.uniform(joint_limits[0][0], joint_limits[0][1]),
        np.random.uniform(joint_limits[1][0], joint_limits[1][1]),
        np.random.uniform(joint_limits[2][0], joint_limits[2][1]),
        np.random.uniform(joint_limits[3][0], joint_limits[3][1]),
        np.random.uniform(joint_limits[4][0], joint_limits[4][1]),
    ]

    # 计算正运动学
    T = dofbot.fkine(q_rand)
    # 提取末端位置坐标并添加到列表中
    workspace_points.append(T.t)

# 将列表转换为numpy数组，方便后续处理
points_array = np.array(workspace_points)

# 开始三维绘图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# 绘制散点图
ax.scatter(
    points_array[:, 0], points_array[:, 1], points_array[:, 2], c="b", s=1
)  # s是点的大小

# 设置坐标轴标签和标题
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title(f"Dofbot Workspace Visualization ({num_samples} points)")

# 设置坐标轴范围和比例
ax.set_box_aspect(
    [np.ptp(points_array[:, 0]), np.ptp(points_array[:, 1]), np.ptp(points_array[:, 2])]
)
ax.axis("equal")  # 设置此项可以使各轴比例相同，更好地反映真实空间形状

print("工作空间计算完成，正在显示图像...")
# 显示图像
plt.show()


# print("\n========== 正在使用遍历法计算工作空间，请耐心等待... ==========")
# start_time = time.time()  # 记录开始时间

# # 定义关节角度范围和步长（全部使用弧度）
# step = np.deg2rad(10)  # 步长设置为10度

# # 创建每个关节的角度遍历列表
# q1_range = np.arange(-pi, pi, step)
# q2_range = np.arange(0, pi, step)
# q3_range = np.arange(0, pi, step)
# q4_range = np.arange(0, pi, step)
# q5_range = np.arange(0, pi, step)


# # 计算总的迭代次数
# total_iterations = (
#     len(q1_range) * len(q2_range) * len(q3_range) * len(q4_range) * len(q5_range)
# )
# print(f"步长: 10 degrees")
# print(f"总计算点数: {total_iterations:.0f}")


# workspace_points = []
# count = 0

# # 五层嵌套循环进行遍历
# for q1 in q1_range:
#     for q2 in q2_range:
#         for q3 in q3_range:
#             for q4 in q4_range:
#                 for q5 in q5_range:
#                     q = [q1, q2, q3, q4, q5]
#                     # 计算正运动学
#                     T = dofbot.fkine(q)
#                     # 提取末端位置坐标并添加到列表中
#                     workspace_points.append(T.t)
#                     count += 1
#                     # 打印进度，避免长时间无响应
#                     if count % 100000 == 0:
#                         print(
#                             f"  已计算 {count} / {total_iterations} ({count/total_iterations*100:.2f}%)"
#                         )


# end_time = time.time()  # 记录结束时间
# print(f"\n工作空间计算完成! 总耗时: {end_time - start_time:.2f} 秒")


# # 将列表转换为numpy数组，方便后续处理
# points_array = np.array(workspace_points)

# # 开始三维绘图
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection="3d")

# # 绘制散点图
# ax.scatter(
#     points_array[:, 0], points_array[:, 1], points_array[:, 2], c="r", s=1
# )  # 用红色以示区别

# # 设置坐标轴标签和标题
# ax.set_xlabel("X (m)")
# ax.set_ylabel("Y (m)")
# ax.set_zlabel("Z (m)")
# ax.set_title(
#     f"Dofbot Workspace (Traversal Method, 10° Step, {total_iterations} points)"
# )

# # 设置坐标轴范围和比例
# ax.set_box_aspect(
#     [np.ptp(points_array[:, 0]), np.ptp(points_array[:, 1]), np.ptp(points_array[:, 2])]
# )
# ax.axis("equal")

# print("正在显示图像...")
# # 显示图像
# plt.show()
