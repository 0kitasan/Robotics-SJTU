from dofbot import DofbotEnv
import numpy as np
import copy
import time, os, datetime
import pybullet as p

# ---------- 1. 准备保存目录 ----------
save_dir = "results/record"
os.makedirs(save_dir, exist_ok=True)
mp4_path = os.path.join(
    save_dir, datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".mp4"
)

if __name__ == "__main__":
    env = DofbotEnv()
    env.reset()
    Reward = False

    # 2. 开始录制
    log_id = p.startStateLogging(
        p.STATE_LOGGING_VIDEO_MP4, mp4_path, physicsClientId=env.physicsClient
    )

    """
    constants here
    """
    GRIPPER_DEFAULT_ANGLE = 20.0 / 180.0 * 3.1415
    GRIPPER_CLOSE_ANGLE = -20.0 / 180.0 * 3.1415

    # define state machine
    PRE_GRASP_STATE = 0
    GRASP_STATE = 1
    MOVE_STATE = 2
    SET_STATE = 3
    current_state = PRE_GRASP_STATE

    # print("object1.size: ", env._object1.size)  # → [0.03, 0.03, 0.03]  （半尺寸）
    obj_offset_grasp = [-0.015, -0.015, 0.045]
    obj_offset_move = [0, 0, 0.145]
    obj_offset_set = [-0.015, 0.015, 0.045]

    block_pos, block_orn, block_euler = env.get_block_pose()
    target_pos = env.get_target_pose()
    start_time = None

    time.sleep(1.0)
    num = 0
    PRE_GRASP_NUM = 1800
    GRASP_NUM = 1200
    MOVE_NUM = 2000
    SET_NUM = 1000

    while not Reward:
        """
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        """

        # # 获取当前物块和目标位置 - 需要在每个循环开始时获取
        # block_pos, block_orn, block_euler = env.get_block_pose()

        # 状态机控制
        if current_state == PRE_GRASP_STATE:  # 0: 预抓取位置
            print("状态: 移动到预抓取位置")

            # 计算预抓取位置（物块上方）
            pre_grasp_pos = [
                block_pos[0] + obj_offset_grasp[0],
                block_pos[1] + obj_offset_grasp[1],
                block_pos[2] + obj_offset_grasp[2],
            ]

            # 使用逆运动学计算关节角度
            joint_angles, gripper_angle = env.dofbot_setInverseKine(pre_grasp_pos)

            # 控制机械臂和夹爪
            if joint_angles is not None:
                env.dofbot_control(joint_angles, GRIPPER_DEFAULT_ANGLE)

            # 短暂停留确保到位
            if num >= PRE_GRASP_NUM:
                current_state = GRASP_STATE
                num = 0

        elif current_state == GRASP_STATE:  # 1: 下降到抓取位置
            print("状态: 下降到抓取位置")

            # 计算精确抓取位置（稍微降低高度）
            grasp_pos = [
                block_pos[0],
                block_pos[1],
                block_pos[2] + 0.025,  # 稍微接触物块
            ]

            joint_angles, gripper_angle = env.dofbot_setInverseKine(grasp_pos)
            if joint_angles is not None:
                env.dofbot_control(joint_angles, GRIPPER_CLOSE_ANGLE)  # 夹紧

            if num >= GRASP_NUM:
                current_state = MOVE_STATE
                num = 0

        elif current_state == MOVE_STATE:  # 2: 提起并移动到目标上方
            if num < MOVE_NUM // 2:
                print("状态: 提起并移动到目标上方")
                # 先提起物块
                lift_pos = [
                    block_pos[0],
                    block_pos[1],
                    block_pos[2] + obj_offset_move[2],
                ]

                joint_angles, gripper_angle = env.dofbot_setInverseKine(lift_pos)
                if joint_angles is not None:
                    env.dofbot_control(joint_angles, GRIPPER_CLOSE_ANGLE)

            # 然后移动到目标上方
            if num >= MOVE_NUM // 2:
                move_pos = [
                    target_pos[0],
                    target_pos[1],
                    target_pos[2] + obj_offset_move[2],
                ]
                print("状态: 移动到目标上方")
                joint_angles, gripper_angle = env.dofbot_setInverseKine(move_pos)
                if joint_angles is not None:
                    env.dofbot_control(joint_angles, GRIPPER_CLOSE_ANGLE)

            if num >= MOVE_NUM:
                current_state = SET_STATE
                num = 0

        elif current_state == SET_STATE:  # 3: 下降到放置位置
            print("状态: 下降到放置位置并释放")

            # 计算预放置位置
            pre_set_pos = [
                target_pos[0],
                target_pos[1],
                target_pos[2] + obj_offset_set[2] * 0.8,
            ]

            joint_angles, gripper_angle = env.dofbot_setInverseKine(pre_set_pos)
            if joint_angles is not None:
                env.dofbot_control(joint_angles, GRIPPER_CLOSE_ANGLE)

                # 最终放置位置
            if num >= SET_NUM // 2:
                set_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.025]

                joint_angles, gripper_angle = env.dofbot_setInverseKine(set_pos)
                if joint_angles is not None:
                    env.dofbot_control(joint_angles, GRIPPER_DEFAULT_ANGLE)  # 释放

            if num >= SET_NUM:
                print("抓取放置任务完成!")
                Reward = True

        # 计数器递增 - 这行应该与上面的if语句平级
        num += 1
        Reward = env.reward()

    # env.step_with_sliders()
    # ---------- 3. 结束录制 ----------
    p.stopStateLogging(log_id)
