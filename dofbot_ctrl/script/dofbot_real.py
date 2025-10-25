"""
Real Env
"""
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState

class RealEnv:
    def __init__(self,init_node=True):
        if init_node:
            rospy.init_node("Arm2")
        self.state = None
        self.pub = rospy.Publisher("/dofbot/cmd", JointState, queue_size=10)
        rospy.Subscriber("/dofbot/joint_state", JointState, self.callback)

    def callback(self, data: JointState):
        self.state = np.asarray(data.position)

    def reset(self):
        while self.state is None:
            time.sleep(0.1)
        self.send([90., 90., 90., 90., 90.], 10., 2000)
        time.sleep(2)

    def get_state(self):
        return self.state.copy()

    def send(self, angles, gripper, t):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = np.round(angles).tolist() + [gripper] + [int(t)]
        msg.name = [f"Joint{i}" for i in range(6)]

        self.pub.publish(msg)

    def control_gripper(self, g):

        self.send(self.state[:-1], g, 100)
        time.sleep(0.1)


    def control_joints(self, p):
        while not rospy.is_shutdown():
            self.send(p, self.state[-1], 150)
            time.sleep(0.15)
            if np.isclose(self.state[:-1], p,atol=3.).all():
                break

    def step(self, joint=None, gripper=None):
        if joint is not None:
            self.control_joints(joint)
        if gripper is not None:
            self.control_gripper(gripper)