#!/usr/bin/env python3
# 【改変】講義の pai_training/scripts/infer.py をコピーして改変したもの。
# observation.environment_state に関節角ではなく「経過時間[秒]」を入れる(work/scripts/add_time_obs.py で作ったデータセット用)。
# 経過時間は publish_action の呼び出し回数 / 20Hz で数え、--period 秒ごとに 0 に戻してお辞儀を繰り返させる。
# 実行(コンテナ内): cd ~/pai_training && ~/.pixi/bin/pixi run python ~/work/scripts/infer_time.py --model-path <checkpoint>
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import torch
import argparse
import numpy as np

from lerobot.common.policies.act.modeling_act import ACTPolicy

ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'

class InferenceNode(Node):
    def __init__(self, model_path, period):
        super().__init__('crane_plus_act_infer')
        self.arm_pub = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.device = torch.device('cpu')
        self.policy.to(self.device)
        self.last_action = np.zeros(5, dtype=np.float32)
        self.timer = self.create_timer(0.05, self.publish_action)  # 20Hz
        self.latest_joint_state = None
        self.step = 0  # 【改変】経過時間を数えるためのカウンタ
        self.period_steps = int(round(period * 20))  # 【改変】この回数ごとに経過時間を 0 に戻す

    def joint_state_callback(self, msg):
        # Extract joint positions in the order of ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
        name_to_pos = dict(zip(msg.name, msg.position))
        arm_pos = [name_to_pos.get(j, 0.0) for j in ARM_JOINT_NAMES]
        gripper_pos = name_to_pos.get(GRIPPER_JOINT_NAME, 0.0)
        self.latest_joint_state = np.array(arm_pos + [gripper_pos], dtype=np.float32)

    def publish_action(self):
        if self.latest_joint_state is None:
            return
        # 【改変】周期の頭で方策の内部キュー(未実行の行動)を捨て、新しいお辞儀として始める
        if self.step % self.period_steps == 0:
            self.policy.reset()
        t = (self.step % self.period_steps) / 20.0
        self.step += 1
        # Prepare observation dict for ACTPolicy
        obs = {
            'observation.state': torch.tensor(self.latest_joint_state, dtype=torch.float32).unsqueeze(0),  # (1, 5)
            'observation.environment_state': torch.full((1, 5), t, dtype=torch.float32),  # 【改変】講義: 関節角 → 経過時間[秒]
        }
        with torch.no_grad():
            action = self.policy.select_action(obs).cpu().numpy().flatten()  # (5,)
        self.last_action = action
        # Publish to arm and gripper
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = action[:4].tolist()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        jt.points.append(pt)
        self.arm_pub.publish(jt)
        jt_g = JointTrajectory()
        jt_g.joint_names = [GRIPPER_JOINT_NAME]
        pt_g = JointTrajectoryPoint()
        pt_g.positions = [float(action[4])]
        pt_g.time_from_start.sec = 0
        pt_g.time_from_start.nanosec = int(0.1 * 1e9)
        jt_g.points.append(pt_g)
        self.gripper_pub.publish(jt_g)
        print(f"t={t:5.2f} action:", action[:4], "gripper:", action[4])  # 【改変】経過時間も表示

def main():
    parser = argparse.ArgumentParser(description='ACTPolicy inference node for CRANE+ V2 (ROS2)')
    parser.add_argument('--model-path', type=str, required=True, help='Path to trained ACTPolicy model directory')
    parser.add_argument('--period', type=float, default=11.0, help='【改変】経過時間を 0 に戻す周期[秒](学習データは約10.3〜11.1秒)')
    args = parser.parse_args()
    rclpy.init()
    node = InferenceNode(args.model_path, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
