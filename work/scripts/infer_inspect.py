#!/usr/bin/env python3
# 【改変】講義の pai_training/scripts/infer.py をコピーして改変したもの(infer_time.py の検査姿勢タスク版)。
# observation.environment_state に [対象の方向[rad], 経過時間[秒] x4] を入れる(work/scripts/add_target_time_obs.py で作ったデータセット用)。
# --targets で指定した対象を1周期ずつ順に与え、全部終わったら直立指令を送って終了する。
# 評価用に、20Hz ごとの関節角と指令を CSV に保存する(work/scripts/eval_inspect.py で集計)。
# 実行(コンテナ内):
#   cd ~/pai_training && ~/.pixi/bin/pixi run python ~/work/scripts/infer_inspect.py \
#       --model-path ~/checkpoints/train/crane_plus_inspect_act_tt --targets -0.5 0.0 0.5
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import torch
import argparse
import csv
from pathlib import Path
import numpy as np

from lerobot.common.policies.act.modeling_act import ACTPolicy

ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'
GRIPPER_OPEN = -0.5236  # 【改変】テレオペ・デモと同じ開の角度

class InferenceNode(Node):
    def __init__(self, model_path, targets, period, log_path):
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
        self.targets = targets  # 【改変】周期ごとに与える対象の方向
        self.step = 0  # 【改変】経過時間を数えるためのカウンタ
        self.period_steps = int(round(period * 20))
        self.done = False
        self.ready = False  # 【改変】開始姿勢がそろうまで方策を動かさない
        # 【改変】評価用ログ
        self.log_file = open(log_path, 'w', newline='')
        self.log = csv.writer(self.log_file)
        self.log.writerow(['cycle', 'target', 't'] + [f'state_{n}' for n in ARM_JOINT_NAMES]
                          + [f'action_{n}' for n in ARM_JOINT_NAMES])

    def joint_state_callback(self, msg):
        # Extract joint positions in the order of ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
        name_to_pos = dict(zip(msg.name, msg.position))
        arm_pos = [name_to_pos.get(j, 0.0) for j in ARM_JOINT_NAMES]
        gripper_pos = name_to_pos.get(GRIPPER_JOINT_NAME, 0.0)
        self.latest_joint_state = np.array(arm_pos + [gripper_pos], dtype=np.float32)

    def publish_arm(self, positions, gripper):
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        jt.points.append(pt)
        self.arm_pub.publish(jt)
        jt_g = JointTrajectory()
        jt_g.joint_names = [GRIPPER_JOINT_NAME]
        pt_g = JointTrajectoryPoint()
        pt_g.positions = [float(gripper)]
        pt_g.time_from_start.sec = 0
        pt_g.time_from_start.nanosec = int(0.1 * 1e9)
        jt_g.points.append(pt_g)
        self.gripper_pub.publish(jt_g)

    def publish_action(self):
        if self.latest_joint_state is None or self.done:
            return
        # 【改変】開始前に「直立・グリッパ開」をそろえる(デモの開始状態と同じにする)。
        # 学習データのグリッパは常に開(-0.5236)で標準偏差がほぼ 0 のため、0.02 ずれただけで正規化後の値が
        # 極端に大きくなり方策の出力が固まった(Gazebo 起動直後はグリッパが 0 で、1周目だけ動かなかった)
        if not self.ready:
            self.publish_arm([0.0] * 4, GRIPPER_OPEN)
            if abs(self.latest_joint_state[4] - GRIPPER_OPEN) < 0.005 and np.abs(self.latest_joint_state[:4]).max() < 0.01:
                self.ready = True
                print("開始姿勢(直立・グリッパ開)を確認。推論を始めます")
            return
        cycle = self.step // self.period_steps
        if cycle >= len(self.targets):
            # 【改変】全対象が終わったら直立に戻して終了
            self.publish_arm([0.0] * 4, self.last_action[4])
            self.done = True
            return
        # 【改変】周期の頭で方策の内部キュー(未実行の行動)を捨て、新しい対象として始める
        if self.step % self.period_steps == 0:
            self.policy.reset()
            print(f"--- 対象 {cycle + 1}/{len(self.targets)}: 方向 {self.targets[cycle]:+.2f} rad ---")
        target = self.targets[cycle]
        t = (self.step % self.period_steps) / 20.0
        self.step += 1
        env = torch.full((1, 5), t, dtype=torch.float32)
        env[0, 0] = target
        # 【改変】グリッパの観測は学習時の値(常に開)に固定する。学習データで標準偏差がほぼ 0 のため、
        # 実測値がわずかでもずれると正規化後に極端な値となり出力が固まる(0.005 rad 以内に合わせても失敗した)。
        # このタスクではグリッパを使わないので、固定しても方策に必要な情報は失われない
        state = self.latest_joint_state.copy()
        state[4] = GRIPPER_OPEN
        # Prepare observation dict for ACTPolicy
        obs = {
            'observation.state': torch.tensor(state, dtype=torch.float32).unsqueeze(0),  # (1, 5)
            'observation.environment_state': env,  # 【改変】講義: 関節角 → [対象方向, 経過時間x4]
        }
        with torch.no_grad():
            action = self.policy.select_action(obs).cpu().numpy().flatten()  # (5,)
        self.last_action = action
        self.publish_arm(action[:4], action[4])
        self.log.writerow([cycle, target, f'{t:.2f}'] + [f'{v:.4f}' for v in self.latest_joint_state[:4]]
                          + [f'{v:.4f}' for v in action[:4]])
        if self.step % 20 == 0:
            print(f"t={t:5.2f} action:", np.round(action[:4], 3))

def main():
    parser = argparse.ArgumentParser(description='ACTPolicy inference node for CRANE+ V2 (ROS2)')
    parser.add_argument('--model-path', type=str, required=True, help='Path to trained ACTPolicy model directory')
    parser.add_argument('--targets', type=float, nargs='+', default=[-0.5, 0.0, 0.5], help='【改変】順に与える対象の方向[rad]')
    parser.add_argument('--period', type=float, default=14.0, help='【改変】1対象あたりの秒数(学習データは約11.7〜14.1秒)')
    parser.add_argument('--log', type=str, default=None, help='【改変】評価用 CSV の保存先')
    args = parser.parse_args()
    log_path = Path(args.log or f"~/work/logs/infer_{Path(args.model_path).name}.csv").expanduser()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = InferenceNode(args.model_path, args.targets, args.period, log_path)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.close()
        print(f"ログ: {log_path}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
