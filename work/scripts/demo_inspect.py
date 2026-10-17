#!/usr/bin/env python3
"""【独自追加】検査姿勢への到達タスクのデモ(お手本)を自動生成し、rosbag に記録する。

テレオペ(physicalai_teleop/teleop_keyboard.py)と同じトピック・形式・周期(20Hz)で指令を送るので、
記録した rosbag は講義の変換(pixi run convert)にそのまま通る。
キー操作だと1エピソード約150回の入力が必要で動きもばらつくため、なめらかな軌道をスクリプトで作る。

1エピソードの流れ(秒数は各 ±10% でばらつかせる):
  直立で待機 0.5s → 検査姿勢へ移動 3.0s → 静止(撮像を想定) 3.0s(アームの追従遅れで実際の静止は約2s) → 直立へ戻る 3.0s → 待機 1.0s
検査姿勢: 対象(根元から 12cm 先)の真上 8cm で手先を真下に向ける(work/scripts/inspect_pose.py で算出)。

使い方(コンテナ内、Gazebo を起動した状態で):
  python3 ~/work/scripts/demo_inspect.py --task inspect --targets -0.5 0.0 0.5 --repeats 4
  → ~/dataset/rosbags/inspect/ep0, ep1, ... と targets.json(エピソード番号 → 対象方向[rad])
  汎化の改善用(対象を範囲内から選ぶ): python3 ~/work/scripts/demo_inspect.py --task inspect_uniform --uniform 20 --seed 1
"""
import argparse
import json
import random
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# テレオペと同じ定義(関節3と4の順番が入れ替わっている点も合わせる)
ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'
GRIPPER_OPEN = -0.5236
RATE = 20.0  # Hz(テレオペと同じ)

# 検査姿勢の関節2〜4(inspect_pose.py の結果。関節1は対象方向)
INSPECT_Q2, INSPECT_Q3, INSPECT_Q4 = 0.330, -1.187, -1.630

TOPICS = ['/joint_states', '/crane_plus_arm_controller/joint_trajectory',
          '/crane_plus_gripper_controller/joint_trajectory']


def min_jerk(s):
    """0→1 をなめらかにつなぐ補間(速度・加速度が両端で 0)。"""
    s = min(max(s, 0.0), 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class DemoNode(Node):
    def __init__(self):
        super().__init__('crane_plus_inspect_demo')
        self.arm_pub = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.joint_pos = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

    def on_joint_states(self, msg):
        self.joint_pos = dict(zip(msg.name, msg.position))

    def publish(self, q1, q2, q3, q4):
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [q1, q2, q4, q3]  # ARM_JOINT_NAMES の順
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        jt.points.append(pt)
        self.arm_pub.publish(jt)
        jg = JointTrajectory()
        jg.joint_names = [GRIPPER_JOINT_NAME]
        pg = JointTrajectoryPoint()
        pg.positions = [GRIPPER_OPEN]
        pg.time_from_start.nanosec = int(0.1 * 1e9)
        jg.points.append(pg)
        self.gripper_pub.publish(jg)

    def run_segments(self, segments):
        """segments: [(秒数, 開始姿勢, 終了姿勢)] を 20Hz で順に送る。"""
        for dur, q_from, q_to in segments:
            n = max(1, int(round(dur * RATE)))
            for i in range(1, n + 1):
                s = min_jerk(i / n)
                self.publish(*[a + (b - a) * s for a, b in zip(q_from, q_to)])
                time.sleep(1.0 / RATE)

    def wait_upright(self, timeout=10.0):
        """直立(全関節ほぼ 0)になるまで直立指令を送り続ける。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.publish(0.0, 0.0, 0.0, 0.0)
            time.sleep(1.0 / RATE)
            if self.joint_pos and all(abs(self.joint_pos.get(f'crane_plus_joint{i}', 1.0)) < 0.01 for i in range(1, 5)):
                return True
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', default='inspect')
    ap.add_argument('--targets', type=float, nargs='+', default=[-0.5, 0.0, 0.5], help='対象方向(関節1の角度)[rad]')
    ap.add_argument('--repeats', type=int, default=4, help='対象ごとの回数')
    ap.add_argument('--jitter', type=float, default=0.03, help='対象方向・姿勢のばらつき[rad]')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--no-record', action='store_true', help='記録せず動きだけ確認する')
    ap.add_argument('--uniform', type=int, default=0,
                    help='0 以外なら --targets を使わず、-0.5〜+0.5 を N 等分した各区間から1つずつ対象を選ぶ(汎化の改善用)')
    args = ap.parse_args()

    rng = random.Random(args.seed)
    root = Path.home() / 'dataset' / 'rosbags' / args.task
    meta_path = root / 'targets.json'
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    # 既存の最大番号の続きから振る(欠番を埋めると、その後の既存フォルダと番号がぶつかるため)
    existing = [int(p.name[2:]) for p in root.glob('ep*') if p.name[2:].isdigit()] if root.exists() else []
    next_ep = max(existing) + 1 if existing else 0

    # 対象の順番はばらばらにする(同じ対象が続かないように)
    plan = [t for t in args.targets for _ in range(args.repeats)]
    if args.uniform:
        # 区間ごとに1つ選ぶ(層別サンプリング)ので、範囲全体に偏りなく散らばる
        edges = [-0.5 + i / args.uniform for i in range(args.uniform + 1)]
        plan = [rng.uniform(lo, hi) for lo, hi in zip(edges[:-1], edges[1:])]
    rng.shuffle(plan)

    rclpy.init()
    node = DemoNode()
    # /joint_states の受信は別スレッドで回し続ける(spin_once を指令ループ内で呼ぶと値が古いまま止まった)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        for target in plan:
            az = target if args.uniform else target + rng.uniform(-args.jitter, args.jitter)
            pose = [az] + [q + rng.uniform(-args.jitter, args.jitter) for q in (INSPECT_Q2, INSPECT_Q3, INSPECT_Q4)]
            k = lambda base: base * rng.uniform(0.9, 1.1)  # 秒数のばらつき
            zero = [0.0, 0.0, 0.0, 0.0]
            segments = [(k(0.5), zero, zero), (k(3.0), zero, pose), (k(3.0), pose, pose),
                        (k(3.0), pose, zero), (k(1.0), zero, zero)]

            if not node.wait_upright():
                print('警告: 直立に戻りきりませんでした(Gazebo が起動しているか確認)')

            ep_dir = root / f'ep{next_ep}'
            rec = None
            if not args.no_record:
                root.mkdir(parents=True, exist_ok=True)
                rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(ep_dir)] + TOPICS,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                # 記録の立ち上がり待ち。この間も直立指令を送り続ける(送らないと、記録の接続が遅れたとき
                # 動き出しの途中から記録が始まった。静止部分は後で trim_static.py が削る)
                node.run_segments([(2.0, [0.0] * 4, [0.0] * 4)])

            print(f'ep{next_ep}: 対象 {target:+.2f} rad(実際 {az:+.3f})、'
                  f'姿勢 [{", ".join(f"{v:+.3f}" for v in pose)}]、合計 {sum(s[0] for s in segments):.1f}s')
            node.run_segments(segments)

            if rec is not None:
                # 記録中は Gazebo の進みが遅れ、指令を送り終えてもアームが戻りきっていなかった。
                # 直立に戻ったのを確認し、さらに 1s 静止を記録してから止める(静止部分は trim_static.py が削る)
                if not node.wait_upright(timeout=15.0):
                    print('警告: 記録終了時に直立へ戻りきっていません')
                node.run_segments([(1.0, [0.0] * 4, [0.0] * 4)])
                rec.send_signal(signal.SIGINT)
                rec.wait(timeout=15)
                meta[f'ep{next_ep}'] = {'target': target, 'azimuth': az, 'pose': pose}
                meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False))
                next_ep += 1
    finally:
        executor.shutdown()  # 受信スレッドを止めてから終了する
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
