#!/usr/bin/env python3
"""【独自追加】検査姿勢タスクのデプロイ結果(infer_inspect.py の CSV)を対象ごとに集計する。

到達姿勢: 各周期で関節2が最大値から 0.01 rad 以内にある区間(=検査姿勢で止まっている区間)の平均。
正解: 関節1 = 対象の方向、関節2〜4 = inspect_pose.py で求めた検査姿勢。
手先の誤差は順運動学(inspect_pose.py)で、正解の手先位置(対象の真上 8cm)との距離[mm]。

使い方(コンテナ内):
  python3 ~/work/scripts/eval_inspect.py ~/work/logs/infer_crane_plus_inspect_act_tt.csv --urdf /tmp/crane.urdf
"""
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from inspect_pose import fk, load_chain  # noqa: E402

INSPECT_Q = np.array([0.330, -1.187, -1.630])  # 関節2, 3, 4(demo_inspect.py と同じ)
HOLD_THR = 0.01


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--urdf', default='/tmp/crane.urdf')
    args = ap.parse_args()

    chain = load_chain(args.urdf)
    cycles = defaultdict(list)
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            # CSV の列はテレオペの並び(joint1, joint2, joint4, joint3) → joint1〜4 の順に直す
            q = [float(r[f'state_crane_plus_joint{i}']) for i in (1, 2, 3, 4)]
            cycles[(int(r['cycle']), float(r['target']))].append(q)

    print(f"{'対象[rad]':>9} {'到達 関節1〜4 [rad]':>34} {'関節誤差max':>10} {'手先誤差':>8} {'真下度':>6} {'静止':>5} {'最後(直立からのずれ)':>10}")
    rows = []
    for (cycle, target), qs in sorted(cycles.items()):
        qs = np.array(qs)
        j2 = qs[:, 1]
        hold = np.abs(j2 - j2.max()) < HOLD_THR
        reached = qs[hold].mean(axis=0)
        gt = np.concatenate([[target], INSPECT_Q])
        joint_err = np.abs(reached - gt).max()
        tip, d = fk(chain, reached)
        tip_gt, _ = fk(chain, gt)
        tip_err = np.linalg.norm(tip - tip_gt) * 1000
        end_dev = np.abs(qs[-1]).max()
        rows.append((joint_err, tip_err))
        print(f"{target:+9.2f} [{', '.join(f'{v:+.3f}' for v in reached)}] {joint_err:10.3f} {tip_err:6.1f}mm "
              f"{-d[2]:6.3f} {hold.sum() / 20:4.1f}s {end_dev:10.3f}")
    rows = np.array(rows)
    print(f"平均: 関節誤差max {rows[:, 0].mean():.3f} rad, 手先誤差 {rows[:, 1].mean():.1f} mm")


if __name__ == '__main__':
    main()
