#!/usr/bin/env python3
"""【独自】検査姿勢(手先が真下を向き、対象の真上に来る姿勢)の関節角を求める。

URDF(xacro 展開済み)から関節の原点・回転軸を読み、順運動学で手先位置と向きを計算する。
関節1は対象の方向(方位角)にそのまま合わせ、関節2〜4は格子探索で決める。

使い方(コンテナ内):
  xacro ~/airobot_ws/src/crane_plus/crane_plus_description/urdf/crane_plus.urdf.xacro > /tmp/crane.urdf
  python3 ~/work/scripts/inspect_pose.py /tmp/crane.urdf --dist 0.12 --height 0.08
"""
import argparse
import itertools
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
# 手先(カメラ想定点)は joint4 から先端方向へ TIP_OFFSET [m] の位置とする(指先付近)
TIP_OFFSET = 0.075


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis, q):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def load_chain(urdf_path):
    root = ET.parse(urdf_path).getroot()
    chain = []
    for name in ARM_JOINTS:
        j = root.find(f"joint[@name='{name}']")
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz").split()]
        rpy = [float(v) for v in o.get("rpy").split()]
        axis = [float(v) for v in j.find("axis").get("xyz").split()]
        lim = j.find("limit")
        chain.append((np.array(xyz), rpy_to_mat(*rpy), axis, float(lim.get("lower")), float(lim.get("upper"))))
    return chain


def fk(chain, q):
    """手先位置と、手先の向き(リンクの z 軸 = 先端方向)を返す。"""
    R, p = np.eye(3), np.zeros(3)
    for (xyz, R0, axis, _, _), qi in zip(chain, q):
        p = p + R @ xyz
        R = R @ R0 @ axis_angle(axis, qi)
    tip_dir = R[:, 2]
    return p + TIP_OFFSET * tip_dir, tip_dir


def solve(chain, dist, height, azimuth):
    """対象 (dist, azimuth) の真上 height [m] に、手先を真下に向けて構える関節角を探す。"""
    target = np.array([dist * np.cos(azimuth), dist * np.sin(azimuth), height])
    best = None
    # 粗い格子 → 最良点の周りを細かく、の2段階探索
    grids = [np.arange(lo, hi, 0.1) for (_, _, _, lo, hi) in chain[1:]]
    for _ in range(2):
        for q2, q3, q4 in itertools.product(*grids):
            q = [azimuth, q2, q3, q4]
            tip, d = fk(chain, q)
            cost = np.linalg.norm(tip - target) + 0.05 * (1 + d[2])  # d[2]=-1 が真下
            if best is None or cost < best[0]:
                best = (cost, q, tip, d)
        c = best[1]
        grids = [np.arange(c[i] - 0.1, c[i] + 0.1, 0.005) for i in (1, 2, 3)]
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--dist", type=float, default=0.12, help="根元から対象までの水平距離 [m]")
    ap.add_argument("--height", type=float, default=0.08, help="手先の高さ [m](ロボット基準)")
    ap.add_argument("--azimuths", type=float, nargs="+", default=[-0.5, -0.25, 0.0, 0.25, 0.5])
    args = ap.parse_args()

    chain = load_chain(args.urdf)
    print(f"初期姿勢(全関節0)の手先: {np.round(fk(chain, [0, 0, 0, 0])[0], 3)}")
    # 関節1は方向を変えるだけなので、関節2〜4は方位0で1回解けば全方向で共通
    _, q0, _, _ = solve(chain, args.dist, args.height, 0.0)
    for az in args.azimuths:
        q = [az] + list(q0[1:])
        tip, d = fk(chain, q)
        target = np.array([args.dist * np.cos(az), args.dist * np.sin(az), args.height])
        cost = np.linalg.norm(tip - target)
        print(f"az={az:+.2f}  q=[{', '.join(f'{v:+.3f}' for v in q)}]  "
              f"tip={np.round(tip, 3)}  down={-d[2]:.3f}  err={cost*1000:.1f}mm")


if __name__ == "__main__":
    main()
