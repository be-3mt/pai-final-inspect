"""変換済み LeRobot データセット(v2.1)から、動き出し前・戻り後の静止フレームを削った別データセットを作る。

【独自追加】講義の pai_training には無いスクリプト。
理由: pekori データの約34%が直立静止で、観測が関節角のみのため
「直立なら止まる」を学習してしまい、デプロイで動き出さなかった。

使い方(コンテナ内):
  ~/pai_training/.pixi/envs/default/bin/python ~/work/scripts/trim_static.py \
      --src ~/dataset/lerobot_dataset/crane_plus_pekori \
      --dst ~/dataset/lerobot_dataset/crane_plus_pekori_trim
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
from lerobot.common.datasets.compute_stats import compute_episode_stats
from lerobot.common.datasets.utils import write_episode_stats

p = argparse.ArgumentParser()
p.add_argument("--src", required=True)
p.add_argument("--dst", required=True)
p.add_argument("--joint", type=int, default=1, help="判定に使う関節の列番号(1=joint2)")
p.add_argument("--state-thr", type=float, default=0.02, help="直立とみなす角度[rad]")
p.add_argument("--action-thr", type=float, default=0.005, help="指令が動いたとみなす変化量[rad]")
p.add_argument("--margin", type=int, default=2, help="動き区間の前後に残すフレーム数")
args = p.parse_args()

src, dst = Path(args.src).expanduser(), Path(args.dst).expanduser()
if dst.exists():
    raise SystemExit(f"出力先が既にあります: {dst}")
(dst / "meta").mkdir(parents=True)
shutil.copy(src / "meta" / "tasks.jsonl", dst / "meta" / "tasks.jsonl")

info = json.loads((src / "meta" / "info.json").read_text())
fps = info["fps"]
episodes = [json.loads(l) for l in (src / "meta" / "episodes.jsonl").read_text().splitlines() if l]

global_index = 0
new_episodes = []
for ep in episodes:
    ei = ep["episode_index"]
    rel = info["data_path"].format(episode_chunk=ei // info["chunks_size"], episode_index=ei)
    df = pd.read_parquet(src / rel)
    s = np.stack(df["observation.state"])[:, args.joint]
    a = np.stack(df["action"])[:, args.joint]
    moving = (s > args.state_thr) | (np.abs(a - a[0]) > args.action_thr)
    idx = np.flatnonzero(moving)
    start = max(idx[0] - args.margin, 0)
    end = min(idx[-1] + args.margin, len(df) - 1)
    df = df.iloc[start : end + 1].reset_index(drop=True)

    n = len(df)
    df["frame_index"] = np.arange(n, dtype=np.int64)
    df["timestamp"] = (np.arange(n) / fps).astype(np.float32)
    df["index"] = np.arange(global_index, global_index + n, dtype=np.int64)
    global_index += n

    (dst / rel).parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(dst / rel)

    ep_data = {k: np.stack(df[k]) if df[k].dtype == object else df[k].to_numpy() for k in df.columns
               if k in info["features"]}
    write_episode_stats(ei, compute_episode_stats(ep_data, info["features"]), dst)
    new_episodes.append({**ep, "length": n})
    print(f"ep{ei}: {len(s)} -> {n} フレーム (残した範囲 {start}〜{end}, {start / fps:.2f}s〜{end / fps:.2f}s)")

with open(dst / "meta" / "episodes.jsonl", "w") as f:
    for ep in new_episodes:
        f.write(json.dumps(ep) + "\n")
info["total_frames"] = global_index
(dst / "meta" / "info.json").write_text(json.dumps(info, indent=4))
print(f"合計 {sum(e['length'] for e in episodes)} -> {global_index} フレーム")
