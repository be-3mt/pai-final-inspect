"""LeRobot データセット(v2.1)の observation.environment_state を「エピソード開始からの経過時間[秒]」に置き換えた別データセットを作る。

【独自追加】講義の pai_training には無いスクリプト。
理由: 観測が関節角のみだと、お辞儀の行きと帰りで同じ角度を通るときに
「進む」「戻る」の区別がつかず、平均して途中(約0.27 rad)で止まってしまった。
講義の変換では environment_state に observation.state と同じ関節角が入っているので、その枠に時間を入れる。
5次元の枠すべてに同じ経過時間を入れる(特徴量の形を講義の configs/crane_features.py のまま使うため)。

使い方(コンテナ内):
  ~/pai_training/.pixi/envs/default/bin/python ~/work/scripts/add_time_obs.py \
      --src ~/dataset/lerobot_dataset/crane_plus_pekori_trim \
      --dst ~/dataset/lerobot_dataset/crane_plus_pekori_trim_time
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
args = p.parse_args()

src, dst = Path(args.src).expanduser(), Path(args.dst).expanduser()
if dst.exists():
    raise SystemExit(f"出力先が既にあります: {dst}")
(dst / "meta").mkdir(parents=True)
for name in ["tasks.jsonl", "episodes.jsonl", "info.json"]:
    shutil.copy(src / "meta" / name, dst / "meta" / name)

info = json.loads((src / "meta" / "info.json").read_text())
fps = info["fps"]
dim = info["features"]["observation.environment_state"]["shape"][0]
episodes = [json.loads(l) for l in (src / "meta" / "episodes.jsonl").read_text().splitlines() if l]

for ep in episodes:
    ei = ep["episode_index"]
    rel = info["data_path"].format(episode_chunk=ei // info["chunks_size"], episode_index=ei)
    df = pd.read_parquet(src / rel)
    t = (df["frame_index"].to_numpy() / fps).astype(np.float32)
    df["observation.environment_state"] = list(np.repeat(t[:, None], dim, axis=1))

    (dst / rel).parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(dst / rel)

    ep_data = {k: np.stack(df[k]) if df[k].dtype == object else df[k].to_numpy() for k in df.columns
               if k in info["features"]}
    write_episode_stats(ei, compute_episode_stats(ep_data, info["features"]), dst)
    print(f"ep{ei}: {len(df)} フレーム, 経過時間 0.00〜{t[-1]:.2f}s")
