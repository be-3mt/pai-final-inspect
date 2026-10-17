"""LeRobot データセット(v2.1)の observation.environment_state を
[対象の方向[rad], 経過時間[秒] x4] に置き換えた別データセットを作る。

【独自追加】講義の pai_training には無いスクリプト(add_time_obs.py の検査姿勢タスク版)。
理由: 対象が左・中央・右のどこにあるかは関節角からは分からない(動き出し前は全部直立)。
      そこで対象の方向を観測として与える。経過時間は pekori で「今どの段階か」を伝えるのに効いたので引き継ぐ。
対象の方向は demo_inspect.py が rosbag フォルダに保存した targets.json の azimuth(ばらつき込みの実際の値)を使う。
特徴量の形は講義の configs/crane_features.py のまま(5次元)。

使い方(コンテナ内):
  ~/pai_training/.pixi/envs/default/bin/python ~/work/scripts/add_target_time_obs.py \
      --src ~/dataset/lerobot_dataset/crane_plus_inspect_trim \
      --dst ~/dataset/lerobot_dataset/crane_plus_inspect_trim_tt \
      --bags ~/dataset/rosbags/inspect
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
p.add_argument("--bags", required=True, help="変換元の rosbag フォルダ(targets.json がある場所)")
args = p.parse_args()

src, dst, bags = (Path(x).expanduser() for x in (args.src, args.dst, args.bags))
if dst.exists():
    raise SystemExit(f"出力先が既にあります: {dst}")
targets = json.loads((bags / "targets.json").read_text())
# 変換(rosbag2lerobot.py)と同じ並び順(名前の文字順: ep0, ep1, ep10, ep11, ep2, ...)でエピソード番号と対応させる
bag_names = [d.parent.name for d in sorted(bags.rglob("*.db3"))]

(dst / "meta").mkdir(parents=True)
for name in ["tasks.jsonl", "episodes.jsonl", "info.json"]:
    shutil.copy(src / "meta" / name, dst / "meta" / name)

info = json.loads((src / "meta" / "info.json").read_text())
fps = info["fps"]
dim = info["features"]["observation.environment_state"]["shape"][0]
episodes = [json.loads(l) for l in (src / "meta" / "episodes.jsonl").read_text().splitlines() if l]
assert len(episodes) == len(bag_names), f"エピソード数が合いません: {len(episodes)} vs {len(bag_names)}"

for ep in episodes:
    ei = ep["episode_index"]
    az = targets[bag_names[ei]]["azimuth"]
    rel = info["data_path"].format(episode_chunk=ei // info["chunks_size"], episode_index=ei)
    df = pd.read_parquet(src / rel)
    t = (df["frame_index"].to_numpy() / fps).astype(np.float32)
    env = np.repeat(t[:, None], dim, axis=1)
    env[:, 0] = az
    df["observation.environment_state"] = list(env.astype(np.float32))

    (dst / rel).parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(dst / rel)

    ep_data = {k: np.stack(df[k]) if df[k].dtype == object else df[k].to_numpy() for k in df.columns
               if k in info["features"]}
    write_episode_stats(ei, compute_episode_stats(ep_data, info["features"]), dst)
    # 確認用: 関節1の到達角と対象方向が合っているか
    j1_peak = np.stack(df["observation.state"])[:, 0]
    j1_peak = j1_peak[np.argmax(np.abs(j1_peak))]
    print(f"episode {ei} ({bag_names[ei]}): 対象 {az:+.3f} rad / 関節1の最大 {j1_peak:+.3f} rad, "
          f"{len(df)} フレーム, 経過時間 0.00〜{t[-1]:.2f}s")
