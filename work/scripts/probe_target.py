"""【独自追加】診断用: 学習済み方策に、関節角・経過時間を固定して「対象の方向」だけを変えた観測を入れ、
出力(関節1の指令)がどれだけ対象に反応するかを調べる。Gazebo は不要。
使い方: cd ~/pai_training && ~/.pixi/bin/pixi run python ~/work/scripts/probe_target.py --model-path <checkpoint>
"""
import argparse
import torch
from lerobot.common.policies.act.modeling_act import ACTPolicy

ap = argparse.ArgumentParser()
ap.add_argument('--model-path', required=True)
args = ap.parse_args()
policy = ACTPolicy.from_pretrained(args.model_path).eval()
print('t[s]  ' + '  '.join(f'対象{az:+.2f}' for az in (-0.5, -0.25, 0.0, 0.25, 0.5)) + '   (関節1の指令 rad)')
for t in (0.0, 1.0, 2.0, 3.0, 5.0):
    outs = []
    for az in (-0.5, -0.25, 0.0, 0.25, 0.5):
        policy.reset()
        env = torch.full((1, 5), t)
        env[0, 0] = az
        obs = {'observation.state': torch.tensor([[0.0, 0.0, 0.0, 0.0, -0.5236]]),  # 直立から動き出す前の関節角
               'observation.environment_state': env}
        with torch.no_grad():
            outs.append(policy.select_action(obs)[0, 0].item())
    print(f'{t:4.1f}  ' + '  '.join(f'{v:+9.3f}' for v in outs))
