"""【独自追加】診断用: グリッパの角度だけを変えたときに方策の出力がどう変わるかを調べる(Gazebo 不要)。
学習データでグリッパは常に開(-0.5236)なので、それ以外の値で出力が壊れないかを確認する。
使い方: cd ~/pai_training && ~/.pixi/bin/pixi run python ~/work/scripts/probe_gripper.py --model-path <checkpoint>
"""
import argparse
import torch
from lerobot.common.policies.act.modeling_act import ACTPolicy

ap = argparse.ArgumentParser()
ap.add_argument('--model-path', required=True)
args = ap.parse_args()
policy = ACTPolicy.from_pretrained(args.model_path).eval()
for name, buf in policy.normalize_inputs.named_buffers():
    if 'observation_state' in name:
        print(name, [round(v, 4) for v in buf.flatten().tolist()])
print('対象 -0.5、直立。出力 = 関節1, 関節2 の指令')
for grip in (-0.5236, -0.5, -0.3, 0.0):
    row = []
    for t in (0.0, 0.5, 1.0, 1.5):
        policy.reset()
        env = torch.full((1, 5), t)
        env[0, 0] = -0.5
        obs = {'observation.state': torch.tensor([[0.0, 0.0, 0.0, 0.0, grip]]), 'observation.environment_state': env}
        with torch.no_grad():
            a = policy.select_action(obs)[0]
        row.append(f't={t}: {a[0]:+.3f},{a[1]:+.3f}')
    print(f'グリッパ {grip:+.4f} | ' + ' | '.join(row))
