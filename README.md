# PAI最終課題: 模倣学習による「検査姿勢への到達」(CRANE+ V2 / Gazebo)

Physical AI講座2026 応用編1 の最終課題「AI×ロボティクスによる創造的タスクの実現」のコードと作業記録です。

製造業の外観検査を想定し、「指定した対象物の真上にカメラ(手先)を構え、静止し、元の姿勢に戻る」動作を
ACT(模倣学習)で学習させ、Gazebo 上の CRANE+ V2 アームで実行しました。

**デモ動画**: https://youtu.be/GCEBsRgTe0Q(約2分、等速)

## ベースライン(講義資料)と、このリポジトリで変えたこと

第5回講義「ROS 2入門(2/2)」の実践演習パイプラインを土台にしています。

- テレオペ: [matsuolab/physicalai_teleop](https://github.com/matsuolab/physicalai_teleop)
- 変換・学習・推論: [matsuolab/pai_training](https://github.com/matsuolab/pai_training)(LeRobot / ACT)

| 講義のパイプライン | このリポジトリ |
|---|---|
| タスク: お辞儀(関節2を往復) | タスク: 対象の方向に向き、真上 8cm で手先を真下に向けて静止 → 直立へ戻る |
| デモ: キーボードのテレオペ | デモ: 順運動学で求めた検査姿勢へ、なめらかな軌道を自動生成(`demo_inspect.py`) |
| 観測: 関節角 + 関節角(重複) | 観測: 関節角 + [対象の方向, 経過時間 x4] |
| 評価: 学習データとの比較(debug-policy) | 評価: デプロイ時の到達姿勢を順運動学で手先位置に直し、正解との距離[mm]で評価 |

改変・追加した箇所は、コード中に `【改変】` / `【独自追加】` とコメントしています。

## 結果

### 1. 土台の再現(お辞儀)

講義設定のままでは、お辞儀の途中で止まりました。観測が関節角だけだと「動き出し前 / 行き / 帰り」を区別できないことが原因でした。
静止フレームを削り、観測に経過時間を加えると、お辞儀を繰り返すようになりました(詳細は `notes/2026-10-16_工程2_まとめ.md`)。

### 2. 検査姿勢への到達(独自タスク)

デプロイ時の手先位置の誤差(正解の検査位置との距離)です。

| 試行 | 学習データ | 設定 | 学習した方向(±0.5, 0 rad) | 未学習の方向(±0.25 rad) |
|---|---|---|---|---|
| 1 | 3か所 x 4回 | batch 2, 3000 steps | 平均 39mm(向きが対象に従わない) | - |
| 2 | 3か所 x 4回 | batch 8, 3000 steps | **平均 4.2mm(成功)** | 約32mm(±0.5 に吸い寄せられる) |
| 3 | -0.5〜+0.5 から20か所 | batch 8, 3000 steps | 4.6〜41.6mm | 27〜29mm(改善せず) |

- 試行1 → 2: batch 2 ではデータを約2周しか見ておらず、対象の方向を覚えられていなかった。batch を 8 にして解決
- 試行3: 範囲全体のデータを与えても、未学習の方向への汎化は改善しなかった。
  関節角を直立に固定して対象の方向だけを変えると出力は方向どおりに並ぶため、方策は方向を区別できている。
  実際に動かすと「今の関節1の角度」に引っぱられて端まで流れていると考えている(考察は `notes/2026-10-16_工程3_設計.md`)

## フォルダ構成

```text
work/
  scripts/   独自スクリプト
  configs/   pai_training の設定(コンテナ内 ~/pai_training/configs/ に置いて使う)
  gazebo/    撮影用の Gazebo 起動ファイル(右側パネルを外したもの)
  logs/      学習・推論・評価のログ(CSV)
notes/       作業記録(試行錯誤、詰まった点と解決策)
run_container.ps1  講義の Docker イメージを起動する(Windows)
record.ps1         コンテナ内の Ubuntu デスクトップだけを録画する
```

### 主なスクリプト(`work/scripts/`)

| ファイル | 内容 |
|---|---|
| `inspect_pose.py` | URDF から順運動学を計算し、検査姿勢(関節2〜4)を探索する |
| `demo_inspect.py` | 検査姿勢へのデモを自動生成し、rosbag に記録する(テレオペと同じトピック・20Hz) |
| `trim_static.py` | 動作の前後の静止フレームを削ったデータセットを作る |
| `add_time_obs.py` | environment_state を経過時間に置き換える(お辞儀用) |
| `add_target_time_obs.py` | environment_state を [対象の方向, 経過時間 x4] に置き換える |
| `infer_time.py` / `infer_inspect.py` | 講義の `infer.py` を改変した推論ノード(経過時間・対象の方向を与える) |
| `eval_inspect.py` | 推論ログから到達姿勢・手先誤差[mm]・静止時間を集計する |
| `probe_target.py` / `probe_gripper.py` | 入力の一部だけを変えて方策の出力を調べる(診断用) |
| `spawn_targets.sh` | 動画用に、対象の位置へ目印の箱を置く |

## 実行の流れ(講義の Docker コンテナ内)

```bash
# 1. Gazebo を起動
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
# 2. デモを自動生成して記録(3か所 x 4回)
python3 ~/work/scripts/demo_inspect.py --task inspect --targets -0.5 0.0 0.5 --repeats 4
# 3. LeRobot 形式に変換(configs/conversion_config.yaml に conversion_config.inspect.yaml をコピーしてから)
cd ~/pai_training && pixi run convert
# 4. 静止フレームを削り、観測に対象の方向と経過時間を加える
python3 ~/work/scripts/trim_static.py --help
python3 ~/work/scripts/add_target_time_obs.py --help
# 5. 学習(configs/train_config.yaml に train_config.inspect_tt_b8.yaml をコピーしてから)
WANDB_MODE=offline pixi run train
# 6. デプロイと評価
pixi run python ~/work/scripts/infer_inspect.py --model-path ~/checkpoints/train/crane_plus_inspect_act_tt_b8 \
    --targets -0.5 0.0 0.5 --log ~/work/logs/infer.csv
python3 ~/work/scripts/eval_inspect.py ~/work/logs/infer.csv
```

rosbag・データセット・チェックポイント・動画は容量が大きいため、このリポジトリには含めていません。

## 環境

- Docker イメージ `airobotbook/ros2-desktop-ai-robot-book-humble`(ROS 2 Humble, Ignition Gazebo, CRANE+ V2)
- 学習は CPU のみ(GPU なし)。3000 steps で約60〜90分
