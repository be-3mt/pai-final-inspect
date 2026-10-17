#!/bin/bash
# 【独自追加】rosbag 記録用ヘルパー(講義の `ros2 bag record` を短く打てるようにしたもの)
# 変換(pai_training/scripts/rosbag2lerobot.py)に必要な3トピックだけを記録する。
# 使い方: rec <タスク名> <エピソード番号>   例: rec pekori 0
# 止めるときは Ctrl+C
TASK=${1:?タスク名を指定してください(例: rec pekori 0)}
EP=${2:?エピソード番号を指定してください(例: rec pekori 0)}
OUT=~/dataset/rosbags/$TASK/ep$EP
if [ -e "$OUT" ]; then
  echo "エラー: $OUT はすでにあります。別の番号を指定してください。"
  echo "既存: $(ls ~/dataset/rosbags/$TASK 2>/dev/null | tr '\n' ' ')"
  exit 1
fi
mkdir -p ~/dataset/rosbags/$TASK
echo "記録開始: $OUT (止めるときは Ctrl+C)"
ros2 bag record -o "$OUT" \
  /joint_states \
  /crane_plus_arm_controller/joint_trajectory \
  /crane_plus_gripper_controller/joint_trajectory
