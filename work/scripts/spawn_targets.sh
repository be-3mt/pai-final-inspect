#!/bin/bash
# 【独自追加】検査対象の目印(見た目だけの小さな箱)を Gazebo に置く。動画で対象の位置が分かるようにするためのもの。
# 位置: ロボット根元から水平 12cm、方向 = 関節1の角度(demo_inspect.py / inspect_pose.py と同じ)。
# 当たり判定なし・固定なので、アームの動きには影響しない。
# 使い方(コンテナ内、Gazebo 起動中): bash ~/work/scripts/spawn_targets.sh [方向rad ...]   例: bash spawn_targets.sh -0.5 0.0 0.5
source /opt/ros/humble/setup.bash
DIST=0.12
BASE_Z=1.015   # ロボットを置いた高さ(crane_plus_with_table.launch.py の -z と同じ)
TARGETS=${@:-"-0.5 0.0 0.5"}
i=0
for AZ in $TARGETS; do
  read X Y < <(python3 -c "import math; print($DIST*math.cos($AZ), $DIST*math.sin($AZ))")
  SDF="<sdf version='1.6'><model name='target_$i'><static>true</static><link name='link'>
    <visual name='v'><geometry><box><size>0.03 0.03 0.01</size></box></geometry>
    <material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material></visual>
    </link></model></sdf>"
  ros2 run ros_gz_sim create -string "$SDF" -name "target_$i" -x "$X" -y "$Y" -z "$(python3 -c "print($BASE_Z + 0.005)")" -allow_renaming true 2>&1 | grep -i -E 'error|success|requested' || true
  echo "目印 target_$i: 方向 $AZ rad, x=$X y=$Y"
  i=$((i+1))
done
