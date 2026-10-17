"""【独自追加】rosbag の中身を確認する: 各エピソードの長さ・トピック数・関節ごとの動いた範囲
使い方: python3 inspect_bags.py ~/dataset/rosbags/pekori
"""
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

root = Path(sys.argv[1]).expanduser()
for ep in sorted(p.parent for p in root.rglob('*.db3')):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(ep), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    counts, pos, t = {}, {}, []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == '/joint_states':
            msg = deserialize_message(data, JointState)
            t.append(ts)
            for n, p in zip(msg.name, msg.position):
                pos.setdefault(n, []).append(p)
    dur = (t[-1] - t[0]) / 1e9 if t else 0
    print(f'== {ep.name}: {dur:.1f}秒')
    for k in ['/joint_states', '/crane_plus_arm_controller/joint_trajectory',
              '/crane_plus_gripper_controller/joint_trajectory']:
        print(f'   {k}: {counts.get(k, 0)}件')
    for n in sorted(pos):
        v = pos[n]
        print(f'   {n:28s} 最小 {min(v):+.2f}  最大 {max(v):+.2f}  動いた幅 {max(v)-min(v):.2f} rad')
