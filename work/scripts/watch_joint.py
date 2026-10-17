"""【独自追加】推論中の関節角を一定間隔で表示する(デプロイの数値確認用)
使い方: python3 watch_joint.py <秒数> [関節名]
"""
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30
name = sys.argv[2] if len(sys.argv) > 2 else 'crane_plus_joint2'
rclpy.init()
node = Node('watch_joint')
latest = {}
node.create_subscription(JointState, '/joint_states',
                         lambda m: latest.update(dict(zip(m.name, m.position))), 10)
t0 = time.time(); next_t = 0
while time.time() - t0 < dur:
    rclpy.spin_once(node, timeout_sec=0.05)
    if time.time() - t0 >= next_t and name in latest:
        print(f'{time.time()-t0:5.1f}s  {name} = {latest[name]:+.3f}', flush=True)
        next_t += 0.5
