#!/usr/bin/env python3
"""Small sim-only, opt-in fault injections with direct ROS queries, no daemon."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
from urllib.request import ProxyHandler, build_opener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=['lift-exit', 'drive-exit', 'lidar-adapter-exit', 'nav2-pause', 'nav2-resume'])
    parser.add_argument('--execute', action='store_true', help='Apply injection; otherwise show preflight only')
    args = parser.parse_args()
    prefix = 'EXECUTE' if args.execute else 'PREVIEW ONLY (no action)'
    with build_opener(ProxyHandler({})).open('http://127.0.0.1:8765/api/config', timeout=3) as response:
        config = json.load(response)
    if (config.get('stack_mode') != 'sim' or str(config.get('stack_domain_id')) != '20'
            or os.environ.get('ROS_DOMAIN_ID') != '20'):
        raise RuntimeError('Requires sim Dashboard and this terminal in ROS_DOMAIN_ID=20')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp':
        raise RuntimeError('Set RMW_IMPLEMENTATION=rmw_fastrtps_cpp to match the sim stack before injection')
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/robot_agent'))
    from robot_agent.recovery.primitives import same_domain_pids
    import rclpy
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from nav2_msgs.srv import ManageLifecycleNodes

    rclpy.init()
    node = rclpy.create_node('recovery_fault_injection_guard')
    observed = {}
    clock = []
    node.create_subscription(String, '/robot_mode', lambda m: observed.update(mode=(m.data, time.monotonic())), 10)
    node.create_subscription(Twist, '/cmd_vel', lambda m: observed.update(
        velocity=(max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z)), time.monotonic())), 10)
    def clock_cb(msg):
        clock.append((msg.clock.sec + msg.clock.nanosec / 1e9, time.monotonic()))
        del clock[:-30]
    node.create_subscription(Clock, '/clock', clock_cb, 10)
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        now = time.monotonic()
        if observed.get('mode', ('', 0))[0] != 'MANUAL' or now - observed['mode'][1] > 2:
            raise RuntimeError('Fresh MANUAL required; cancel Mission and select MANUAL first')
        if observed.get('velocity', (1, 0))[0] > .001 or now - observed['velocity'][1] > 1:
            raise RuntimeError('Fresh zero cmd_vel required')
        if len(clock) < 2 or clock[-1][0] <= clock[0][0] or now - clock[-1][1] > 1:
            raise RuntimeError('Fresh advancing sim clock required')
        patterns = {'lift-exit': 'robot_sim/sim_lift_command_node',
                    'drive-exit': 'robot_sim/sim_omni_drive_command_node',
                    'lidar-adapter-exit': 'robot_sim/pointcloud2_to_livox_custom_msg_node'}
        if args.case in patterns:
            pids = same_domain_pids(patterns[args.case])
            if len(pids) != 1:
                raise RuntimeError(f'Expected exactly one actual node process, found {pids}')
            print(f'{prefix}: {args.case}: SIGINT node PID {pids[0]}; MANUAL/zero/clock checked', flush=True)
            if args.execute:
                os.kill(pids[0], signal.SIGINT)
            return
        service = '/lifecycle_manager_navigation/manage_nodes'
        command = (ManageLifecycleNodes.Request.PAUSE if args.case == 'nav2-pause'
                   else ManageLifecycleNodes.Request.RESUME)
        print(f'{prefix}: {args.case}: {service}, command={command}; MANUAL/zero/clock checked', flush=True)
        if args.execute:
            client = node.create_client(ManageLifecycleNodes, service)
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Lifecycle manager service unavailable')
            future = client.call_async(ManageLifecycleNodes.Request(command=command))
            rclpy.spin_until_future_complete(node, future, timeout_sec=30)
            if not future.done():
                raise RuntimeError('Service timeout: outcome UNKNOWN; inspect before repeating')
            if not future.result().success:
                raise RuntimeError('Lifecycle manager returned success=false; inspect current states')
            print('Service accepted; verify fresh lifecycle evidence on Dashboard', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
