#!/usr/bin/env python3

import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Float32MultiArray, Int8MultiArray


class MotorSweepWithTModeNode(Node):
    """
    各モータ(1..9)の離散角度集合（初期角 + k*step_deg, k=0..K）を作り、
    9モータの直積（全組み合わせ）を順に司令します。

    各組み合わせベクトルを /motor_angles に司令し、全モータが到達（許容誤差内）したら
    Tモードを一定時間（既定:1.5秒）だけ有効化。Tモード中は /switch の値に従って
    switch==0 のモータを +3° ずつ加算して publish します。時間経過で自動終了し、
    次の組み合わせへ進みます。

    パラメータ:
    - step_deg: ステップ角度(既定 90.0)
    - max_plus_deg: 初期角からの最大加算角(既定 360.0)
    - include_initial: 初期角(オフセット0)を含めるか（既定 True）
    - reach_tolerance_deg: 目標到達判定許容(既定 2.0)
    - reach_all_timeout_sec: 組み合わせ到達待ちタイムアウト(既定 20.0)
    - t_mode_duration_sec: Tモード稼働時間(既定 1.5)
    - max_combinations: 上限（0 は無制限）。非常に大きい直積の暴走防止用（既定 0）
    """

    def __init__(self) -> None:
        super().__init__('motor_sweep_with_tmode_node')

        # Parameters
        self.declare_parameter('step_deg', 90.0)
        self.declare_parameter('max_plus_deg', 360.0)
        self.declare_parameter('include_initial', True)
        self.declare_parameter('reach_tolerance_deg', 2.0)
        self.declare_parameter('reach_all_timeout_sec', 20.0)
        self.declare_parameter('t_mode_duration_sec', 1.5)
        self.declare_parameter('max_combinations', 0)

        self.step_deg: float = float(self.get_parameter('step_deg').value)
        self.max_plus_deg: float = float(self.get_parameter('max_plus_deg').value)
        self.include_initial: bool = bool(self.get_parameter('include_initial').value)
        self.reach_tol: float = float(self.get_parameter('reach_tolerance_deg').value)
        self.reach_all_timeout_sec: float = float(self.get_parameter('reach_all_timeout_sec').value)
        self.t_mode_duration_sec: float = float(self.get_parameter('t_mode_duration_sec').value)
        self.max_combinations: int = int(self.get_parameter('max_combinations').value)

        # State
        self.current_angles: Optional[List[float]] = None
        self.initial_angles: Optional[List[float]] = None
        self.angles_lock = threading.Lock()

        # T-mode window
        self.t_mode_active = False
        self.t_mode_until: Optional[Time] = None

        # Subscriptions
        self.angle_sub = self.create_subscription(
            Float32MultiArray, '/motor_current_angles', self._angle_cb, 10
        )
        self.switch_sub = self.create_subscription(
            Int8MultiArray, '/switch', self._switch_cb, 10
        )

        # Publisher
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_angles', 10)

        # Sequencer thread
        self.seq_thread = threading.Thread(target=self._sequence_worker, daemon=True)
        self.seq_thread.start()

        self.get_logger().info(
            'Motor sweep node: step=%.1f, max+=%.1f, include0=%s, tol=%.1f, t-mode=%.2fs, maxComb=%d' % (
                self.step_deg, self.max_plus_deg, str(self.include_initial), self.reach_tol,
                self.t_mode_duration_sec, self.max_combinations
            )
        )

    # --------- Callbacks ---------
    def _angle_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 9:
            self.get_logger().warn(f'Invalid /motor_current_angles length: {len(msg.data)}')
            return
        with self.angles_lock:
            self.current_angles = list(msg.data)
            if self.initial_angles is None:
                self.initial_angles = list(msg.data)

    def _switch_cb(self, msg: Int8MultiArray) -> None:
        if not self.t_mode_active:
            return
        if len(msg.data) != 9:
            return

        now = self.get_clock().now()
        if self.t_mode_until is None or now > self.t_mode_until:
            # Time window ended
            self.t_mode_active = False
            return

        with self.angles_lock:
            base = self.current_angles.copy() if self.current_angles else [0.0] * 9
        updated = False
        for idx, val in enumerate(msg.data):
            if val == 0:
                base[idx] += 3.0
                updated = True
        if updated:
            out = Float32MultiArray()
            out.data = base
            self.motor_pub.publish(out)
            formatted = ', '.join(f'{a:.2f}' for a in base)
            self.get_logger().info(f'[T-mode] Published angles: [{formatted}]')

    # --------- Sequencer Logic ---------
    def _sequence_worker(self) -> None:
        # wait for initial angles
        self.get_logger().info('Waiting for initial /motor_current_angles...')
        while rclpy.ok() and self.initial_angles is None:
            time.sleep(0.05)
        if not rclpy.ok():
            return
        self.get_logger().info('Received initial angles. Building level sets...')

        with self.angles_lock:
            init = self.initial_angles.copy() if self.initial_angles else [0.0] * 9

        # Build per-motor level arrays
        levels: List[List[float]] = []
        steps = int(self.max_plus_deg // self.step_deg)
        for m in range(9):
            lv = []
            start_k = 0 if self.include_initial else 1
            for k in range(start_k, steps + 1):
                lv.append(init[m] + k * self.step_deg)
            if not lv:
                # 例えば include_initial=False かつ steps==0 の場合でも最低1つは用意
                lv.append(init[m])
            levels.append(lv)

        # Calculate total combinations (may be huge)
        total = 1
        for lv in levels:
            total *= len(lv)
        self.get_logger().info(f'Total combinations (theoretical): {total}')
        if self.max_combinations and total > self.max_combinations:
            self.get_logger().warn(f'Limiting combinations to max_combinations={self.max_combinations}')

        # Odometer-like iteration over combinations without holding all in memory
        idx = [0] * 9
        produced = 0

        def next_indices() -> bool:
            # advance odometer; return False when exhausted
            for i in range(9):
                idx[i] += 1
                if idx[i] < len(levels[i]):
                    return True
                idx[i] = 0
            return False

        # Start with current idx = zeros (first combination)
        more = True
        while rclpy.ok() and more:
            # Build target vector
            target_vec = [levels[m][idx[m]] for m in range(9)]
            self._command_all_absolute(target_vec)

            # Wait all motors reach
            if not self._wait_all_reach(target_vec, self.reach_all_timeout_sec):
                self.get_logger().warn('Timeout waiting all motors to reach target vector; continue')
            else:
                self.get_logger().info('Reached target vector')

            # T-mode window
            self._start_t_mode_window(self.t_mode_duration_sec)
            while rclpy.ok() and self.t_mode_active:
                time.sleep(0.02)

            produced += 1
            if self.max_combinations and produced >= self.max_combinations:
                break
            more = next_indices()

        self.get_logger().info(f'Sweep finished. Produced {produced} combinations.')

    def _command_motor_absolute(self, motor_index: int, target_deg: float) -> None:
        with self.angles_lock:
            base = self.current_angles.copy() if self.current_angles else [0.0] * 9
        base[motor_index] = target_deg
        out = Float32MultiArray()
        out.data = base
        self.motor_pub.publish(out)
        self.get_logger().info(f'Command: motor {motor_index+1} -> {target_deg:.2f} deg (absolute)')

    def _command_all_absolute(self, targets: List[float]) -> None:
        out = Float32MultiArray()
        out.data = targets
        self.motor_pub.publish(out)
        formatted = ', '.join(f'{a:.2f}' for a in targets)
        self.get_logger().info(f'Command vector -> [{formatted}]')

    def _wait_reach(self, motor_index: int, target_deg: float, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self.angles_lock:
                cur = (self.current_angles[motor_index]
                       if self.current_angles is not None else None)
            if cur is not None and abs(cur - target_deg) <= self.reach_tol:
                return True
            time.sleep(0.02)
        return False

    def _wait_all_reach(self, targets: List[float], timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self.angles_lock:
                cur = self.current_angles.copy() if self.current_angles else None
            if cur is not None:
                ok = True
                for i in range(9):
                    if abs(cur[i] - targets[i]) > self.reach_tol:
                        ok = False
                        break
                if ok:
                    return True
            time.sleep(0.02)
        return False

    def _start_t_mode_window(self, duration_sec: float) -> None:
        now = self.get_clock().now()
        self.t_mode_active = True
        self.t_mode_until = now + Duration(seconds=duration_sec)
        self.get_logger().info(f'Enter T-mode for {duration_sec:.2f}s')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorSweepWithTModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
