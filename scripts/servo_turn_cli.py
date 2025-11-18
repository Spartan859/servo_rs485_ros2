#!/usr/bin/env python3
import argparse
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from servo_rs485_ros2.srv import SetAngleWithSpeed


class ServoTurnCLI(Node):
    def __init__(self, service_name: str, step_deg: float, speed_dps: float, step_interval_ms: int,
                 init_target: float, min_deg: float, max_deg: float):
        super().__init__('servo_turn_cli')
        self._srv_name = service_name
        self._client = self.create_client(SetAngleWithSpeed, self._srv_name)
        self._step = float(step_deg)
        self._speed = float(speed_dps)
        self._interval = int(step_interval_ms)
        self._target = float(init_target)
        self._min_deg = float(min_deg)
        self._max_deg = float(max_deg)

        self.get_logger().info(f"Using service: {self._srv_name}")
        self.get_logger().info(f"Initial target: {self._target:.2f} deg  | step={self._step} deg  speed={self._speed} dps  interval={self._interval} ms")

    def _clamp(self, v: float) -> float:
        return max(self._min_deg, min(self._max_deg, v))

    def set_target(self, target_deg: float) -> bool:
        """Set absolute target angle in deg (preempting ongoing motion at the server)."""
        target_deg = self._clamp(target_deg)
        req = SetAngleWithSpeed.Request()
        req.degree = float(target_deg)
        req.speed_dps = self._speed
        req.step_interval_ms = self._interval

        if not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {self._srv_name} not available")
            return False

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error("Service call timeout")
            return False
        resp = future.result()
        if not resp or not resp.success:
            self.get_logger().error("Service returned failure")
            return False
        self._target = target_deg
        self.get_logger().info(f"New target set to {self._target:.2f} deg")
        return True

    def left(self) -> bool:
        # 左转=目标角度减少 step（基于目标而非当前位置）
        return self.set_target(self._target - self._step)

    def right(self) -> bool:
        # 右转=目标角度增加 step（基于目标而非当前位置）
        return self.set_target(self._target + self._step)


def main(argv: Optional[list] = None):
    parser = argparse.ArgumentParser(description='Servo turn CLI using SetAngleWithSpeed (preemptive via target updates).')
    parser.add_argument('--service', default='/set_angle_with_speed', help='Service name for SetAngleWithSpeed')
    parser.add_argument('--step', type=float, default=10.0, help='Step degrees per left/right command')
    parser.add_argument('--speed', type=float, default=2.0, help='Turning speed in deg/s')
    parser.add_argument('--interval', type=int, default=1000, help='Step interval ms (granularity of smooth move)')
    parser.add_argument('--init-target', type=float, default=0.0, help='Initial absolute target angle in degrees')
    parser.add_argument('--min-deg', type=float, default=-135.0, help='Minimum allowed degree')
    parser.add_argument('--max-deg', type=float, default=135.0, help='Maximum allowed degree')
    parser.add_argument('--once', choices=['left', 'right', 'center'], help='Send a single command then exit')

    args = parser.parse_args(argv)

    rclpy.init()
    node = ServoTurnCLI(
        service_name=args.service,
        step_deg=args.step,
        speed_dps=args.speed,
        step_interval_ms=args.interval,
        init_target=args.init_target,
        min_deg=args.min_deg,
        max_deg=args.max_deg,
    )

    try:
        if args.once:
            if args.once == 'left':
                node.left()
            elif args.once == 'right':
                node.right()
            elif args.once == 'center':
                node.set_target(0.0)
        else:
            print("\nControls: l=left  r=right  c=center  q=quit\n")
            while rclpy.ok():
                ch = input('> ').strip().lower()
                if ch == 'l':
                    node.left()
                elif ch == 'r':
                    node.right()
                elif ch == 'c':
                    node.set_target(0.0)
                elif ch == 'q':
                    break
                else:
                    print("Enter l/r/c/q")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
