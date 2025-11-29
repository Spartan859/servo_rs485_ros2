#!/usr/bin/env python3
import argparse
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from servo_rs485_ros2.srv import SetAngleWithSpeed
from std_msgs.msg import String
from flask import Flask, request, jsonify
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType


app = Flask(__name__)
node_global: 'Optional[ServoTurnCLI]' = None


class ServoTurnCLI(Node):
    def __init__(self, service_name: str, step_deg: float, speed_dps: float, step_interval_ms: int,
                 init_target: float, min_deg: float, max_deg: float, march_speed: float):
        super().__init__('servo_turn_cli')
        self._srv_name = service_name
        self._client = self.create_client(SetAngleWithSpeed, self._srv_name)
        self._step = float(step_deg)
        self._speed = float(speed_dps)
        self._interval = int(step_interval_ms)
        self._target = float(init_target)
        self._min_deg = float(min_deg)
        self._max_deg = float(max_deg)
        self._march_speed = float(march_speed)

        self.get_logger().info(f"Using service: {self._srv_name}")
        self.get_logger().info(f"Initial target: {self._target:.2f} deg  | step={self._step} deg  speed={self._speed} dps  interval={self._interval} ms")

        # Subscribe to safety status
        self.safety_sub = self.create_subscription(String, 'safety/status', self._safety_callback, 10)
        self.safety_status = 'OK'
        self.last_safety_status = 'OK'
        self.enable_safety_stop = True  # Default to enabled
        self.auto_follow_mode = False  # Initialize auto follow mode

        # Parameter client for balance_controller
        self.param_client = self.create_client(SetParameters, '/balance_controller/set_parameters')

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

    def _safety_callback(self, msg):
        self.safety_status = msg.data
        if msg.data != self.last_safety_status:
            self.last_safety_status = msg.data
            if msg.data != 'OK':
                if self.enable_safety_stop or self.auto_follow_mode:
                    self.set_stop()
                self.get_logger().warn(f"检测到不安全状态: {msg.data}，{'已自动停止' if self.enable_safety_stop else '未停止（安全停止已禁用）'}")
            elif msg.data == 'OK' and self.auto_follow_mode:
                self.set_forward()
                self.get_logger().info("安全状态恢复，继续前进")

    def _set_march_velocity(self, velocity: float):
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Parameter service not available")
            return False
        req = SetParameters.Request()
        param = Parameter()
        param.name = 'march_velocity'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = velocity
        req.parameters = [param]
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().error("Set parameter timeout")
            return False
        resp = future.result()
        if resp.results[0].successful:
            self.get_logger().info(f"Set march_velocity to {velocity}")
            return True
        else:
            self.get_logger().error(f"Failed to set march_velocity: {resp.results[0].reason}")
            return False

    def set_forward(self):
        self._set_march_velocity(self._march_speed)

    def set_backward(self):
        self._set_march_velocity(-self._march_speed)

    def set_stop(self):
        self.auto_follow_mode = False
        self._set_march_velocity(0.0)

    def set_safety_stop_enabled(self, enabled: bool):
        self.enable_safety_stop = enabled
        self.get_logger().info(f"安全停止已{'启用' if enabled else '禁用'}")

    def start_auto_follow(self):
        if self.safety_status == 'OK':
            self.auto_follow_mode = True
            self.set_forward()
            self.get_logger().info("开始自动跟随模式")
        else:
            self.get_logger().warn("安全状态不佳，无法开始自动跟随")


@app.route('/')
def index():
    return '''
    <html>
    <head><title>Servo Controller</title></head>
    <body>
    <h1>Servo Controller</h1>
    <div>
        <input type="checkbox" id="safetyStop" checked onchange="setSafetyStop(this.checked)">
        <label for="safetyStop">启用安全停止</label>
    </div>
    <button onclick="send('forward')">前进</button>
    <button onclick="send('backward')">后退</button>
    <button onclick="send('stop')">停止</button>
    <button onclick="send('auto_follow')" style="background-color: green; color: white;">自动跟随</button>
    <button onclick="send('left')">左转</button>
    <button onclick="send('right')">右转</button>
    <script>
    function send(action) {
        fetch('/move', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({action: action})
        });
    }
    function setSafetyStop(enabled) {
        fetch('/set_safety_stop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({enabled: enabled})
        });
    }
    </script>
    </body>
    </html>
    '''

@app.route('/move', methods=['POST'])
def move():
    global node_global
    if not node_global:
        return jsonify(status="error", message="node not ready")
    action = request.json.get('action')
    if action == 'forward':
        node_global.set_forward()
    elif action == 'backward':
        node_global.set_backward()
    elif action == 'stop':
        node_global.set_stop()
    elif action == 'left':
        node_global.left()
    elif action == 'right':
        node_global.right()
    elif action == 'auto_follow':
        node_global.start_auto_follow()
    return jsonify(status="ok")

@app.route('/set_safety_stop', methods=['POST'])
def set_safety_stop():
    global node_global
    if not node_global:
        return jsonify(status="error", message="node not ready")
    enabled = request.json.get('enabled', True)
    node_global.set_safety_stop_enabled(enabled)
    return jsonify(status="ok")


def main(argv: Optional[list] = None):
    parser = argparse.ArgumentParser(description='Servo turn CLI using SetAngleWithSpeed (preemptive via target updates).')
    parser.add_argument('--service', default='/set_angle_with_speed', help='Service name for SetAngleWithSpeed')
    parser.add_argument('--step', type=float, default=10.0, help='Step degrees per left/right command')
    parser.add_argument('--speed', type=float, default=2.0, help='Turning speed in deg/s')
    parser.add_argument('--interval', type=int, default=1000, help='Step interval ms (granularity of smooth move)')
    parser.add_argument('--init-target', type=float, default=8.0, help='Initial absolute target angle in degrees')
    parser.add_argument('--min-deg', type=float, default=-30, help='Minimum allowed degree')
    parser.add_argument('--max-deg', type=float, default=30, help='Maximum allowed degree')
    parser.add_argument('--march-speed', type=float, default=5.0, help='March speed for forward/backward')
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
        march_speed=args.march_speed,
    )
    
    global node_global
    node_global = node
    
    node.set_target(args.init_target)

    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000), daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
