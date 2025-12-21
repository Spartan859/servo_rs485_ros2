#!/usr/bin/env python3
import argparse
import sys
import threading
from typing import Optional
import os
import tempfile
import speech_recognition as sr
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from servo_rs485_ros2.srv import SetAngleWithSpeed
from std_msgs.msg import String
from flask import Flask, request, jsonify
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType



try:
    from chat_api import DoubaoChatBot
except ImportError:
    print("Warning: Could not import DoubaoChatBot. Voice features will be disabled.")
    DoubaoChatBot = object


app = Flask(__name__)
node_global: 'Optional[ServoTurnCLI]' = None
chatbot_global = None  # 全局变量存储 chatbot 实例

DEBUG = True

def dbg(msg: str):
    if DEBUG:
        print(f"[speech_turn_cli][{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}", flush=True)


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
            self.get_logger().info(f"安全状态更新: {msg.data}, 当前自动跟随模式: {'启用' if self.auto_follow_mode else '禁用'}")
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
        # self.auto_follow_mode = False
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
    try:
        with open(os.path.join(os.path.dirname(__file__), "index.html"), "r", encoding="utf-8") as f:
            return f.read()
    except Exception as e:
        dbg(f"Failed to load index.html: {e}")
        return "<h1>index.html not found</h1>"

@app.route('/move', methods=['POST'])
def move():
    data = request.get_json(silent=True) or {}
    action = data.get("action")
    dbg(f"/move called, action={action}")
    global node_global
    if not node_global:
        return jsonify(status="error", message="node not ready")
    if action == 'forward':
        node_global.set_forward()
    elif action == 'backward':
        node_global.set_backward()
    elif action == 'stop':
        node_global.set_stop()
        node_global.auto_follow_mode = False
    elif action == 'left':
        node_global.left()
    elif action == 'right':
        node_global.right()
    elif action == 'auto_follow':
        node_global.start_auto_follow()
    dbg(f"/move done, action={action}")
    return jsonify(status="ok")

@app.route('/set_safety_stop', methods=['POST'])
def set_safety_stop():
    data = request.get_json(silent=True) or {}
    enabled = data.get("enabled")
    dbg(f"/set_safety_stop called, enabled={enabled}")
    global node_global
    if not node_global:
        return jsonify(status="error", message="node not ready")
    enabled = request.json.get('enabled', True)
    node_global.set_safety_stop_enabled(enabled)
    dbg(f"/set_safety_stop done, enabled={enabled}")
    return jsonify(status="ok")


@app.route('/chat', methods=['POST'])
def chat():
    global chatbot_global
    if not chatbot_global:
        return jsonify(status="error", message="Chatbot not initialized")

    # 支持两种调用方式：
    # 1) 旧式上传文件（兼容性保留），使用 multipart/form-data 包含 'audio' 文件。
    # 2) 前端触发本地录音：JSON { action: 'record', duration: N }
    if 'audio' in request.files:
        audio_file = request.files['audio']
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            audio_file.save(tmp.name)
            tmp_path = tmp.name
        try:
            text = chatbot_global.recognize_audio_file(tmp_path)
            if not text:
                return jsonify(status="error", message="Recognition failed")
            reply_text, action = chatbot_global.run_gpt(text)
            if action:
                chatbot_global._handle_action(action)
            chatbot_global._text_to_voice(reply_text)
            return jsonify(status="ok", reply=reply_text, action=action)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    # 处理本地录音触发（非文件上传）
    data = request.get_json(silent=True) or {}
    if data.get('action') == 'record':
        duration = int(data.get('duration', 5))

        def _bg_run():
            try:
                # DoubaoChatBot.run() 包含录音->识别->LLM->动作->TTS 的完整流程（一次循环）。
                chatbot_global.run()
            except Exception as e:
                try:
                    print(f"Background chat run error: {e}")
                except Exception:
                    pass

        threading.Thread(target=_bg_run, daemon=True).start()
        return jsonify(status="ok", message=f"Triggered local recording (duration={duration}s)")

    return jsonify(status="error", message="No audio provided and no record action specified")


class WebDoubaoChatBot(DoubaoChatBot):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node_ref = node  # 持有 ROS2 节点的引用

    def recognize_audio_file(self, audio_path):
        try:
            import os
            size = os.path.getsize(audio_path) if os.path.exists(audio_path) else -1
            dbg(f"recognize_audio_file path={audio_path}, size={size}, asr_type={getattr(self, 'asr_type', None)}")
        except Exception:
            pass

        transcript = ""
        try:
            if self.asr_type == "whisper":
                with sr.AudioFile(audio_path) as source:
                    audio_data = self.voice_recognizer.record(source)
                transcript = self.voice_recognizer.recognize_whisper(
                    audio_data, self.whisper_model_type
                )
            elif self.asr_type == "funasr":
                transcript = self.funasr_client.recognize(audio_path)
            
            dbg(f"You said: {transcript!r}")
            return transcript
        except Exception as e:
            print(f"Recognition error: {e}")
            return None

    def _handle_action(self, action):
        dbg(f"_handle_action action={action!r}")
        """重写动作执行逻辑，调用 ROS2 节点方法"""
        if not self.node_ref:
            return
        print(f"Executing action: {action}")
        if action == 'forward':
            self.node_ref.set_forward()
        elif action in ['back', 'backward']:
            self.node_ref.set_backward()
        elif action == 'stop':
            self.node_ref.set_stop()
            self.node_ref.auto_follow_mode = False
        elif action == 'left':
            self.node_ref.left()
        elif action == 'right':
            self.node_ref.right()
        elif action == 'follow':
            self.node_ref.start_auto_follow()
        dbg(f"_handle_action done action={action!r}")

    def run(self):
        """Run one interaction cycle but respect ROS node safety state.

        This wraps the parent `run()` to check `node_ref.safety_status` before
        starting a new voice interaction and to capture/log exceptions.
        """
        super().run()
        # try:
        #     # 如果有 node 引用，优先检查安全状态
        #     if getattr(self, 'node_ref', None) is not None:
        #         try:
        #             status = getattr(self.node_ref, 'safety_status', 'OK')
        #         except Exception:
        #             status = 'OK'
        #         if status != 'OK':
        #             dbg(f"Skipping voice interaction due to safety_status={status}")
        #             return

        #     # 调用父类的 run() 完成录音->识别->LLM->动作->TTS 的流程
        #     super().run()
        # except Exception as e:
        #     dbg(f"WebDoubaoChatBot.run error: {e}")
        #     try:
        #         traceback.print_exc()
        #     except Exception:
        #         pass


def main(argv: Optional[list] = None):
    parser = argparse.ArgumentParser(description='Servo turn CLI using SetAngleWithSpeed (preemptive via target updates).')
    parser.add_argument('--service', default='/set_angle_with_speed', help='Service name for SetAngleWithSpeed')
    parser.add_argument('--step', type=float, default=10.0, help='Step degrees per left/right command')
    parser.add_argument('--speed', type=float, default=2.0, help='Turning speed in deg/s')
    parser.add_argument('--interval', type=int, default=1000, help='Step interval ms (granularity of smooth move)')
    parser.add_argument('--init-target', type=float, default=8.0, help='Initial absolute target angle in degrees')
    parser.add_argument('--min-deg', type=float, default=-30, help='Minimum allowed degree')
    parser.add_argument('--max-deg', type=float, default=30, help='Maximum allowed degree')
    parser.add_argument('--march-speed', type=float, default=0.3, help='March speed for forward/backward')
    parser.add_argument('--once', choices=['left', 'right', 'center'], help='Send a single command then exit')

    # 添加 ChatBot 相关参数
    parser.add_argument("--api-key", type=str, default=os.environ.get("ARK_API_KEY"), help="Doubao API Key")
    parser.add_argument("--base-url", type=str, default="https://ark.cn-beijing.volces.com/api/v3", help="Doubao Base URL")
    parser.add_argument("-m", "--model-name", type=str, default="doubao-seed-1-6-flash-250828", help="Doubao Model Name")
    
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

    # 初始化 ChatBot
    global chatbot_global
    if args.api_key:
        chatbot_global = WebDoubaoChatBot(
            node=node,
            api_key=args.api_key,
            base_url=args.base_url,
            model_name=args.model_name,
            asr_type="funasr",
            tts_type="qwen",
            mic_index=0
        )
    else:
        print("Warning: No API Key provided, voice chat disabled.")

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

dbg("System ready. Web controller started.")
dbg(f"api_key set: {bool(args.api_key)} model={args.model_name} base_url={args.base_url}")
dbg(f"servo service={args.service} step={args.step} speed={args.speed} interval={args.interval}")
