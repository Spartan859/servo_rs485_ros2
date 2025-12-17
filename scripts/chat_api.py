import argparse
import os
# [新增] 解决 OpenBLAS Warning: Detect OpenMP Loop 问题
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import subprocess
import tempfile
import re  # 引入正则模块用于解析
import urllib.request # 用于下载音频
import time # [新增] 引入 time 模块
import dashscope # 引入阿里云 DashScope SDK
from dashscope.audio.asr import Recognition, RecognitionCallback, RecognitionResult # 引入 FunASR 相关类

# 移除 gpt4all，引入 openai
from openai import OpenAI
# from playsound import playsound # 移除 playsound，改用 ffplay
import speech_recognition as sr
#from TTS.api import TTS

# --- FunASR 相关类定义 ---
class FunASRCallback(RecognitionCallback):
    def __init__(self):
        super().__init__()
        self.final_text = ""
        self.is_finished = False

    def on_open(self) -> None:
        # print('FunASR: Connection open.')
        pass

    def on_close(self) -> None:
        # print('FunASR: Connection close.')
        self.is_finished = True

    def on_complete(self) -> None:
        # print('FunASR: Recognition completed.')
        self.is_finished = True

    def on_error(self, message) -> None:
        print('FunASR Error:', message.message)
        self.is_finished = True

    def on_event(self, result: RecognitionResult) -> None:
        sentence = result.get_sentence()
        if 'text' in sentence:
            # print('FunASR Partial:', sentence['text'])
            if RecognitionResult.is_sentence_end(sentence):
                self.final_text += sentence['text']
                # print('FunASR Sentence End:', sentence['text'])

class FunASRClient:
    def __init__(self):
        if not os.environ.get("DASHSCOPE_API_KEY"):
            print("Warning: DASHSCOPE_API_KEY not found for FunASR.")
        
        # 设置 API URL (北京节点)
        dashscope.base_websocket_api_url = 'wss://dashscope.aliyuncs.com/api-ws/v1/inference'

    def recognize(self, audio_file_path):
        """
        使用 FunASR 识别本地音频文件
        注意：虽然 FunASR 是实时的，但为了兼容现有的“录音-识别”流程，
        我们这里模拟实时发送数据。
        """
        callback = FunASRCallback()
        
        # 初始化识别服务
        recognition = Recognition(
            model='fun-asr-realtime',
            format='pcm', 
            sample_rate=16000, # FunASR 通常需要 16k
            callback=callback
        )
        
        recognition.start()
        
        # 读取音频文件并发送
        # 注意：我们需要将录制的 44.1k/双声道 音频转换为 16k/单声道
        # 这里使用 ffmpeg 转换流
        cmd = [
            "ffmpeg", 
            "-i", audio_file_path,
            "-f", "s16le",       # 输出 PCM
            "-ac", "1",          # 单声道
            "-ar", "16000",      # 16000Hz
            "-loglevel", "quiet",
            "pipe:1"             # 输出到 stdout
        ]
        
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            
            while True:
                data = process.stdout.read(3200) # 每次读取一小块
                if not data:
                    break
                recognition.send_audio_frame(data)
                
            process.wait()
            
        except Exception as e:
            print(f"FunASR Audio Processing Error: {e}")
            
        recognition.stop()
        
        # 等待回调完成（简单处理）
        # import time # [移除] 已经在顶部导入
        timeout = 5
        start_time = time.time()
        while not callback.is_finished and (time.time() - start_time < timeout):
            time.sleep(0.1)
            
        return callback.final_text

# ------------------------

class QwenTTS:
    def __init__(self, model="qwen3-tts-flash", voice="Dylan"):
        self.model = model
        self.voice = voice
        # 确保 API Key 存在
        if not os.environ.get("DASHSCOPE_API_KEY"):
            print("Warning: DASHSCOPE_API_KEY not found in environment variables.")

    def process(self, text, audio_save_path):
        try:
            # 使用 MultiModalConversation 接口
            response = dashscope.MultiModalConversation.call(
                model=self.model,
                api_key=os.environ.get("DASHSCOPE_API_KEY"),
                text=text,
                voice=self.voice,
                language_type="Chinese"
            )
            
            if response.status_code == 200:
                # 检查 response 结构
                if hasattr(response, 'output') and response.output.audio and response.output.audio.url:
                    audio_url = response.output.audio.url
                    # 下载音频文件
                    urllib.request.urlretrieve(audio_url, audio_save_path)
                else:
                    print(f"QwenTTS Error: Invalid response structure: {response}")
            else:
                print(f"QwenTTS Error: {response.message}")
                
        except Exception as e:
            print(f"QwenTTS Exception: {e}")


class DoubaoChatBot:
    """Smart Bicycle Voice Assistant based on Whisper/FunASR and Doubao API"""

    def __init__(self, api_key, base_url, model_name, whisper_model_type="base", tts_type="qwen", tts_rate=165, mic_index=None, asr_type="funasr"):
        print(f"==> Doubao Model: {model_name}, ASR: {asr_type}, TTS: {tts_type}")
        
        # 初始化 OpenAI 客户端 (用于豆包 API)
        self.client = OpenAI(
            api_key=api_key,
            base_url=base_url,
        )
        self.model_name = model_name

        self.asr_type = asr_type
        self.whisper_model_type = whisper_model_type

        # 初始化 ASR 引擎
        if self.asr_type == "whisper":
            self.voice_recognizer = sr.Recognizer()
        elif self.asr_type == "funasr":
            self.funasr_client = FunASRClient()
            self.voice_recognizer = sr.Recognizer() # 仅用于读取文件辅助

        # 保存 mic_index 供 ffmpeg 使用
        self.mic_index = mic_index if mic_index is not None else 0

        if tts_type == "qwen":
            self.tts_engine = QwenTTS()

    def run(self):
        """Run the listen-think-response loop"""
        cycle_start = time.time() # [新增] 记录循环开始时间

        input_words = self._voice_to_text()
        if input_words:
            # 获取结构化回复 (文本 + 动作)
            reply_text, action = self.run_gpt(input_words)
            
            # 执行动作
            if action:
                self._handle_action(action)
            
            # 播放语音回复
            self._text_to_voice(reply_text)

            cycle_end = time.time() # [新增] 记录循环结束时间
            print(f"--- [Timing] Total Interaction Time: {cycle_end - cycle_start:.2f}s ---\n")

    def _voice_to_text(self):
        """Listen voice using ffmpeg and convert to text"""
        print("Listening...")
        
        # 创建临时文件保存录音
        tmp_wav = tempfile.NamedTemporaryFile(suffix=".wav", delete=False).name
        
        # 构造设备名称，main.c 使用的是 plughw，这里保持一致
        device_name = f"plughw:{self.mic_index}"
        
        # 构造 ffmpeg 命令 (录制 5 秒)
        cmd = [
            "ffmpeg",
            "-y",
            "-f", "alsa",
            "-i", device_name,
            "-t", "5", 
            "-ar", "44100",
            "-ac", "2",
            "-loglevel", "error", # 减少日志输出
            tmp_wav
        ]
        
        try:
            print(f"Recording from {device_name} via ffmpeg (5s)...")
            t_rec_start = time.time() # [新增]
            # 调用系统 ffmpeg 进行录音
            subprocess.run(cmd, check=True)
            t_rec_end = time.time() # [新增]
            print(f"[Timing] Recording: {t_rec_end - t_rec_start:.2f}s") # [新增]
            
            if os.path.exists(tmp_wav) and os.path.getsize(tmp_wav) > 0:
                print("Recognizing...")
                t_asr_start = time.time() # [新增]
                transcript = ""
                
                if self.asr_type == "whisper":
                    # 使用 Whisper
                    with sr.AudioFile(tmp_wav) as source:
                        audio_data = self.voice_recognizer.record(source)
                    transcript = self.voice_recognizer.recognize_whisper(
                        audio_data, self.whisper_model_type
                    )
                elif self.asr_type == "funasr":
                    # 使用 FunASR
                    transcript = self.funasr_client.recognize(tmp_wav)
                
                t_asr_end = time.time() # [新增]
                print(f"[Timing] ASR Recognition: {t_asr_end - t_asr_start:.2f}s") # [新增]

                print(f"You said: {transcript}")
                return transcript
            else:
                print("Recording failed: File is empty.")
                return None
                
        except subprocess.CalledProcessError as e:
            print(f"FFmpeg recording error: {e}")
            print("Please check if 'ffmpeg' is installed and the device index is correct.")
            return None
        except Exception as e:
            print(f"Recognition error: {e}")
            return None
        finally:
            # 清理临时文件
            if os.path.exists(tmp_wav):
                os.remove(tmp_wav)

    def run_gpt(self, question):
        """Run Doubao API model and parse structured output"""
        print("Thinking (Doubao API)...")
        t_llm_start = time.time() # [新增]
        
        # 定义系统提示词，强制规定输出格式
        system_prompt = """
# Role
You are a smart bicycle assistant. You control the bike and talk to the rider.

# Task Instructions
1. **Bicycle Control**: If the user asks to control the bike (move, turn, stop, follow), output the corresponding action command. DO NOT use web search for these requests.
2. **Knowledge/Weather Query**: If the user asks about general knowledge, weather, news, or location info, use the `web_search` tool to get real-time information.
3. **Language**: Always reply in the same language as the user's input (Chinese or English).

# Output Format
You must output your response in a strict format with two parts:
1. The verbal reply to the user.
2. The action command wrapped in <action> tags.
# Available actions:
- forward (Move forward)
- back (Move backward)
- left (Turn left)
- right (Turn right)
- follow (Follow mode)
- avoid (Obstacle avoidance mode)
- stop (Stop the bike)
- none (No action needed)

Example User: "Go forward please."
Example Output: OK, moving forward now. <action>forward</action>

Example User: "北京今天天气怎么样？" (Use web_search)
Example Output: 北京今天天气晴朗，气温 25 度，适合骑行

Example User: "Hello."
Example Output: Hello! Ready to ride. <action>none</action>

Keep your verbal reply short and concise.
"""
        tools = [{
            "type": "web_search",
            "max_keyword": 3,
            "sources": ["douyin", "moji"],
            "user_location": {  # 用户地理位置（用于优化搜索结果）
                "type": "approximate",  # 大致位置
                "country": "中国",
                "region": "北京",
                "city": "北京"
            },
            "limit": 3
        }]
        try:
            # 调用豆包 API (Response API)
            response = self.client.responses.create(
                model=self.model_name,
                input=[
                    {
                        "role": "system",
                        "content": [{"type": "input_text", "text": system_prompt}]
                    },
                    {
                        "role": "user",
                        "content": [{"type": "input_text", "text": question}]
                    }
                ],
                tools=tools
            )
            t_llm_end = time.time() # [新增]
            print(f"[Timing] LLM Thinking: {t_llm_end - t_llm_start:.2f}s") # [新增]

            # print(response)
            
            # 修复解析逻辑：Response API 的结构不同于 ChatCompletion
            raw_answer = ""
            if hasattr(response, 'output'):
                for item in response.output:
                    if item.type == 'message':
                        for content in item.content:
                            if content.type == 'output_text':
                                raw_answer += content.text
            
            print(f"==> Raw LLM Output: {raw_answer}")

            # 解析输出
            reply_text = raw_answer
            action = "none"

            # 使用正则提取 <action>...</action>
            match = re.search(r'<action>(.*?)</action>', raw_answer, re.IGNORECASE)
            if match:
                action = match.group(1).strip().lower()
                # 从回复文本中移除 action 标签部分，只保留语音内容
                reply_text = raw_answer.replace(match.group(0), "").strip()
            
            print(f"==> Parsed Reply: {reply_text}")
            print(f"==> Parsed Action: {action}")
            
            return reply_text, action

        except Exception as e:
            print(f"API Error: {e}")
            return "Sorry, I encountered an error.", "none"

    def _handle_action(self, action):
        """Execute the bicycle control action"""
        if action == "none":
            return
            
        print(f"\n>>> 🚲 BICYCLE ACTION EXECUTED: [{action.upper()}] <<<\n")
        # 在这里添加实际的硬件控制代码
        # 例如: serial.write(action.encode())

    def _text_to_voice(self, answer):
        """Convert text to voice using TTS tools"""
        if not answer:
            return

        tmp_file = tempfile.NamedTemporaryFile(
            prefix="talkgpt4all-", suffix=".wav", delete=False
        )
        # 立即关闭文件句柄
        tmp_file.close()
        
        try:
            t_tts_gen_start = time.time() # [新增]
            try:
                self.tts_engine.process(answer, tmp_file.name)
            except RuntimeError as e:
                print(f"TTS Error: {e}")
                return
            t_tts_gen_end = time.time() # [新增]
            print(f"[Timing] TTS Generation: {t_tts_gen_end - t_tts_gen_start:.2f}s") # [新增]

            # [修复 2] 替换 playsound，直接调用系统验证过的 ffplay
            # -autoexit: 播放完自动退出
            # -nodisp: 不显示图形窗口
            # -ss 0: 从头播放
            print(f"Playing audio using ffplay: {tmp_file.name}")
            t_play_start = time.time() # [新增]
            subprocess.run(
                ["ffplay", "-autoexit", "-nodisp", "-hide_banner", "-loglevel", "error", tmp_file.name], 
                check=False
            )
            t_play_end = time.time() # [新增]
            print(f"[Timing] Audio Playback: {t_play_end - t_play_start:.2f}s") # [新增]
            
        finally:
            if os.path.exists(tmp_file.name):
                os.remove(tmp_file.name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    # 豆包 API 相关参数
    parser.add_argument(
        "--api-key",
        type=str,
        default=os.environ.get("ARK_API_KEY"),
        help="Doubao API Key (or set ARK_API_KEY env var)",
    )
    parser.add_argument(
        "--base-url",
        type=str,
        default="https://ark.cn-beijing.volces.com/api/v3",
        help="Doubao Base URL",
    )
    parser.add_argument(
        "-m",
        "--model-name",
        type=str,
        default="doubao-seed-1-6-flash-250828", # 你的接入点 ID
        help="Doubao Endpoint ID / Model Name",
    )
    
    # ASR 相关参数
    parser.add_argument(
        "--asr-type",
        type=str,
        default="whisper",
        choices=["whisper", "funasr"],
        help="ASR engine type: whisper (local) or funasr (cloud)",
    )
    parser.add_argument(
        "-w",
        "--whisper-model-type",
        type=str,
        default="base",
        help="whisper model type, default is base",
    )
    
    # TTS 相关参数
    parser.add_argument(
        "--tts-type",
        type=str,
        default="qwen",
        choices=["qwen"],
        help="TTS engine type: glow (local) or qwen (cloud)",
    )
    parser.add_argument(
        "--voice-rate",
        type=int,
        default=165,
        help="voice rate, default is 165, the larger the speak faster",
    )
    
    # 1. 添加麦克风索引参数
    parser.add_argument(
        "--mic-index",
        type=int,
        default=0, # 默认为 None (使用系统默认)
        help="Microphone device index (use search_device.py to find)",
    )
    
    args = parser.parse_args()

    if not args.api_key:
        print("Error: API Key is required. Please provide --api-key or set ARK_API_KEY environment variable.")
        exit(1)

    chat_bot = DoubaoChatBot(
        api_key=args.api_key,
        base_url=args.base_url,
        model_name=args.model_name,
        whisper_model_type=args.whisper_model_type,
        tts_type=args.tts_type,
        tts_rate=args.voice_rate,
        mic_index=args.mic_index,
        asr_type=args.asr_type # 传入 ASR 类型
    )
    
    print("System ready. Please speak.")
    while True:
        chat_bot.run()
