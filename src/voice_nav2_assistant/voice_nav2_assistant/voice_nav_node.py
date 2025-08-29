# -*- coding: utf-8 -*-
# 檔案：voice_nav2_assistant/voice_nav_node.py
# 功能：讀 WAV 檔 → Whisper(語音辨識) →（LLM/規則：理解去哪裡）→ 查 places.yaml → Nav2 導航

import os, math, json, glob, yaml                # 作業系統/數學/JSON/萬用字元找檔/讀 YAML
from typing import Dict
from dataclasses import dataclass               # 讓我們用簡單的資料結構存座標

import rclpy                                    # ROS 2 Python 客戶端
from rclpy.node import Node
from rclpy.action import ActionClient           # 送 Nav2 的 action 用

from nav2_msgs.action import NavigateToPose     # Nav2 的目標訊息格式
from geometry_msgs.msg import PoseStamped, Quaternion

from faster_whisper import WhisperModel         # 本地 Whisper（把聲音→文字）

import subprocess, tempfile

# ====（可選）OpenAI：只有你要用 GPT 才會載====
OPENAI_OK = False
try:
    from openai import OpenAI                   # 官方 SDK
    OPENAI_OK = True if os.getenv("OPENAI_API_KEY") else False
except Exception:
    OPENAI_OK = False


def yaw_to_quat(yaw: float) -> Quaternion:
    """把平面上的朝向角 (yaw, 弧度) 轉成四元數(ROS 需要的格式)"""
    q = Quaternion()
    half = yaw / 2.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    q.x = 0.0
    q.y = 0.0
    return q


@dataclass
class Place:
    """存一個地點：x, y, yaw（從 places.yaml 來）"""
    x: float
    y: float
    yaw: float


class VoiceNavNode(Node):
    """整個語音導航節點：啟動時就會把所有音檔跑一遍"""

    def __init__(self):
        super().__init__("voice_nav_node")

        # ---- 宣告 ROS 參數（可以從 launch 或命令列帶進來）----
        self.declare_parameter("places_file", "")            # places.yaml 路徑
        self.declare_parameter("audio_mode", "file")         # file | mic（現在用 file）
        self.declare_parameter("audio_files", "")            # 語音檔清單（逗號或萬用字元）
        self.declare_parameter("asr_model_size", "small")    # Whisper 尺寸：tiny/base/small/medium
        self.declare_parameter("use_openai", False)          # 要不要用 GPT
        self.declare_parameter("openai_model", "gpt-4o-mini")

        # ---- 讀取參數值 ----
        self.places_file = self.get_parameter("places_file").get_parameter_value().string_value
        self.audio_mode  = self.get_parameter("audio_mode").get_parameter_value().string_value
        self.audio_files_pat = self.get_parameter("audio_files").get_parameter_value().string_value
        self.asr_model_size = self.get_parameter("asr_model_size").get_parameter_value().string_value
        self.use_openai = self.get_parameter("use_openai").get_parameter_value().bool_value
        self.openai_model = self.get_parameter("openai_model").get_parameter_value().string_value


        self.get_logger().info(
        f"Params: places_file={self.places_file}, audio_mode={self.audio_mode}, "
        f"asr_model={self.asr_model_size}, use_openai={self.use_openai}, model={self.openai_model}"
)

        # ---- 讀 places.yaml 變成字典 ----
        #    例如 snack_bin: {x: 3.12, y: -1.45, yaw: 1.57}
        self.places: Dict[str, Place] = {}
        with open(self.places_file, "r") as f:
            raw = yaml.safe_load(f)
        for k, v in raw.items():
            self.places[k] = Place(float(v["x"]), float(v["y"]), float(v["yaw"]))
        self.get_logger().info(f"Loaded places: {list(self.places.keys())}")

        for name, p in self.places.items():
            self.get_logger().info(f"Place loaded: {name} -> x={p.x:.2f}, y={p.y:.2f}, yaw={p.yaw:.3f}rad")

        # ---- 準備 ASR（語音→文字）----
        self.get_logger().info(f"Loading faster-whisper model: {self.asr_model_size}")
        # device="auto"：CUDA 有就用 GPU；compute_type="int8_float16"：在 Jetson 上省記憶體
        # self.whisper = WhisperModel(self.asr_model_size, device="auto", compute_type="int8_float16")
        self.whisper = WhisperModel(self.asr_model_size, device="cpu", compute_type="int8")


        # ---- （選用）準備 LLM ----
        self.llm = None
        if self.use_openai:
            if not OPENAI_OK:
                self.get_logger().error("use_openai=True 但 OPENAI_API_KEY 未設定。")
            else:
                try:
                    self.llm = OpenAI()  # 之後呼叫 chat.completions 用
                    self.get_logger().info(f"OpenAI model: {self.openai_model}")
                except Exception as e:
                    self.get_logger().error(f"OpenAI 初始化失敗：{e}")

        # ---- Nav2 Action Client（把目標 Pose 丟給 Nav2）----
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 action server…")
        self.nav_client.wait_for_server()                 # 等 Nav2 準備好
        self.get_logger().info("Nav2 action server ready.")

        # ---- 現在是「檔案模式」，啟動後就把音檔一次處理完 ----
        if self.audio_mode == "file":
            self.get_logger().info("🗂️ 使用檔案模式 (audio_mode=file)")
            self.process_audio_files()
        else:
            self.loop_mic()

    # ============= 讀入所有語音檔 → 逐一處理 =============
    def process_audio_files(self):
        self.get_logger().info(f"Audio patterns: {self.audio_files_pat}")
        paths = []
        if not self.audio_files_pat:
            self.get_logger().error("audio_files 參數未指定。")
            return

        # 把字串拆成多個 pattern（逗號分隔；可用萬用字元 *.wav）
        for token in self.audio_files_pat.split(","):
            token = token.strip()
            if not token:
                continue
            # glob 會把 "~/voice_tests/*.wav" 展開成真正的檔案路徑列表
            paths.extend(sorted(glob.glob(os.path.expanduser(token))))

        self.get_logger().info(f"Found {len(paths)} audio files")
        if not paths:
            self.get_logger().error(f"找不到音檔：{self.audio_files_pat}")
            return

        # 逐個檔案：ASR→文字→理解去哪→查座標→請 Nav2 走
        for p in paths:
            self.get_logger().info(f"[ASR] 轉錄音檔：{p}")
            text = self.transcribe_file(p)                # Whisper 轉成文字
            self.get_logger().info(f"[ASR] 文字：{text}")
            if not text:
                continue

            place_id = self.infer_place(text)             # 讀懂「要去哪」
            if not place_id:
                self.get_logger().warn("解析不到目的地，略過此檔。")
                continue
            if place_id not in self.places:
                self.get_logger().warn(f"未知地點：{place_id}，請加入 places.yaml")
                continue

            self.navigate_to(place_id)                    # 真的送 Nav2

        self.get_logger().info("所有音檔處理完成。")

    # ============= Whisper 把語音檔轉成文字 =============
    def transcribe_file(self, wav_path: str) -> str:
        try:
            # language="zh" 告訴它用中文模型解；beam_size=1 速度快一點
            segments, info = self.whisper.transcribe(
                wav_path, language="zh", vad_filter=False, beam_size=1
            )
            # Whisper 會產生多段字串，這裡把它們接起來
            out = "".join([seg.text for seg in segments]).strip()
            return out
        except Exception as e:
            self.get_logger().error(f"Whisper 轉錄失敗：{e}")
            return ""

    # ============= 讀懂你要去哪（意圖解析）=============
    def infer_place(self, text: str) -> str:
        self.get_logger().info(f"[NLU] input: {text}")
        # 先嘗試用 GPT（如果你有開 use_openai 且有金鑰）
        if self.use_openai and self.llm is not None:
            try:
                places = list(self.places.keys())
                # 設計一個「只回傳 JSON」的提示詞，讓 GPT 回 {"place_id":"xxx"}
                prompt = (
                    "你是機器人任務解析器。輸入是一句中文口語，"
                    "請只回傳 JSON，欄位只有 place_id。"
                    f"可選地點：{places}\n"
                    "若能對應，輸出如 {\"place_id\":\"snack_bin\"}；"
                    "若無法對應，輸出 {\"place_id\":\"\"}。\n"
                    f"指令：「{text}」"
                )
                resp = self.llm.chat.completions.create(
                    model=self.openai_model,
                    messages=[{"role": "user", "content": prompt}],
                    temperature=0.0,                    # 不要發揮創意，照準確配對
                )
                content = resp.choices[0].message.content
                self.get_logger().info(f"[NLU/GPT] raw: {content}")
                data = json.loads(content)               # 解析 JSON 字串
                return data.get("place_id", "")
            except Exception as e:
                # GPT 壞了就退回離線規則
                self.get_logger().warn(f"OpenAI 解析失敗：{e}，改用離線規則。")

        # ---- 離線規則：不靠網路、看關鍵字 ----
        kw = {
            "零食": "snack_bin",
            "零食櫃": "snack_bin",
            "餅乾": "snack_bin",
            "飲料": "snack_bin",
            "廚房": "kitchen_table",
            "桌子": "kitchen_table",
            "會議室": "meeting_room",
        }
        # 有對到關鍵字就回對應地點
        for k, pid in kw.items():
            if k in text:
                return pid

        # 你直接說地點代號（例如 "snack_bin"）也接受
        for pid in self.places.keys():
            if pid in text:
                return pid

        # 一些常見句型補強
        if "吃" in text and ("零食" in text or "餅乾" in text or "飲料" in text):
            return "snack_bin"
        if ("去" in text or "到" in text) and "桌" in text:
            return "kitchen_table"
        return ""                                     # 真的聽不懂就回空字串

    # ============= 送導航目標給 Nav2 =============
    def navigate_to(self, place_id: str):
        self.get_logger().info("Sending goal to Nav2…")
        p = self.places[place_id]                    # 取出 (x,y,yaw)
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"                 # 以地圖座標為基準
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quat(p.yaw)   # 朝向轉成四元數
        goal.pose = pose

        self.get_logger().info(
            f"導航 → {place_id}: ({p.x:.2f},{p.y:.2f}, yaw={p.yaw:.2f})"
        )

        # 非同步送出目標，並等待結果
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh and gh.accepted:
            self.get_logger().info("Nav2 已接受目標。")
        if not gh or not gh.accepted:
            self.get_logger().error("目標被 Nav2 拒絕。")
            return

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        status = res_future.result().status
        if status == 4:
            self.get_logger().info("✅ 已到達目標。")      # Nav2 回報成功的代號是 4
        else:
            self.get_logger().warn(f"⚠️ 導航未成功，狀態={status}")
    def loop_mic(self):
        """按 Enter 錄 3 秒 → Whisper → 解析 → Nav2。q + Enter 離開。"""
        dur = int(os.environ.get('MIC_DURATION', '3'))   # 錄音秒數
        rate = int(os.environ.get('MIC_RATE', '16000'))  # 取樣率
        self.get_logger().info("🎤 麥克風模式：按 Enter 開始錄音 3 秒，輸入 q + Enter 離開")
        while rclpy.ok():
            try:
                line = input()
            except EOFError:
                break
            if line.strip().lower() == 'q':
                break
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav = tmp.name
            # 錄單聲道 16k wav（Jetson 上最穩）
            cmd = ['arecord','-q','-d',str(dur),'-f','S16_LE','-r',str(rate),'-c','1', wav]
            subprocess.run(cmd, check=False)
            text = self.transcribe_file(wav)
            self.get_logger().info(f"[ASR/MIC] 文字：{text}")
            if not text:
                continue
            place_id = self.infer_place(text)
            if place_id and place_id in self.places:
                self.navigate_to(place_id)
            else:
                self.get_logger().warn("聽不懂或未知地點，請再說一次。")


def main():
    rclpy.init()               # 啟動 ROS
    node = VoiceNavNode()      # 建立並執行上面流程
    rclpy.try_shutdown()       # 做完收攤


if __name__ == "__main__":
    main()

