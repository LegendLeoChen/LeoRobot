import pyaudio
import wave
import websocket
import datetime
import hashlib
import base64
import hmac
import json
from urllib.parse import urlencode
import time
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread
import os
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识
target = -1
stop_recording = False

def input_listener():
    global stop_recording
    print("开始录音... 输入 'q' 停止录音")
    while True:
        user_input = input()
        if user_input == 'q':
            stop_recording = True
            print("录音结束")
            break

def classify(words):            # 文本输入大模型的API输出分类
    # 替换为你的实际访问令牌
    access_token = ""
    url = f"https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/completions?access_token={access_token}"
    # 定义要发送的消息
    payload = json.dumps({
        "messages": [
            {
                "role": "user",
                "content": f'这是字典[41:杯子,39:瓶子,73书本],模仿这个例子：请到B区拿个瓶子，再到A区拿一个杯子->B39A41。也就是你只要按出现顺序回答连续的‘区域字母-类别数字对’，接下来请听话：{words}'
            }
        ],
        "temperature": 0.95,
        "top_p": 0.8,
        "penalty_score": 1,
        "enable_system_memory": False,
        "disable_search": False,
        "enable_citation": False,
        "response_format": "text"
    })
    headers = {
        'Content-Type': 'application/json'
    }
    try:
        # 发送POST请求
        response = requests.post(url, headers=headers, data=payload)
        # 检查响应状态码
        if response.status_code == 200:
            # 打印返回的结果
            response_data = response.json()
            print("回答:", response_data.get('result', '没有返回结果'))
            return response_data.get('result', '没有返回结果')
        else:
            print(f"请求失败，状态码: {response.status_code}，错误信息: {response.text}")
            return -1
    except Exception as e:
        print(f"发生异常: {e}")
        return -1


class Ws_Param(object):
    # 初始化
    def __init__(self, APPID, APIKey, APISecret, AudioFile):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.AudioFile = AudioFile

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {"domain": "iat", "language": "zh_cn", "accent": "mandarin", "vinfo":1,"vad_eos":10000}

    # 生成url
    def create_url(self):
        url = 'wss://ws-api.xfyun.cn/v2/iat'
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        # 进行hmac-sha256进行加密
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        # 将请求的鉴权参数组合为字典
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        # 拼接鉴权参数，生成url
        url = url + '?' + urlencode(v)
        return url

# 收到websocket消息的处理
def on_message(ws, message):
    global target
    try:
        code = json.loads(message)["code"]
        sid = json.loads(message)["sid"]
        if code != 0:
            errMsg = json.loads(message)["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            data = json.loads(message)["data"]["result"]["ws"]
            result = ""
            for i in data:
                for w in i["cw"]:
                    result += w["w"]
            # print("sid:%s call success!,data is:%s" % (sid, json.dumps(data, ensure_ascii=False)))
        if len(result) > 2:
            print(result)
            target = classify(result)
    except Exception as e:
        print("receive msg,but parse exception:", e)

# 收到websocket错误的处理
def on_error(ws, error):
    print("### error:", error)

# 收到websocket关闭的处理
def on_close(ws, a, b):
    print("### closed ###")

# 收到websocket连接建立的处理
def on_open(ws):
    def run(*args):
        frameSize = 8000  # 每一帧的音频大小
        intervel = 0.04  # 发送音频间隔(单位:s)
        status = STATUS_FIRST_FRAME  # 音频的状态信息，标识音频是第一帧，还是中间帧、最后一帧

        with open(wsParam.AudioFile, "rb") as fp:
            while True:
                buf = fp.read(frameSize)
                if not buf:
                    status = STATUS_LAST_FRAME
                if status == STATUS_FIRST_FRAME:
                    d = {"common": wsParam.CommonArgs,
                         "business": wsParam.BusinessArgs,
                         "data": {"status": 0, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    d = json.dumps(d)
                    ws.send(d)
                    status = STATUS_CONTINUE_FRAME
                elif status == STATUS_CONTINUE_FRAME:
                    d = {"data": {"status": 1, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    ws.send(json.dumps(d))
                elif status == STATUS_LAST_FRAME:
                    d = {"data": {"status": 2, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    ws.send(json.dumps(d))
                    time.sleep(1)
                    break
                time.sleep(intervel)
        ws.close()

    thread.start_new_thread(run, ())

def record_audio(file_name):            # 录音生成文件
    FORMAT = pyaudio.paInt16  # 16-bit 深度
    CHANNELS = 1  # 单声道
    RATE = 16000  # 采样率16kHz
    CHUNK = 1024  # 每个块的帧数
    frames = []

    # 初始化pyaudio
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    threading.Thread(target=input_listener, daemon=True).start()

    while not stop_recording:
        data = stream.read(CHUNK)
        frames.append(data)

    # 停止并关闭音频流
    stream.stop_stream()
    stream.close()
    p.terminate()

    # 将录音数据保存为WAV文件
    with wave.open(file_name, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))

class AudioClassifyNode(Node):
    def __init__(self):
        super().__init__('audio_classify_node')
        self.timer = self.create_timer(1, self.publish_msg)
        self.publisher = self.create_publisher(String, 'voice_commands', 10)        # 创建话题，发布识别的目标的类
        
    def publish_msg(self):
        self.publisher.publish(String(data=str(target)))


audio_file = "output.wav"  # 输出音频文件名
wsParam = Ws_Param(APPID='your appid', APISecret='your apisecret',
                APIKey='your apikey',
                AudioFile=audio_file)

def main():
    # 先录音
    record_audio(audio_file)
    # 创建WebSocket连接并发送音频
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    ws = websocket.WebSocketApp(wsUrl, on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
    rclpy.init()
    my_node = AudioClassifyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()  # cleans up pub-subs, etc
    rclpy.shutdown()

if __name__ == "__main__":
    main()