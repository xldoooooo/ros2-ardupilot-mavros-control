import paho.mqtt.client as mqtt
import time
import random
import json

# ================= 配置区域 =================
# 使用 EMQX 免费的公共测试服务器
BROKER = "192.168.30.37"
WS_PORT = 8083  # EMQX 默认的 WebSocket 端口
TOPIC = "test/drone0001/command"
TOPIC2 = "test/drone0001/status"
CLIENT_ID = "drone0001" #无人机编号
# ===========================================

# 1. 连接成功时的回调函数
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✅ 成功通过 WebSocket 连接到 EMQX Broker: {BROKER}")
        # 连接成功后，自动订阅指定主题
        client.subscribe(TOPIC)
        print(f"📥 已订阅主题: {TOPIC}")
    else:
        print(f"❌ 连接失败，返回码: {rc}")

# 2. 接收到消息时的回调函数
def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8")
    print(f"📨 收到消息 | 主题: {msg.topic} | 内容: {payload}")

# 3. 创建 MQTT 客户端实例 (关键：指定 transport="websockets")
client = mqtt.Client(client_id=CLIENT_ID, transport="websockets")

# 4. 绑定回调函数
client.on_connect = on_connect
client.on_message = on_message

# 5. 建立 WebSocket 连接并启动后台网络循环
print(f"🚀 正在通过 WebSocket 连接 EMQX Broker...")
try:
    client.connect(BROKER, WS_PORT, keepalive=60)
    client.loop_start()  # 在后台线程处理网络通信和消息接收

    # 6. 模拟定时发布消息
    count = 0
    while True:
        payload = json.dumps({
            "count": count, 
            "msg": "Hello EMQX via WebSocket!",
            "protocol": "ws"
        })
        
        # 发布消息 (qos=1 表示至少送达一次)
        result = client.publish(TOPIC2, payload, qos=1)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"📤 消息已发布: {payload}")
        else:
            print(f"⚠️ 消息发布失败，错误码: {result.rc}")
            
        count += 1
        time.sleep(3)  # 每 3 秒发布一次
        
except KeyboardInterrupt:
    print("\n🛑 收到停止信号，正在断开连接...")
except Exception as e:
    print(f"❌ 连接异常: {e}")
finally:
    # 优雅地停止后台网络循环并断开连接
    client.loop_stop()
    client.disconnect()
    print("🔌 已安全断开连接")