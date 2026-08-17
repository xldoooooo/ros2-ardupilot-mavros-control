import asyncio
import json
import logging
import random
from datetime import datetime

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class NativeIotClient:
    def __init__(self, device_id, server_url="ws://localhost:8080/ws-native"):
        self.device_id = device_id
        self.server_url = server_url
        self.ws = None
        self.is_running = False
        
        # 模拟设备状态
        self.device_status = {
            "temperature": 25.0,
            "humidity": 60.0,
            "status": "online"
        }

    async def connect(self):
        """建立连接并启动监听"""
        self.is_running = True
        while self.is_running:
            try:
                logger.info(f"[{self.device_id}] 正在连接: {self.server_url}")
                async with websockets.connect(self.server_url) as websocket:
                    self.ws = websocket
                    logger.info(f"[{self.device_id}] 连接成功")
                    
                    # 1. 订阅主题 (例如: "devices/status")
                    await self.subscribe("devices/status")
                    
                    # 并行运行任务
                    await asyncio.gather(
                        self.listen_for_messages(),
                        self.report_status_loop(),
                        self.heartbeat_loop()
                    )
            except websockets.exceptions.ConnectionClosed as e:
                logger.warning(f"[{self.device_id}] 连接关闭: {e.code}")
            except ConnectionRefusedError:
                logger.error(f"[{self.device_id}] 连接被拒绝")
            except Exception as e:
                logger.error(f"[{self.device_id}] 错误: {e}")
            
            if self.is_running:
                wait_time = min(2 ** (getattr(self, 'reconnect_attempts', 0)), 30)
                self.reconnect_attempts = getattr(self, 'reconnect_attempts', 0) + 1
                logger.info(f"[{self.device_id}] {wait_time}秒后重连...")
                await asyncio.sleep(wait_time)

    async def subscribe(self, topic):
        """发送订阅请求"""
        msg = {
            "type": "SUBSCRIBE",
            "topic": topic
        }
        await self.send_json(msg)
        logger.info(f"[{self.device_id}] 已请求订阅主题: {topic}")

    async def listen_for_messages(self):
        """监听服务器下发的命令或广播"""
        try:
            async for message in self.ws:
                data = json.loads(message)
                msg_type = data.get("type")
                
                if msg_type == "SUB_ACK":
                    logger.info(f"[{self.device_id}] 订阅确认: {data.get('topic')}")
                elif msg_type == "BROADCAST":
                    logger.info(f"[{self.device_id}] 收到广播 [Topic: {data.get('topic')}]: {data.get('data')}")
                    # 在这里处理收到的命令，例如控制硬件
                    await self.handle_command(data.get('data'))
                elif msg_type == "HEARTBEAT_ACK":
                    pass # 心跳响应，无需处理
                elif msg_type == "ERROR":
                    logger.error(f"[{self.device_id}] 服务器错误: {data.get('msg')}")
        except websockets.exceptions.ConnectionClosed:
            pass

    async def handle_command(self, command_data):
        """处理具体命令"""
        if not command_data:
            return
        # 示例：如果收到 {"action": "blink"}
        action = command_data.get("action")
        if action == "blink":
            logger.info(f"[{self.device_id}] 执行闪烁操作...")
            # 模拟硬件操作
            await asyncio.sleep(0.1)

    async def report_status_loop(self):
        """定期发布状态到主题"""
        while self.is_running and self.ws:
            try:
                # 更新模拟数据
                self.device_status["temperature"] += random.uniform(-0.5, 0.5)
                
                publish_msg = {
                    "type": "PUBLISH",
                    "topic": "devices/status",
                    "data": {
                        "deviceId": self.device_id,
                        "metrics": self.device_status
                    }
                }
                await self.send_json(publish_msg)
                await asyncio.sleep(5) # 每5秒发布一次
            except Exception as e:
                logger.error(f"[{self.device_id}] 状态发布失败: {e}")
                break

    async def heartbeat_loop(self):
        """发送心跳"""
        while self.is_running and self.ws:
            try:
                await self.send_json({"type": "HEARTBEAT"})
                await asyncio.sleep(10)
            except Exception as e:
                break

    async def send_json(self, data):
        """发送 JSON 消息"""
        if self.ws and not self.ws.closed:
            await self.ws.send(json.dumps(data))

    def stop(self):
        self.is_running = False
        if self.ws:
            asyncio.create_task(self.ws.close())

if __name__ == "__main__":
    import websockets
    
    # 模拟两个设备
    dev1 = NativeIotClient("DEV-001")
    dev2 = NativeIotClient("DEV-002")

    async def main():
        await asyncio.gather(dev1.connect(), dev2.connect())

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        dev1.stop()
        dev2.stop()
        print("退出")