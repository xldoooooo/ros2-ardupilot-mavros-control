import asyncio
import websockets
import json

# 替换成服务端显示的局域网WS地址
SERVER_WS_URL = "ws://127.0.0.1:8765"

async def parse_uav_task(task_json):
    # 自动解析航点报文，不需要手动解析字段
    if task_json.get("commandNo") == "02":
        print(f"\n🚁 收到无人机航点任务 | 无人机编号: {task_json['clientNo']}")
        for point in task_json["taskPoints"]:
            print(f"航点{point['index']}: 坐标({point['x']},{point['y']},{point['z']}) | 偏航角{point['forwardAngle']}° | 云台角度{point['cameraAngle']} | 拍照序号{point['photoNo']}")
        return True
    else:
        print(f"\n📩 收到普通消息: {task_json}")
    return False

# 把核心连接逻辑完整包装在异步函数内
async def connect_and_run():
    print("🔄 尝试连接航点指令服务端...")
    async with websockets.connect(SERVER_WS_URL, ping_interval=10, ping_timeout=30) as websocket:
        print("✅ 无人机端成功接入指令系统")
        async def send_loop():
            while True:
                # 无人机端也可以手动回传状态给服务端
                feedback = await asyncio.to_thread(input, "输入无人机回传状态直接发送: ")
                await websocket.send(json.dumps({"feedback": feedback}))
                # for i in range(10000):
                #     await websocket.send(json.dumps({"feedback": i}))
                #     await asyncio.sleep(1)

        async def recv_loop():
            async for raw_msg in websocket:
                try:
                    recv_data = json.loads(raw_msg)
                    # 自动识别航点任务，解析打印明细
                    await parse_uav_task(recv_data)
                except json.JSONDecodeError:
                    print("⚠️ 收到非标准JSON报文，自动跳过处理")

        await asyncio.gather(send_loop(), recv_loop())

# 主重连逻辑也放到异步主函数中
async def main():
    while True:
        try:
            await connect_and_run()
        except (websockets.exceptions.ConnectionClosed, OSError):
            print("\n❌ 连接断开，3秒后自动重连...")
            await asyncio.sleep(3)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 无人机客户端手动退出")
