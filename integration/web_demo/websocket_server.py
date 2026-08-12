import asyncio
import websockets
import json
import socket

# 获取本机局域网IP
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = "127.0.0.1"
    finally:
        s.close()
    return local_ip

# 你预先定义好的无人机航点任务报文
UAV_TASK_PAYLOAD = {
    "clientNo": "UAV01001",
    "commandNo": "02",
    "taskPoints": [
        {
            "index": 1, "x": 3.4, "y": -0.549, "z": 0.221, "forwardAngle": 0, "cameraAngle": 1495, "photoNo": 1
        },
        {
            "index": 2, "x": 14.67, "y": -0.548, "z": 0.219, "forwardAngle": -90, "cameraAngle": 1495, "photoNo": 2
        }
    ]
}
ncount = 0
async def handle_client(websocket):
    print("✅ 无人机/客户端成功连接")

    try:
        async def send_loop():
            while True:
                global ncount
                ncount = ncount + 1

                #定时发送消息
                # await asyncio.sleep(1)
                # user_input = ncount
                # send_data = json.dumps({"msg": user_input, "sender": "server"})
                # await websocket.send(send_data)

                # 支持两种发送模式
                user_input = await asyncio.to_thread(input, "输入普通文本消息直接发送 | 输入send_task一键发送航点任务: ")
                if user_input.strip() == "send_task":
                    send_data = json.dumps(UAV_TASK_PAYLOAD)
                    print("📤 正在发送预定义航点任务报文...")
                else:
                    send_data = json.dumps({"msg": user_input, "sender": "server"})
                await websocket.send(send_data)

        async def recv_loop():
            async for raw_msg in websocket:
                try:
                    recv_data = json.loads(raw_msg)
                    # 识别无人机回传的任务状态
                    if recv_data.get("commandNo") == "02":
                        print(f"\n✅ 无人机返回航点任务执行状态: {recv_data}")
                    else:
                        print(f"\n📩 收到普通消息: {recv_data}")
                except json.JSONDecodeError:
                    print("\n⚠️ 收到非标准JSON报文，自动跳过")

        await asyncio.gather(send_loop(), recv_loop())

    except websockets.exceptions.ConnectionClosed:
        print("\n🔌 无人机断开连接，服务端保持运行")
    finally:
        await websocket.close()

async def main():
    local_ip = get_local_ip()
    async with websockets.serve(handle_client, "0.0.0.0", 8765):
        print(f"🚀 航点指令服务端启动完成，局域网地址: ws://{local_ip}:8765")
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 服务端手动退出")
