# 上位机联调资料索引；协议原件与测试服务器保留供对照。

无人机与上位机通信协议V2.2.pdf：当前协议原件。
无人机与上位机通信协议V2.1(1).pdf、V2.0.docx：历史原件，仅作差异核对。

websocket-server-1.0.0.jar：生产环境提供的主题路由服务器，无飞行业务逻辑。
在仓库根目录执行 java -jar dev/integration/websocket-server-1.0.0.jar，
本地测试 URL 为 ws://127.0.0.1:8581/ws；现场服务地址由上位机提供。

地面站与测试客户端均连接服务器，不能直接互连。
当前消息格式及接入步骤见 docs/websocket/上位机-WebSocket 对接.md。
web_demo、web_demo_2 和 websocket_test_demo 为历史联调示例。
