# 甲方现场演示指引

## 最推荐的演示命令

在仓库根目录打开一个较大的终端，执行：

```bash
bash integration/websocket_test_demo/run_showcase.sh --step
```

程序会自动启动甲方提供的原始 JAR。看到 `>>>` 时先讲解当前屏幕，再按 Enter 进入下一步。
随时可以按 Ctrl+C；程序只清理自己启动的 JAR 和 WebSocket 连接。
默认完整 JSON 以单行显示，现场更容易跟随；需要展开缩进时追加 `--pretty-json`。

如果只想自动播放，不逐步按 Enter：

```bash
bash integration/websocket_test_demo/run_showcase.sh --delay 1.2
```

## 开场可以这样说

> 现在演示的是贵方地面站到我方地面站 client 的 WebSocket 通讯，不连接真实飞机。
> 紫色是贵方提供的原始 JAR 通道，青色是贵方地面站模拟端，绿色是我方地面站 client。
> 每一步都会显示发送方向、主题和真实 JSON，而不是预先写好的结论。

终端支持颜色时：

- 青色：甲方地面站模拟端；
- 紫色：甲方原始 WebSocket JAR；
- 绿色：我方地面站 client；
- 黄色：本地验收和计时结论。

## 五幕讲解顺序

### 第 1 幕：连接和订阅

重点指出：

1. JAR 真实返回 `SYSTEM: Connected successfully`。
2. 甲方端订阅 `drone/UAV01001/status`。
3. 我方 client 订阅 `drone/UAV01001/command`。
4. 双方均收到真实 `SUB_ACK`，不是本地伪造成功。

### 第 2 幕：四类控制命令

程序依次演示 `02/03/05/07`。以 `02` 为例说明：

1. 甲方发送完整 `PUBLISH` 信封，`data` 中包含两个巡检点。
2. JAR 转成 `BROADCAST`，我方收到完整原始信封。
3. 我方解析 `clientNo`、`commandNo` 和全部航点字段。
4. 我方向状态主题发布协议表 1 的 `{clientNo, commandNo}` 确认。
5. 我方再发布对应 `uavStatus`。
6. 甲方状态订阅端收到两条带真实 `sender` 会话 ID 的 `BROADCAST`。

屏幕上的“仅记录模拟业务动作”是刻意的安全边界：本次不连接飞控，不执行真实紧急停机、
返航、降落或起飞。

### 第 3 幕：主动状态上报

演示剩余状态 `01/08/09/0A/0B/0C`，与第 2 幕的 `02/03/05/07` 合计覆盖协议十种状态。
重点展示带详细数据的：

- `08`：视频路径和 JPG 文件夹；
- `09`：点位编号、名称和照片路径；
- `0A`：电量百分比；
- `0B`：XYZ 位置；
- `0C`：电量不足、暂停巡检并返航充电。

`0C` 来自权威 DOCX 表 2；甲方随附 TXT 漏掉了该行。

### 第 4 幕：1 秒周期遥测

程序实际等待并接收两轮 `0A` 和 `0B`，最后根据接收时刻计算间隔。这里可以强调：

> 这个数值是本轮现场测出来的，不是固定打印的预期值。

### 第 5 幕：多飞机隔离

第二个我方 client 使用 `UAV01002`：

- 只订阅 `drone/UAV01002/command`；
- 响应只发布到 `drone/UAV01002/status`；
- `UAV01001` 精确状态主题在隔离观察窗口中没有收到消息。

这对应“甲方一个地面站维护多个 WebSocket/主题，我方每个地面站只控制一架飞机”。

## 连接甲方实际测试服务器

如果甲方已经在局域网启动服务器，不让脚本启动本地 JAR：

```bash
bash integration/websocket_test_demo/run_showcase.sh --step \
  --server-url ws://甲方服务器IP:8581/ws
```

只有在甲方确认实际路径后替换 URL。协议文档示例路径和本次 JAR 路径不同，不能自行猜测。

## 保存演示记录

保存无颜色、可直接发给甲方的终端全文：

```bash
bash integration/websocket_test_demo/run_showcase.sh --delay 0.8 --no-color \
  | tee integration/websocket_test_demo/results/showcase.log
```

快速回归仍使用：

```bash
bash integration/websocket_test_demo/run_demo.sh
```

快速回归用于确认 `23/23 + 25/25`，逐帧入口才用于讲过程。
