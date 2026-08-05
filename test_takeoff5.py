#!/usr/bin/env python3
"""起飞测试 v5：先检查初始武装状态，已武装则 disarm 再从头来。"""

import rclpy
import sys
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, SetMavFrame, MessageInterval
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    rclpy.init()
    node = rclpy.create_node("takeoff_v5")

    # ═══ 起飞高度：命令行参数或默认 ═══
    if len(sys.argv) > 1:
        try:
            TAKEOFF_ALT = float(sys.argv[1])
        except ValueError:
            print(f"[TEST] Invalid altitude arg '{sys.argv[1]}', using default 1.0", flush=True)
            TAKEOFF_ALT = 1.0
    else:
        TAKEOFF_ALT = 1.0

    # ═══ 消息频率 ═══
    rate_client = node.create_client(MessageInterval, "/mavros/set_message_interval")
    rate_client.wait_for_service(timeout_sec=10.0)
    for msg_id, rate in [(32, 100.0), (31, 100.0), (105, 100.0)]:
        req = MessageInterval.Request()
        req.message_id = msg_id; req.message_rate = rate
        fut = rate_client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        print(f"[TEST] rate {msg_id}={rate}: {'OK' if fut.result().success else 'FAIL'}", flush=True)

    # ═══ Timer 驱动 setpoint ═══
    target_pose = PoseStamped()
    target_pose.pose.orientation.w = 1.0
    pos_pub = node.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 1)
    timer = node.create_timer(0.1, lambda: pos_pub.publish(target_pose))

    set_mode_client = node.create_client(SetMode, "/mavros/set_mode")
    arming_client = node.create_client(CommandBool, "/mavros/cmd/arming")
    takeoff_client = node.create_client(CommandTOL, "/mavros/cmd/takeoff")
    frame_client = node.create_client(SetMavFrame, "/mavros/setpoint_velocity/mav_frame")

    armed = [False]
    mode = [""]
    def state_cb(msg: State):
        armed[0] = msg.armed
        mode[0] = msg.mode
    node.create_subscription(State, "/mavros/state", state_cb, 10)

    # ═══ 0) 检查初始状态 — 如果已武装，先 disarm ═══
    print("[TEST] 0) Checking initial state...", flush=True)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"[TEST]    initial: armed={armed[0]}, mode={mode[0]}", flush=True)

    if armed[0]:
        print("[TEST]    Already armed! Disarming first...", flush=True)
        req_d = CommandBool.Request()
        req_d.value = False
        fut = arming_client.call_async(req_d)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        print(f"[TEST]    Disarm result: {fut.result().success}", flush=True)
        # 等状态更新
        for i in range(50):
            rclpy.spin_once(node, timeout_sec=0.1)
            if not armed[0]:
                break
            if i % 10 == 0:
                print(f"[TEST]    waiting disarm... armed={armed[0]}", flush=True)
        print(f"[TEST]    armed={armed[0]}", flush=True)
        if armed[0]:
            print("[TEST] FAIL: Can't disarm!", flush=True); return

    # ── 1) 预发 + GUIDED ──
    print("[TEST] 1) GUIDED...", flush=True)
    req = SetMode.Request(); req.custom_mode = "GUIDED"
    fut = set_mode_client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.result() or not fut.result().mode_sent:
        print("[TEST] FAIL", flush=True); return

    # ── 2) Arm ──
    print("[TEST] 2) Arm...", flush=True)
    req_a = CommandBool.Request(); req_a.value = True
    fut = arming_client.call_async(req_a)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    print(f"[TEST]    success={fut.result().success}", flush=True)

    # BODY_FRAME
    if frame_client.wait_for_service(timeout_sec=2.0):
        fr = SetMavFrame.Request(); fr.mav_frame = 1
        frame_client.call_async(fr)

    # ── 3) Wait armed ──
    print("[TEST] 3) Wait armed...", flush=True)
    for i in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if armed[0]: break
        if i % 10 == 0: print(f"[TEST]    armed={armed[0]}", flush=True)
    print(f"[TEST]    armed={armed[0]}", flush=True)
    if not armed[0]:
        print("[TEST] FAIL: Not armed", flush=True); return

    # ── 4) Wait GUIDED ──
    print("[TEST] 4) Wait GUIDED...", flush=True)
    for i in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if mode[0] == "GUIDED": break
    print(f"[TEST]    mode={mode[0]}", flush=True)

    # ── 5) Stabilize ──
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    # ── 6) Takeoff ──
    print(f"[TEST] 6) Takeoff (armed={armed[0]})...", flush=True)
    req_t = CommandTOL.Request()
    req_t.altitude = TAKEOFF_ALT; req_t.min_pitch = 0.0; req_t.yaw = 0.0
    req_t.latitude = 0.0; req_t.longitude = 0.0

    for attempt in range(3):
        fut = takeoff_client.call_async(req_t)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        ok = fut.result() and fut.result().success
        print(f"[TEST]    attempt {attempt+1}: {ok}, armed={armed[0]}", flush=True)
        if ok:
            # 对标 C++：takeoff 后停止发位置 setpoint，让 drone 自己爬升
            timer.cancel()
            break
        time.sleep(1.0)

    # ── 7) Wait liftoff ──
    print("[TEST] 7) Wait liftoff...", flush=True)
    alt = [0.0]
    def alt_cb(msg: PoseStamped):
        alt[0] = msg.pose.position.z
    node.create_subscription(
        PoseStamped, "/mavros/local_position/pose", alt_cb,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    for i in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if alt[0] > 0.15: break
        if i % 15 == 0:
            print(f"[TEST]    z={alt[0]:.2f}m, armed={armed[0]}", flush=True)

    print(f"[TEST] FINAL: z={alt[0]:.2f}m, armed={armed[0]}", flush=True)
    print("[TEST] SUCCESS!" if alt[0] >= 0.15 else "[TEST] FAIL", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
