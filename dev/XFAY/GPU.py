import torch
import time

# -------------------------- 跑分配置 --------------------------
MATRIX_SIZE = 4096        # 基础矩阵尺寸
BIG_MATRIX_SIZE = 8192    # 超大矩阵尺寸
ITERATIONS = 50           # 标准测试循环次数
WARMUP_ITER = 10          # 预热次数
# -------------------------------------------------------------


def print_banner(title):
    print("\n" + "=" * 60)
    print(f"🎮 {title}")
    print("=" * 60)


def main():
    # 环境检查
    print_banner("PyTorch CUDA 环境检测")
    print(f"PyTorch 版本: {torch.__version__}")
    print(f"CUDA 可用: {torch.cuda.is_available()}")
    if not torch.cuda.is_available():
        print("❌ 错误: CUDA 不可用，请先正确安装CUDA版PyTorch")
        return

    device = torch.device("cuda")
    gpu_name = torch.cuda.get_device_name(device)
    gpu_total_mem = torch.cuda.get_device_properties(device).total_memory / 1024**3

    print(f"显卡名称: {gpu_name}")
    print(f"显存总量: {gpu_total_mem:.2f} GB")

    scores = []
    results = {}

    # ---------------- 预热阶段 ----------------
    print_banner("显卡预热")
    print("正在预热显卡，避免跑分结果不准...")
    a_warm = torch.randn(MATRIX_SIZE, MATRIX_SIZE, device=device)
    b_warm = torch.randn(MATRIX_SIZE, MATRIX_SIZE, device=device)
    for _ in range(WARMUP_ITER):
        c_warm = a_warm @ b_warm
        torch.cuda.synchronize()
    print("✅ 预热完成")

    # ---------------- 测试1: 常规矩阵乘法吞吐 ----------------
    print_banner("测试1 - 常规FP32矩阵乘法")
    a = torch.randn(MATRIX_SIZE, MATRIX_SIZE, device=device)
    b = torch.randn(MATRIX_SIZE, MATRIX_SIZE, device=device)

    t0 = time.perf_counter()
    for _ in range(ITERATIONS):
        c = a @ b
        torch.cuda.synchronize()
    t1 = time.perf_counter()

    avg_time = (t1 - t0) / ITERATIONS
    gflops = (2 * MATRIX_SIZE**3) / (avg_time * 1e9)
    results[f"{MATRIX_SIZE}x{MATRIX_SIZE}平均耗时"] = avg_time
    results[f"估算FP32 TFLOPS"] = gflops / 1000
    print(f"单轮平均耗时: {avg_time*1000:.2f} ms")
    print(f"估算理论FP32算力: {gflops/1000:.2f} TFLOPS")
    scores.append(gflops)

    # ---------------- 测试2: 超大矩阵压力 ----------------
    print_banner("测试2 - 超大矩阵全显存压力")
    a_big = torch.randn(BIG_MATRIX_SIZE, BIG_MATRIX_SIZE, device=device)
    b_big = torch.randn(BIG_MATRIX_SIZE, BIG_MATRIX_SIZE, device=device)

    big_iter = 10
    t0 = time.perf_counter()
    for _ in range(big_iter):
        c_big = a_big @ b_big
        torch.cuda.synchronize()
    t1 = time.perf_counter()

    avg_big_time = (t1 - t0) / big_iter
    big_gflops = (2 * BIG_MATRIX_SIZE**3) / (avg_big_time * 1e9)
    results[f"{BIG_MATRIX_SIZE}x{BIG_MATRIX_SIZE}平均耗时"] = avg_big_time
    results[f"超大矩阵FP32 TFLOPS"] = big_gflops / 1000
    print(f"单轮平均耗时: {avg_big_time*1000:.2f} ms")
    print(f"超大矩阵算力: {big_gflops/1000:.2f} TFLOPS")
    scores.append(big_gflops)

    # ---------------- 测试3: 张量逐元素运算 ----------------
    print_banner("测试3 - 大规模逐元素计算")
    elem_size = 20000
    x = torch.randn(elem_size, elem_size, device=device)
    y = torch.randn(elem_size, elem_size, device=device)

    elem_iter = 100
    t0 = time.perf_counter()
    for _ in range(elem_iter):
        z = torch.sin(x) + torch.cos(y) + torch.sqrt(torch.abs(x))
        torch.cuda.synchronize()
    t1 = time.perf_counter()

    elem_throughput = (elem_size * elem_size * 3 * elem_iter) / (t1 - t0) / 1e9
    results["逐元素运算总耗时"] = t1 - t0
    results["每秒百亿次操作数"] = elem_throughput
    print(f"总耗时: {t1 - t0:.2f} s")
    print(f"运算吞吐: {elem_throughput:.2f}e9 op/s")
    scores.append(elem_throughput * 100)

    # ---------------- 测试4: FP16半精度算力 ----------------
    print_banner("测试4 - FP16半精度张量核心跑分")
    a_fp16 = torch.randn(MATRIX_SIZE, MATRIX_SIZE, device=device, dtype=torch.half)
    b_fp16 = torch.randn(MATRIX_SIZE, MATRIX_SIZE, device=device, dtype=torch.half)

    t0 = time.perf_counter()
    for _ in range(ITERATIONS):
        c_fp16 = a_fp16 @ b_fp16
        torch.cuda.synchronize()
    t1 = time.perf_counter()

    avg_fp16_time = (t1 - t0) / ITERATIONS
    fp16_tflops = (2 * MATRIX_SIZE**3) / (avg_fp16_time * 1e12)
    results["FP16平均耗时"] = avg_fp16_time
    results["FP16 TFLOPS"] = fp16_tflops
    print(f"单轮平均耗时: {avg_fp16_time*1000:.2f} ms")
    print(f"估算FP16张量核心算力: {fp16_tflops:.2f} TFLOPS")
    scores.append(fp16_tflops * 1000)

    # ---------------- 总分统计 ----------------
    print_banner("最终GPU跑分结果汇总")
    for k, v in results.items():
        print(f"  {k} : {v:.3f}")

    total_gpu_score = sum(scores) / 10
    print(f"\n🏆 RTX 5080 综合GPU跑分得分: {total_gpu_score:.1f}")
    print(f"💡 得分越高，CUDA浮点算力越能满血发挥")
    print("=" * 60)


if __name__ == "__main__":
    main()
