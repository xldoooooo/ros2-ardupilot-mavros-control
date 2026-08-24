import time
import math
import multiprocessing
import sys

# -------------------------- 测试配置 --------------------------
TEST_DURATION_SEC = 10       # 每个单核心计算任务的持续时间
MATRIX_SIZE = 800            # 矩阵乘法维度
FIB_MAX = 40                 # 递归斐波那契深度
PI_ITERATIONS = 5_000_000    # 圆周率计算迭代次数
# -------------------------------------------------------------


def cpu_single_task(task_id):
    """单核心CPU暴力计算子任务"""
    start = time.perf_counter()
    total = 0.0
    i = 0
    while time.perf_counter() - start < TEST_DURATION_SEC:
        total += math.sin(i) * math.cos(i) * math.sqrt(i + 1)
        i += 1
    return i


def calculate_pi(iterations):
    """蒙特卡洛法计算圆周率，高浮点压力"""
    import random
    count = 0
    for _ in range(iterations):
        x = random.random()
        y = random.random()
        if x * x + y * y <= 1.0:
            count += 1
    return 4.0 * count / iterations


def fib(n):
    """递归斐波那契，压栈+整数运算压力"""
    if n <= 1:
        return n
    return fib(n - 1) + fib(n - 2)


def matrix_benchmark(size):
    """内存+CPU综合压力：大矩阵乘法"""
    import numpy as np
    a = np.random.rand(size, size)
    b = np.random.rand(size, size)
    t0 = time.perf_counter()
    c = a @ b
    t1 = time.perf_counter()
    return t1 - t0


def main():
    print("=" * 60)
    print("🎯 电脑综合暴力跑分开始")
    print(f"CPU核心数量: {multiprocessing.cpu_count()}")
    print("=" * 60)

    scores = []
    results = {}

    # 1. 单核浮点运算测试
    print("\n[1/4] 单核浮点持续运算测试...")
    t0 = time.perf_counter()
    single_loop_count = cpu_single_task(0)
    t1 = time.perf_counter()
    single_time = t1 - t0
    results["单核浮点_10秒循环次数"] = single_loop_count
    print(f"   耗时: {single_time:.2f}s  |  循环次数: {single_loop_count}")
    scores.append(single_loop_count / 10000)

    # 2. 全核心并行压力测试
    print("\n[2/4] 全核心并行运算测试...")
    t0 = time.perf_counter()
    with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
        multi_results = pool.map(cpu_single_task, range(multiprocessing.cpu_count()))
    t1 = time.perf_counter()
    total_multi_loops = sum(multi_results)
    results["多核浮点_总循环次数"] = total_multi_loops
    results["多核耗时"] = t1 - t0
    print(f"   耗时: {t1 - t0:.2f}s  |  总循环次数: {total_multi_loops}")
    scores.append(total_multi_loops / 200000)

    # 3. 圆周率计算测试
    print("\n[3/4] 圆周率蒙特卡洛计算测试...")
    t0 = time.perf_counter()
    pi_val = calculate_pi(PI_ITERATIONS)
    t1 = time.perf_counter()
    results["圆周率计算耗时"] = t1 - t0
    print(f"   π ≈ {pi_val:.8f}  |  耗时: {t1 - t0:.2f}s")
    scores.append(30.0 / max(t1 - t0, 0.001))

    # 4. 大矩阵运算测试 (numpy)
    print("\n[4/4] 大矩阵乘法测试...")
    try:
        mat_time = matrix_benchmark(MATRIX_SIZE)
        results["矩阵乘法耗时"] = mat_time
        print(f"   {MATRIX_SIZE}x{MATRIX_SIZE} 矩阵乘法耗时: {mat_time:.2f}s")
        scores.append(10.0 / max(mat_time, 0.001))
    except ImportError:
        print("   ⚠️ numpy未安装，跳过矩阵测试")
        scores.append(20.0)

    # 5. 递归计算测试
    print("\n[额外] 递归斐波那契深度测试...")
    t0 = time.perf_counter()
    fib_result = fib(FIB_MAX)
    t1 = time.perf_counter()
    results[f"斐波那契(F{FIB_MAX})耗时"] = t1 - t0
    print(f"   F{FIB_MAX} = {fib_result}  |  耗时: {t1 - t0:.2f}s")
    scores.append(5.0 / max(t1 - t0, 0.001))

    # 总分归一化
    total_score = sum(scores) * 10

    print("\n" + "=" * 60)
    print("📊 跑分结果汇总")
    print("=" * 60)
    for k, v in results.items():
        if isinstance(v, float):
            print(f"  {k} : {v:.3f}")
        else:
            print(f"  {k} : {v}")
    print(f"\n🏆 综合跑分得分: {total_score:.1f}")
    print("   (分数越高，整体计算性能越强)")
    print("=" * 60)


if __name__ == "__main__":
    main()
