#!/usr/bin/env python3
"""Offline hover errors, oscillation evidence and PD+DOB tuning directions; no ROS needed."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

# Changes to the active profile split windows instead of mixing different controllers.
PROFILE = ("tracking_controller", "wn_xy", "zeta_xy", "wn_z", "zeta_z", "observer_xy", "observer_z",
           "output_jerk", "acceleration_feedforward", "max_acceleration_xy", "max_acceleration_z",
           "max_disturbance", "max_tilt_rad", "gravity")
# Numeric vectors needed for statistics and reconstruction of the unclipped PD+DOB request.
VECTORS = ("position", "velocity", "reference_position", "reference_velocity",
           "reference_acceleration", "disturbance", "controller_acceleration", "mapped_acceleration")
# Integer identifiers must retain exact spelling, including values beyond float64 precision.
TEXT = {"session_id", "sequence", "header_frame_id"}


def load_rows(path):
    """Count corrupt rows and preserve break markers so bad records cannot join windows."""
    rows, invalid = [], 0
    required = {"schema_version", "monotonic_s", "dt_s", "control_mode", "calibration_active",
                "pose_age_s", "velocity_age_s", "reference_yaw", "thrust", "hover_throttle", *TEXT, *PROFILE}
    required.update(f"{name}_{a}" for name in VECTORS for a in "xyz")
    required.update(f"{name}_{a}" for name in ("attitude", "attitude_setpoint") for a in "xyzw")
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("CSV missing columns: " + ", ".join(sorted(missing)))
        for original in reader:
            try:
                row = {k: original[k] if k in TEXT else float(original[k]) for k in required}
                if row["schema_version"] != 1 or not all(
                        math.isfinite(v) for v in row.values() if isinstance(v, float)):
                    raise ValueError("unsupported schema or nonfinite sample")
                int(row["sequence"])
                int(row["session_id"])
                if row["monotonic_s"] <= 0 or any(row[k] <= 0 for k in ("wn_xy", "wn_z", "zeta_xy", "zeta_z")):
                    raise ValueError("invalid timestamp/gain")
                rows.append(row)
            except (ValueError, TypeError):
                invalid += 1
                rows.append(None)
    return rows, invalid


def vector(rows, name):
    """Read three FCU local ENU columns as an N×3 array."""
    return np.array([[row[f"{name}_{a}"] for a in "xyz"] for row in rows])


def hover_windows(rows, args):
    """Select by reference/mode, never by measured error or speed, retaining poor tracking."""
    windows, current = [], []
    first = next((r["monotonic_s"] for r in rows if r is not None), 0.)
    for row in rows:
        eligible = row is not None
        if eligible:
            relative = row["monotonic_s"] - first
            eligible = (row["control_mode"] in ({3, 4} if args.include_waypoint_holds else {3})
                        and not row["calibration_active"] and relative >= args.start
                        and (args.end is None or relative <= args.end)
                        and np.linalg.norm(vector([row], "reference_velocity")) <= .02
                        and np.linalg.norm(vector([row], "reference_acceleration")) <= .05)
        if not eligible:
            if current:
                windows.append(current)
            current = []
            continue
        if current:
            previous, anchor = current[-1], current[0]
            delta = row["monotonic_s"] - previous["monotonic_s"]
            same_source = all(row[k] == previous[k] for k in TEXT - {"sequence"})
            same_profile = all(row[k] == previous[k] for k in PROFILE)
            target_delta = np.linalg.norm(vector([row], "reference_position") -
                                          vector([anchor], "reference_position"))
            yaw_delta = math.atan2(math.sin(row["reference_yaw"] - anchor["reference_yaw"]),
                                   math.cos(row["reference_yaw"] - anchor["reference_yaw"]))
            if (not same_source or not same_profile or row["control_mode"] != previous["control_mode"]
                    or not 0 < delta <= args.max_gap
                    or int(row["sequence"]) <= int(previous["sequence"])
                    or target_delta > args.target_tolerance or abs(yaw_delta) > .02):
                windows.append(current)
                current = []
        current.append(row)
    if current:
        windows.append(current)
    retained = []
    for window in windows:
        trimmed = [row for row in window if row["monotonic_s"] - window[0]["monotonic_s"] >= args.settle]
        if len(trimmed) >= 8 and trimmed[-1]["monotonic_s"] - trimmed[0]["monotonic_s"] >= args.min_seconds:
            retained.append(trimmed)
    return retained, len(windows)


def spectrum(times, values):
    """Resample one continuous window, remove a linear trend, then apply a Hann FFT."""
    step = float(np.median(np.diff(times)))
    grid = np.arange(times[0], times[-1], step)
    data = np.interp(grid, times, values)
    data -= np.polyval(np.polyfit(grid - grid[0], data, 1), grid - grid[0])
    power = np.abs(np.fft.rfft(data * np.hanning(len(data)))) ** 2
    return np.fft.rfftfreq(len(data), step), power


def summarize(rows, args):
    """Measure tracking, DOB/limit activity and input quality before suggesting gain changes."""
    times = np.array([r["monotonic_s"] for r in rows])
    error = vector(rows, "reference_position") - vector(rows, "position")
    velocity_error = vector(rows, "reference_velocity") - vector(rows, "velocity")
    disturbance, mapped = vector(rows, "disturbance"), vector(rows, "mapped_acceleration")
    profile = {key: rows[0][key] for key in PROFILE}
    wn = np.array([profile["wn_xy"], profile["wn_xy"], profile["wn_z"]])
    zeta = np.array([profile["zeta_xy"], profile["zeta_xy"], profile["zeta_z"]])
    p_term, d_term = error * wn ** 2, velocity_error * (2 * zeta * wn)
    nominal = p_term + d_term
    if profile["acceleration_feedforward"]:
        nominal += vector(rows, "reference_acceleration")
    raw = nominal - disturbance
    thrust = np.array([r["thrust"] for r in rows])
    attitude = np.array([[r[f"attitude_{a}"] for a in "xyzw"] for r in rows])
    command = np.array([[r[f"attitude_setpoint_{a}"] for a in "xyzw"] for r in rows])
    norms = np.linalg.norm(attitude, axis=1) * np.linalg.norm(command, axis=1)
    angle = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(attitude * command, axis=1)) /
                                            np.maximum(norms, 1.e-12), 0, 1)))
    lost = sum(max(0, int(b["sequence"]) - int(a["sequence"]) - 1) for a, b in zip(rows, rows[1:]))
    duration = float(times[-1] - times[0])
    result = {"duration_s": duration, "samples": len(rows), "profile": profile,
              "source_session": rows[0]["session_id"], "start_monotonic_s": float(times[0]),
              "sample_rate_hz": float(1 / np.median(np.diff(times))), "gap_max_s": float(max(np.diff(times))),
              "skipped_snapshots": lost, "skipped_fraction": lost / (lost + len(rows)),
              "position_xy_rms_m": float(np.sqrt(np.mean(np.sum(error[:, :2] ** 2, axis=1)))),
              "position_xy_p95_m": float(np.percentile(np.linalg.norm(error[:, :2], axis=1), 95)),
              "thrust_mean": float(thrust.mean()), "hover_throttle_mean": float(np.mean([r["hover_throttle"] for r in rows])),
              "thrust_rail_fraction": float(np.mean((thrust <= .01) | (thrust >= .99))),
              "attitude_error_p95_deg": float(np.percentile(angle, 95)),
              "pose_age_max_s": max(r["pose_age_s"] for r in rows),
              "velocity_age_max_s": max(r["velocity_age_s"] for r in rows),
              "clamped_dt_fraction": float(np.mean([not .001 <= r["dt_s"] <= .05 for r in rows])),
              "axes": {}, "directions": []}
    quality_issue = (result["skipped_fraction"] > .1 or result["pose_age_max_s"] > .1
                     or result["velocity_age_max_s"] > .1 or result["clamped_dt_fraction"] > .01)
    if quality_issue:
        result["directions"].append("采样缺口/遥测年龄/控制周期异常：先核对链路和调度，当前调参方向可信度降低。跳号也可能来自快照覆盖，不能直接认定控制漏周期。")
    if result["attitude_error_p95_deg"] > 5:
        result["directions"].append("姿态跟随差异较大：先核对内环、姿态延迟及推力映射，再提高外环 wn。同周期姿态差也包含正常内环滞后。")
    for i, axis in enumerate("xyz"):
        frequencies, power = spectrum(times, error[:, i])
        band = (frequencies >= max(.05, 2 / duration)) & (frequencies <= min(5., .4 * result["sample_rate_hz"]))
        peak = int(np.flatnonzero(band)[np.argmax(power[band])]) if np.any(band) else 0
        total = float(np.sum(power[band]))
        if total <= 1.e-16:
            peak = 0  # Constant signals have no meaningful oscillation frequency.
        peak_fraction = min(1., float(np.sum(power[max(1, peak - 1):peak + 2]) / total)) if total > 1.e-16 else 0.
        _, dob_power = spectrum(times, disturbance[:, i])
        high = frequencies >= max(1., 2 * wn[i] / (2 * np.pi))
        dob_high_fraction = float(dob_power[high].sum() / max(dob_power[1:].sum(), 1.e-16))
        metrics = {"bias_m": float(error[:, i].mean()), "rms_m": float(np.sqrt(np.mean(error[:, i] ** 2))),
                   "std_m": float(error[:, i].std()), "p95_abs_m": float(np.percentile(np.abs(error[:, i]), 95)),
                   "peak_abs_m": float(np.max(np.abs(error[:, i]))),
                   "velocity_error_rms_m_s": float(np.sqrt(np.mean(velocity_error[:, i] ** 2))),
                   "p_rms_m_s2": float(np.sqrt(np.mean(p_term[:, i] ** 2))),
                   "d_rms_m_s2": float(np.sqrt(np.mean(d_term[:, i] ** 2))),
                   "dob_mean_m_s2": float(disturbance[:, i].mean()), "dob_std_m_s2": float(disturbance[:, i].std()),
                   "dob_limit_fraction": float(np.mean(np.abs(disturbance[:, i]) >= .98 * profile["max_disturbance"])),
                   "output_modified_fraction": float(np.mean(np.abs(mapped[:, i] - raw[:, i]) > .02)),
                   "dominant_hz": float(frequencies[peak]), "peak_power_fraction": peak_fraction,
                   "dob_high_frequency_fraction": dob_high_fraction,
                   "kp": float(wn[i] ** 2), "kd": float(2 * zeta[i] * wn[i])}
        result["axes"][axis] = metrics
        group = "xy" if i < 2 else "z"
        prefix = "trajectory_" if profile["tracking_controller"] == 1 else "hover_"
        dob_name = ("trajectory_dob_L_" if profile["tracking_controller"] == 1 else "dob_L_") + group
        if quality_issue:
            advice = "先改善采样/遥测质量，再依据本段调整增益。"
        elif metrics["dob_limit_fraction"] > .05 or metrics["output_modified_fraction"] > .05 or result["thrust_rail_fraction"] > .01:
            advice = "DOB/输出限幅或斜率限制明显：先核对偏置、推力映射及当前限额，暂缓增大 wn/L。"
        elif dob_high_fraction > .4 and metrics["dob_std_m_s2"] > .1:
            advice = f"DOB 高频活动明显：检查速度噪声；可将 {dob_name} 降低 10–20% 做同条件对照，观察误差和输出抖动是否同时改善。"
        elif metrics["std_m"] > args.position_tolerance / 2 and peak_fraction > .4 and frequencies[peak] * duration >= 3:
            advice = f"存在周期摆动候选：先将 {prefix}zeta_{group} 提高约 10%（只增 D）；若噪声/滞后加重，降低 {prefix}wn_{group} 约 10%，并对照较小 L。频谱不能单独确认因果。"
        elif abs(metrics["bias_m"]) > args.position_tolerance / 2:
            advice = "稳态偏差明显：先核对定位偏置、姿态零偏和悬停推力；仅凭静态偏差不能判定应提高 DOB 带宽。"
        elif metrics["rms_m"] > args.position_tolerance:
            advice = f"误差偏大且未见明显限幅/窄带摆动：若人工扰动后恢复确实偏慢，可将 {prefix}wn_{group} 提高 5–10%，需通过恢复过程对照确认。"
        else:
            advice = "当前位置误差较小，先保留参数；评价抗扰恢复速度需另录可辨认的扰动与恢复过程。"
        result["directions"].append(f"{axis.upper()}：{advice}")
    return result


def plot_window(rows, result, path):
    """Render standalone scientific figures without a display server."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = np.array([r["monotonic_s"] for r in rows])
    relative = times - times[0]
    error = vector(rows, "reference_position") - vector(rows, "position")
    velocity_error = vector(rows, "reference_velocity") - vector(rows, "velocity")
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), constrained_layout=True)
    for i, axis in enumerate("xyz"):
        axes[0, 0].plot(relative, error[:, i], label=axis)
        axes[0, 1].plot(relative, velocity_error[:, i], label=axis)
        axes[1, 0].plot(relative, vector(rows, "disturbance")[:, i], label=axis)
        axes[1, 1].plot(relative, vector(rows, "mapped_acceleration")[:, i], label=axis)
        f, p = spectrum(times, error[:, i])
        axes[3, 0].plot(f[1:], p[1:] / max(p[1:].sum(), 1.e-16), label=axis)
        axes[3, 1].plot(relative, vector(rows, "position")[:, i], label=axis, color=f"C{i}")
        axes[3, 1].plot(relative, vector(rows, "reference_position")[:, i],
                        linestyle="--", color=f"C{i}", alpha=.6)
    axes[2, 0].plot(relative, [r["thrust"] for r in rows], label="thrust")
    axes[2, 0].plot(relative, [r["hover_throttle"] for r in rows], label="hover throttle")
    for name in ("dt_s", "pose_age_s", "velocity_age_s"):
        axes[2, 1].plot(relative, [r[name] * 1000 for r in rows], label=name)
    titles = ("Position error [m]", "Velocity error [m/s]", "DOB [m/s^2]",
              "Mapped command (not measured) [m/s^2]", "Normalized thrust",
              "Control dt / input age [ms]", "Position spectrum (relative power)", "Position / reference dashed [m]")
    for ax, title in zip(axes.flat, titles):
        ax.set_title(title)
        ax.set_xlabel("Frequency [Hz]" if ax is axes[3, 0] else "Time [s]")
        ax.legend()
        ax.grid(alpha=.3)
    axes[3, 0].set_xlim(0, min(5., result["sample_rate_hz"] / 2))
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main(argv=None):
    """Write per-window JSON, Chinese tuning notes and optional PNG plots without ROS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="New result directory")
    parser.add_argument("--settle", type=float, default=2., help="Discard initial seconds of every window")
    parser.add_argument("--min-seconds", type=float, default=10.)
    parser.add_argument("--max-gap", type=float, default=.1, help="Split on larger source-time gaps")
    parser.add_argument("--target-tolerance", type=float, default=.01, help="Maximum reference drift [m]")
    parser.add_argument("--position-tolerance", type=float, default=.1, help="Tuning heuristic scale [m]")
    parser.add_argument("--start", type=float, default=0., help="Seconds from first source sample")
    parser.add_argument("--end", type=float)
    parser.add_argument("--include-waypoint-holds", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    for name in ("settle", "min_seconds", "max_gap", "target_tolerance", "position_tolerance", "start"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name not in {"settle", "start"} and value == 0):
            parser.error(f"--{name.replace('_', '-')} has invalid value")
    if args.end is not None and (not math.isfinite(args.end) or args.end <= args.start):
        parser.error("--end must be finite and greater than --start")
    try:
        rows, invalid = load_rows(args.csv)
    except (ValueError, OSError) as error:
        parser.exit(2, str(error) + "\n")
    windows, candidates = hover_windows(rows, args)
    args.output.mkdir(parents=True, exist_ok=False)
    summaries = [summarize(window, args) for window in windows]
    output = {"input": str(args.csv.resolve()), "invalid_rows": invalid,
              "candidate_windows": candidates, "analyzed_windows": len(windows),
              "settings": {k: v for k, v in vars(args).items() if not isinstance(v, Path)}, "windows": summaries}
    notes = ["# 悬停 PD+DOB 离线分析", "", f"输入：{args.csv.resolve()}",
             f"无效行：{invalid}；候选段：{candidates}；分析段：{len(windows)}。", "",
             "误差=参考−实测；坐标为 FCU local ENU。DOB 是真实内部估计值，不是实测外力；mapped_acceleration 是倾角限幅后的命令，未计入推力截断影响，不是实测加速度。",
             "建议是基于现象的调参方向，不是系统辨识结果；一次只改变一组参数并做同条件对照。", "",
             "Kp=wn²，Kd=2·zeta·wn；wn 增加10%使 P 增加21%、D 增加10%；zeta 只改变 D。",
             "静态悬停不能唯一识别最佳 wn/zeta/L；XY 共用参数，方向不一致时先排查机体/测量各向异性。", ""]
    if not windows:
        notes.append("没有足够长的连续悬停段；检查模式、目标变化、校准或缺口，或调整 --settle/--min-seconds。未生成调参结论。")
    for number, (window, result) in enumerate(zip(windows, summaries), 1):
        notes += [f"## 段 {number}", "", f"时长 {result['duration_s']:.1f}s；{result['sample_rate_hz']:.1f}Hz；"
                  f"快照缺失 {result['skipped_fraction']:.1%}；XY RMS {result['position_xy_rms_m']:.3f}m。", "",
                  f"实际增益：XY wn={result['profile']['wn_xy']:.3g}、zeta={result['profile']['zeta_xy']:.3g}、"
                  f"L={result['profile']['observer_xy']:.3g}；Z wn={result['profile']['wn_z']:.3g}、"
                  f"zeta={result['profile']['zeta_z']:.3g}、L={result['profile']['observer_z']:.3g}。", "",
                  "| 轴 | 偏差 m | RMS m | P95 m | 速度误差 RMS m/s | DOB 均值 m/s² | 主峰 Hz | 输出修改占比 |",
                  "| --- | --- | --- | --- | --- | --- | --- | --- |"]
        for axis, m in result["axes"].items():
            notes.append(f"| {axis} | {m['bias_m']:.3f} | {m['rms_m']:.3f} | {m['p95_abs_m']:.3f} | "
                         f"{m['velocity_error_rms_m_s']:.3f} | {m['dob_mean_m_s2']:.3f} | "
                         f"{m['dominant_hz']:.2f} | {m['output_modified_fraction']:.1%} |")
        notes += ["", *["- " + d for d in result["directions"]], ""]
        if not args.no_plots:
            try:
                name = f"window-{number:02d}.png"
                plot_window(window, result, args.output / name)
                notes += [f"![段{number}曲线]({name})", ""]
            except ImportError:
                notes += ["绘图依赖缺失：安装 matplotlib，或使用 --no-plots；数值分析已保留。", ""]
    (args.output / "summary.json").write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    (args.output / "analysis.md").write_text("\n".join(notes) + "\n")
    print(f"Analyzed {len(windows)} windows → {args.output / 'analysis.md'}")
    return 0 if windows else 1


if __name__ == "__main__":
    raise SystemExit(main())
