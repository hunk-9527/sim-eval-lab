"""07_low_speed_balance.py -- 低速段（v < 6 m/s）平衡点标定：换掉"跑到速度不变"的测法。

为什么必须重做
--------------
03 的稳态判据是「窗口首尾速度差 < 0.05 m/s 即认为到稳态」。实测它在 v < 6 m/s 段失效：

- **换挡狩猎**：carlacola @ theta=0.60 跑了 920 拍（92 s）卡在 v ≈ 5.0 上下振荡，
  a 在 ±0.5 ~ ±2.5 m/s² 之间持续交替 —— 首尾速度差被振荡**平均掉了**，
  判据说"稳态"，但那是**极限环**，不是平衡点。
- 于是 38 个"真稳态"里 23 个 drift 贴在 0.05 阈值下沿（**判据定义的量 ≠ 物理量**）。

而 ACC 跟车恰恰常工作在 0~15 m/s —— 这一段必须有可用标定，否则把 acc_sim 接进来
是在它最常工作的速度段上瞎跑，测出的问题分不清是算法还是标定。

改法（本脚本）
--------------
1. **判据换掉**：不再"跑到速度不变就停"，而是**跑固定时长**（warmup + window），
   只看最后 window 拍的**区间统计量**：
     v_mean / v_std / 净漂移率 / 穿越均值的次数
   -> 分类：
     「稳定平衡」均值稳、方差小、净漂移≈0
     「极限环」  均值稳但**方差大**、反复穿越均值   ← 03 会把它误记成稳态
     「未达平衡」净漂移仍然显著
     「死区」    全程没动
   固定时长还有个副作用好处：**不再有"刚好满足判据就停机"**，
   采样点和判据解耦（S5 那个坑从根上没了）。

2. **稳定性直接测（迟滞对照）**：同一档油门跑两条路径 ——
     **升速路径**（从静止起，theta 直接加载）
     **降速路径**（先用高油门冲到远高于目标的速度，再阶跃降到 theta）
   - 两条路径落到同一个速度 -> **稳定的吸引子**
   - 落到不同速度        -> **迟滞**：该点不是唯一平衡，控制器的平衡点不可信
   （换挡狩猎的本质就是迟滞，这条对照正面命中它。）

3. **判据与阈值写进产出**（`balance_notes.json`）—— 交付物必须带适用边界。

跑法（必须全路径 python + utf-8）
--------------------------------
    PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe 07_low_speed_balance.py \
        --vehicle vehicle.tesla.model3
    ... --levels 0.10,0.15,0.20 --modes up          # 只跑单路径
    ... --skip-run                                   # 只汇总已有曲线

分档落盘 + 已有结果自动跳过 —— CARLA 崩了重跑即可，不丢已完成档。

⚠ 不要用 client.load_world() 切地图（P-SIM-01）。本脚本连不上就交给 carla_env 重启。
"""

import argparse
import csv
import json
import os
import statistics as st
import sys
import time

from carla_env import ensure_carla, settle, slug, speed_of, tick_pinned

# ------------------------- 实验参数 -------------------------
DT = 0.1                  # 固定步长，必须对齐 acc_sim 的 AccConfig.dt
WARMUP = 400              # 40 s：低油门起步有 3~4 s 蓄力期，换挡狩猎可能要几十秒才成形
WINDOW = 500              # 50 s：统计窗口。**这个值是实测选出来的，不是拍的** ——
# 拿 tesla 跑 300 / 500 / 600 三个窗口对比：300 与 500 给出同样的 v_mean 与 Δ
# （迟滞差稳定在 +0.17~+0.22），说明系统确实已静止；600 则开始吃掉 warmup，
# 把降速段的衰减圈进来，全体 v_std 从 ~0.01 跳到 0.12~0.44（假振荡）。
# → 500 是"统计量够长"与"不污染 warmup"的平衡点。
MAX_STEPS = 3000          # 单次运行 tick 上限（保险丝，不该被触发）

# 低速段阶梯：加密到 0.05 一档，覆盖 03 表里"v<6 m/s"的空洞区
LOW_LEVELS = [0.10, 0.12, 0.15, 0.18, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50]

PRE_SPIN_THETA = 0.80     # 降速路径的冲刺油门
PRE_SPIN_V = 25.0         # 冲到这么快就停（远高于狩猎带；够不到就跑满步数）
PRE_SPIN_STEPS = 300      # 冲刺上限 (30 s)

# ---- 分类阈值（全部写进 balance_notes.json）----
DEAD_V = 0.05             # 窗口内最高速都低于此值 -> 死区
A_DRIFT_TOL = 0.02        # 净漂移率上限 (m/s²)：超了就是"还在变"
A_ABS_TOL = 0.15          # 窗口内 |a| 均值上限 (m/s²)：超了就是抖动/狩猎
V_STD_TOL = 0.15          # 窗口内 v 标准差上限 (m/s)
CROSS_BAND = 0.05         # 穿越计的**死区带** (m/s)，见 _crossings()
# ↑ 判据为什么是"两个幅度量"（|a|均值 + v 标准差）而不是"穿越次数"：
#   穿越次数对**幅度**不敏感 —— 幅度 0.001 m/s 的数值抖动和幅度 2 m/s 的狩猎
#   都是"反复穿越均值"，只有带上死区带才分得开（本次实测踩过一次）。
#   而 |a| 均值和 v 标准差直接就是幅度，物理含义清楚，不需要调死区带。
#   A_ABS_TOL=0.15 与 06 的 CY_ABS_A 取同一个值 —— 两处判据必须同源，
#   否则跨脚本比较又会变成 P-SIM-02。
#   ⚠ 已知灰区：实测 tesla θ=0.30 的 |a| 均值 = 0.122、θ=0.35 的 = 0.136，
#   紧贴 0.15 下沿（与 06 里 patrol@0.40=0.114 同一片灰区）。灰区**照实报**，
#   不再为了"分干净"去调阈值 —— 调阈值只是把不确定性藏起来。

FIELDS = ["mode", "throttle", "step", "t", "v", "a"]


# ------------------------- 一次运行 -------------------------
def run_balance(world, bp, spawn_pt, theta, mode, warmup, window,
                pre_spin_theta=PRE_SPIN_THETA, pre_spin_v=PRE_SPIN_V,
                pre_spin_steps=PRE_SPIN_STEPS, v_max=40.0):
    """一次平衡点测量。mode: "up"（从静止）| "down"（先冲高速再阶跃降到 theta）。

    返回 (rows, note)。rows 里含 warmup 段（落在 CSV 里可回看），
    但统计只用最后 window 拍 —— 由 window_stats() 决定。
    """
    vehicle = world.try_spawn_actor(bp, spawn_pt)
    if vehicle is None:
        return None, "spawn 失败（出生点被占）"
    rows, speeds = [], []
    try:
        if not settle(world, vehicle):
            return None, "settle 失败（车没停稳）"
        home = vehicle.get_transform()

        # ---- 降速路径：先冲刺，制造"从高速过来"的初条件 ----
        spin = 0
        if mode == "down":
            for _ in range(pre_spin_steps):
                spin += 1
                if tick_pinned(world, vehicle, home, pre_spin_theta) >= pre_spin_v:
                    break
            speeds.append(speed_of(vehicle))     # 阶跃瞬间记起点，第一步才有真 a
        else:
            speeds.append(speed_of(vehicle))

        total = warmup + window
        for i in range(total):
            v = tick_pinned(world, vehicle, home, theta)
            a = (v - speeds[-1]) / DT
            speeds.append(v)
            rows.append({"mode": mode, "throttle": round(theta, 3), "step": i,
                         "t": round(i * DT, 3), "v": round(v, 5), "a": round(a, 5)})
            if v >= v_max:
                return rows, f"撞 v_max={v_max}（{i} 拍），该档未到平衡"
        return rows, f"ok（冲刺 {spin} 拍 + 测量 {total} 拍）"
    finally:
        vehicle.destroy()
        world.tick()


# ------------------------- 统计与分类 -------------------------
def _crossings(seg, ref, band=CROSS_BAND):
    """seg 穿越 ref 的次数，**带死区带**：跑出 [ref±band] 再折返才算一次。

    ★ 死区带不是可选项：不带的话，幅度 0.001 m/s 的数值抖动每 1.5 拍就穿一次均值，
    300 拍能数出 196 次"穿越"，把 θ=0.50 这种 v_std=0.001 的干净平衡点
    判成极限环（本次实测踩到）。
    真狩猎的幅度是 ±0.5~2.5 m/s，0.05 的死区带对它完全没有影响。
    """
    n, state = 0, 0
    for x in seg:
        if x > ref + band:
            s = 1
        elif x < ref - band:
            s = -1
        else:
            continue          # 带内不动，保持上一状态（这就是死区）
        if state and s != state:
            n += 1
        state = s
    return n


def window_stats(rows, window=WINDOW):
    """只看最后 window 拍的**区间统计量**。

    ★ 这是本脚本与 03 的分界线：03 问"速度还变不变"（阈值定义的量），
      这里问"这段过程的均值/方差/漂移是多少"（过程的统计量）。
    """
    # 注意：rows 可能是**内存里的 dict（跑完直接统计）**，也可能是**CSV 读回来的 str**。
    # 两条路径都走这里，所以一律 float() 一次，别假设调用方已经转好类型。
    v = [float(r["v"]) for r in rows]
    a = [float(r["a"]) for r in rows]
    n = min(window, len(v))
    sv, sa = v[-n:], a[-n:]
    v_mean = st.mean(sv)
    v_std = st.pstdev(sv) if n > 1 else 0.0
    # 净漂移率：前后各 1/3 的均值差 / 中间间隔时间。用三分之一均值而不是首尾两点，
    # 是为了让振荡自己抵消掉（首尾两点会被单拍噪声主导）。
    k = max(1, n // 3)
    drift = (st.mean(sv[-k:]) - st.mean(sv[:k])) / ((n - k) * DT) if n > k else 0.0
    return {"n": n, "v_mean": v_mean, "v_std": v_std, "v_min": min(sv),
            "v_max": max(sv), "a_abs_mean": st.mean([abs(x) for x in sa]),
            "a_mean": st.mean(sa), "drift": drift,
            "cross": _crossings(sv, v_mean)}


def classify(s):
    """给统计量贴标签。阈值见文件头，全部落进 notes.json。"""
    if s["v_max"] < DEAD_V:
        return "死区"
    if abs(s["drift"]) > A_DRIFT_TOL:
        return "未达平衡"
    if s["a_abs_mean"] > A_ABS_TOL or s["v_std"] > V_STD_TOL:
        return "极限环"
    return "稳定平衡"


# ------------------------- 落盘 / 汇总 -------------------------
def write_run(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
        f.flush()


def collect(out_dir, levels, modes, window=WINDOW):
    """读 low_runs/*.csv -> {(theta, mode): stats}。曲线是唯一数据源。"""
    tbl = {}
    for theta in levels:
        for mode in modes:
            p = os.path.join(out_dir, "low_runs", f"bal_{mode}_{theta:.2f}.csv")
            if not os.path.exists(p):
                continue
            rows = list(csv.DictReader(open(p, encoding="utf-8")))
            if not rows:
                continue
            s = window_stats(rows, window)
            s["cls"] = classify(s)
            tbl[(theta, mode)] = s
    return tbl


def report(tbl, levels, modes, out_dir, warmup=WARMUP, window=WINDOW):
    """对照表：升/降两条路径并排 + 迟滞差。"""
    print("\n" + "=" * 96)
    print("低速段平衡点扫描   （固定时长 %d+%d 拍，统计最后 %d 拍）"
          % (warmup, window, window))
    print("  up   = 从静止起，theta 直接加载")
    print("  down = 先 %.2f 油门冲到目标速度，再阶跃降到 theta" % PRE_SPIN_THETA)
    print("=" * 96)
    print(f"{'theta':>6} | {'mode':>4} | {'v_mean':>8} | {'v_std':>7} | "
          f"{'漂移':>8} | {'穿越':>5} | {'|a|均':>7} | 分类")
    print("-" * 96)
    rows_out = []
    for theta in levels:
        got = [m for m in modes if (theta, m) in tbl]
        for mode in got:
            s = tbl[(theta, mode)]
            print(f"{theta:6.2f} | {mode:>4} | {s['v_mean']:8.3f} | {s['v_std']:7.3f} | "
                  f"{s['drift']:+8.4f} | {s['cross']:5d} | {s['a_abs_mean']:7.3f} | {s['cls']}")
        if len(got) == 2:      # 迟滞差
            dv = tbl[(theta, "down")]["v_mean"] - tbl[(theta, "up")]["v_mean"]
            tag = "★ 迟滞" if abs(dv) > max(0.5, 0.1 * abs(tbl[(theta, "up")]["v_mean"])) else ""
            print(f"{'':6} | {'Δ':>4} | {dv:+8.3f}  (down - up) {tag}")
            rows_out.append({"throttle": theta, "v_up": tbl[(theta, "up")]["v_mean"],
                             "cls_up": tbl[(theta, "up")]["cls"],
                             "v_down": tbl[(theta, "down")]["v_mean"],
                             "cls_down": tbl[(theta, "down")]["cls"],
                             "dv_hysteresis": dv})
        print("-" * 96)

    p = os.path.join(out_dir, "balance_table.csv")
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["throttle", "mode", "v_mean", "v_std", "v_min", "v_max",
                    "drift_mps2", "cross", "a_abs_mean", "a_mean", "class", "n_ticks"])
        for theta in levels:
            for mode in modes:
                if (theta, mode) not in tbl:
                    continue
                s = tbl[(theta, mode)]
                w.writerow([f"{theta:.2f}", mode, f"{s['v_mean']:.4f}", f"{s['v_std']:.4f}",
                            f"{s['v_min']:.4f}", f"{s['v_max']:.4f}", f"{s['drift']:.5f}",
                            s["cross"], f"{s['a_abs_mean']:.4f}", f"{s['a_mean']:.4f}",
                            s["cls"], s["n"]])
    print(f"[out] {p}")

    # ★ 判据必须跟结果一起交付（S6）
    nt = os.path.join(out_dir, "balance_notes.json")
    with open(nt, "w", encoding="utf-8") as f:
        json.dump({
            "what": "低速段（v<6 m/s）平衡点扫描：固定时长 + 区间统计量 + 迟滞对照",
            "why": "03 的『首尾速度差<0.05 即稳态』在换挡狩猎段失效：振荡被平均掉，"
                   "极限环被判成稳态（carlacola @0.60 跑了 920 拍，a 在 ±0.5~±2.5 交替）",
            "method": {
                "warmup_ticks": WARMUP, "window_ticks": WINDOW, "dt": DT,
                "up": "从静止起，theta 直接加载",
                "down": f"先冲刺 {PRE_SPIN_THETA} 到 {PRE_SPIN_V} m/s，再阶跃降到 theta",
                "pre_spin_steps_max": PRE_SPIN_STEPS,
            },
            "criteria": {
                "死区": f"窗内 v_max < {DEAD_V} m/s",
                "未达平衡": f"|净漂移率| > {A_DRIFT_TOL} m/s²",
                "极限环": f"|a| 均值 > {A_ABS_TOL} m/s² 或 v_std > {V_STD_TOL} m/s",
                "稳定平衡": "以上都不满足",
                "_为什么用幅度而不用穿越次数":
                    "穿越次数对幅度不敏感：幅度 0.001 m/s 的数值抖动和幅度 2 m/s 的"
                    "狩猎都是『反复穿越均值』。实测 θ=0.50 的 v_std=0.001 却被数出 "
                    "196 次穿越 → 判成极限环。|a|均值与 v_std 本身就是幅度，无需调参。",
                "_灰区（照实报，不调阈值遮盖）":
                    f"tesla θ=0.30 |a|均值=0.122、θ=0.35 =0.136，紧贴 {A_ABS_TOL} 下沿",
                "_与 06 同源": f"A_ABS_TOL={A_ABS_TOL} 即 06 的 CY_ABS_A，"
                              "两处判据必须同一个值，否则跨脚本比较会变成 P-SIM-02",
            },
            "boundaries": [
                "只覆盖恒定油门下的平衡行为，不含从静止起步的瞬态",
                "『稳定平衡』≠ 无振荡，只是振荡幅度低于阈值",
                "降速路径的初条件由冲刺油门决定，未扫多个初速",
                "单位 m/s、m/s²；采样 0.1 s，与 acc_sim 的 AccConfig.dt 对齐",
            ],
        }, f, ensure_ascii=False, indent=2)
    print(f"[out] {nt}")
    return rows_out


def plot(tbl, levels, modes, out_png, vehicle):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.2))
    style = {"稳定平衡": ("o", "-"), "极限环": ("x", None), "未达平衡": (">", None),
             "死区": ("s", None)}

    for mode, color in (("up", "tab:blue"), ("down", "tab:red")):
        pts = [(t, tbl[(t, mode)]) for t in levels if (t, mode) in tbl]
        if not pts:
            continue
        ax1.plot([t for t, _ in pts], [s["v_mean"] for _, s in pts],
                 "o-", color=color, lw=1.6, ms=5, label=f"{mode} 路径")
        for t, s in pts:      # 非稳定平衡点单独标出来
            mk = style.get(s["cls"], ("?", None))[0]
            if s["cls"] != "稳定平衡":
                ax1.plot([t], [s["v_mean"]], mk, ms=10, mew=2.0, color=color)
    ax1.axhspan(0, 6, color="orange", alpha=0.10)
    ax1.text(0.105, 6.2, "v < 6 m/s：03 测不准的区间", color="darkorange", fontsize=9)
    ax1.set_xlabel("throttle  θ")
    ax1.set_ylabel("平衡速度 v_mean  (m/s)")
    ax1.set_title("迟滞对照：升速路径 vs 降速路径\n"
                  "(两条线重合 = 稳定吸引子；分开 = 迟滞/多稳态)")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9)

    for mode, color in (("up", "tab:blue"), ("down", "tab:red")):
        pts = [(t, tbl[(t, mode)]) for t in levels if (t, mode) in tbl]
        if not pts:
            continue
        ax2.plot([t for t, _ in pts], [s["v_std"] for _, s in pts],
                 "o-", color=color, lw=1.6, ms=5, label=f"{mode}")
    ax2.axhline(V_STD_TOL, color="gray", ls="--", lw=1.0)
    ax2.text(0.105, V_STD_TOL * 1.15, f"极限环阈值 {V_STD_TOL} m/s", color="gray", fontsize=9)
    ax2.set_xlabel("throttle  θ")
    ax2.set_ylabel("窗口内 v 标准差  (m/s)")
    ax2.set_title("振荡强度：这就是 03 的判据看不见的东西\n"
                  "(标准差大 = 均值背后在来回穿，不是平衡点)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=9)

    fig.suptitle(f"{vehicle}  低速段平衡点扫描", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))   # 给 suptitle 留位置，否则告警
    fig.savefig(out_png, dpi=130)
    print(f"[out] {out_png}")


# ------------------------- main -------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="vehicle.tesla.model3",
                    help="车型 blueprint id（标定是车型专属的，换车必须重标）")
    ap.add_argument("--levels", default="", help='只跑指定档，如 "0.10,0.15,0.20"')
    ap.add_argument("--modes", default="up,down", help='"up" / "down" / "up,down"')
    ap.add_argument("--warmup", type=int, default=WARMUP)
    ap.add_argument("--window", type=int, default=WINDOW)
    ap.add_argument("--force", action="store_true", help="忽略已有曲线，重跑")
    ap.add_argument("--skip-run", action="store_true", help="只汇总，不跑仿真")
    ap.add_argument("--quick", action="store_true", help="冒烟：档位砍半、时长砍半")
    ap.add_argument("--pre-spin-v", type=float, default=PRE_SPIN_V,
                    help="降速路径的冲刺目标速度 (m/s)。★ 扫它 = 扫迟滞环的初条件："
                         "落点随初速连续变 -> 没有唯一平衡点（踏板映射不可逆）；"
                         "落点离散跳变 -> 是挡位/状态分支")
    ap.add_argument("--pre-spin-theta", type=float, default=PRE_SPIN_THETA)
    ap.add_argument("--pre-spin-steps", type=int, default=PRE_SPIN_STEPS)
    ap.add_argument("--spawn-idx", type=int, default=2)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    levels = [float(x) for x in args.levels.split(",")] if args.levels else LOW_LEVELS
    modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    warmup, window = args.warmup, args.window
    if args.quick:
        warmup, window = max(60, warmup // 4), max(60, window // 4)
        if not args.levels:
            levels = levels[::4]
    out_dir = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "calib", "veh_" + slug(args.vehicle))
    os.makedirs(os.path.join(out_dir, "low_runs"), exist_ok=True)

    if not args.skip_run:
        world, map_name = ensure_carla()
        print(f"[carla] map = {map_name}（★ 不切图：load_world 第二次必崩，P-SIM-01）")
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        world.apply_settings(settings)
        try:
            try:
                bp = world.get_blueprint_library().find(args.vehicle)
            except IndexError:
                print(f"[carla] ❌ 找不到车型 {args.vehicle}")
                return 1
            spawn_pt = world.get_map().get_spawn_points()[args.spawn_idx %
                                                          len(world.get_map().get_spawn_points())]
            n = len(levels) * len(modes)
            k = 0
            t_start = time.time()
            for theta in levels:
                for mode in modes:
                    k += 1
                    rf = os.path.join(out_dir, "low_runs", f"bal_{mode}_{theta:.2f}.csv")
                    if os.path.exists(rf) and not args.force:
                        print(f"[{k}/{n}] θ={theta:.2f} {mode:<4} 已有曲线，跳过")
                        continue
                    t0 = time.time()
                    rows, note = run_balance(world, bp, spawn_pt, theta, mode,
                                             warmup, window,
                                             pre_spin_theta=args.pre_spin_theta,
                                             pre_spin_v=args.pre_spin_v,
                                             pre_spin_steps=args.pre_spin_steps)
                    if not rows:
                        print(f"[{k}/{n}] θ={theta:.2f} {mode:<4} ❌ {note}")
                        continue
                    write_run(rf, rows)
                    s = window_stats(rows, window)
                    print(f"[{k}/{n}] θ={theta:.2f} {mode:<4} -> {len(rows)} 拍 "
                          f"({time.time()-t0:.0f}s)  v_mean={s['v_mean']:7.3f} "
                          f"σ={s['v_std']:6.3f} 漂移={s['drift']:+.4f} "
                          f"穿越={s['cross']:4d}  {classify(s)}   [{note}]")
            print(f"\n[07] 本轮用时 {time.time()-t_start:.0f}s")
        finally:
            try:
                settings.synchronous_mode = False
                world.apply_settings(settings)
                print("[carla] 同步模式已关闭。")
            except Exception as e:
                print(f"[carla] ⚠ 收尾失败（服务端可能已崩）：{e}")

    tbl = collect(out_dir, levels, modes, window)
    if not tbl:
        print("[07] 没有拿到任何曲线")
        return 1
    report(tbl, levels, modes, out_dir, warmup, window)
    plot(tbl, levels, modes, os.path.join(out_dir, "hysteresis.png"), args.vehicle)
    return 0


if __name__ == "__main__":
    sys.exit(main())
