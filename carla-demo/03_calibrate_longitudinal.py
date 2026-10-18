"""03_calibrate_longitudinal.py -- CARLA 车辆纵向动力学标定（跑步机法）。

背景
----
02_spawn_vehicle.py 实测发现：恒定油门 0.5 下，车速 1 m/s 时加速度约 1.85 m/s^2，
13 m/s 时只剩 0.085 m/s^2 -- 差 20 倍。油门是"力"，阻力随速度增长，二者趋于平衡。

而 acc_sim 的 _map_pedals 假设 throttle = accel / accel_max 是线性的，
直接接进 CARLA 会"想加速却加不动"。本脚本就是注释里说的"正式用标定量替换"。

跑步机法（treadmill）—— 本版核心
--------------------------------
每个 tick 跑完物理后，把车 transform 拉回出发点（set_transform 保留速度），
车在世界上位置不动、只让速度积分。

为什么值得：开车法（让车真的在路上跑）踩了一串坑，全部消失。
  - 不用 lane_keep：位置不动，无需转向
  - 不会撞车/掉出路沿：实测开车法 theta=0.8/0.9/0.7 都在高速段冲出车道，
    加速度出现 a=-8 ~ -12 m/s^2 的乱跳（全油门不可能有这种减速度）
  - 不用挑地图/挑出生点：Town10HD 城区也能用，不再是瓶颈
有效性已验证：同一档位下跑步机与自由行驶的速度曲线**小数点后三位吻合**
（t=1.0s 2.601 vs 2.589；t=6.0s 9.742 vs 9.743），且 teleport 前后 v 连续。

开车法踩过的坑（保留在案，别再走回头路）
--------------------------------------
① steer=0 直线开 -> 路一弯就压护栏，90~140 m 必撞 static.guardrail
② spawn 从出生点高度落下 -> 每档 step0 都读到假初速 v≈0.98 m/s，要先刹车 settle
③ CARLA 服务端二次 load_world 会 OOM 崩进程（16 GB 机器）
④ 验收判据连踩三次假阳性：|a|>3 把正常加速（6~7 m/s^2）全判死；
   加速度符号翻转在趋近稳态时被噪声触发；只看"窗口首尾速度差"会把震荡平均成稳态

换成跑步机法后又踩两个（同一类错误：判据比物理过程还快）
------------------------------------------------------
⑤ settle 一见 v<0.05 就返回 —— spawn 那一瞬车在半空、v 本来就是 0，
   于是 transform 被抓在半空中，整条曲线建在悬空车上，油门加不动。
   症状极隐蔽：theta=0.15 只爬到 0.13 m/s 就冻住（正常应 2.0 m/s）。
   -> settle 必须跑满固定步数再判，不看瞬时值。
⑥ 稳态判据在"起步蓄力期"提前触发 —— 低油门起步有 3~4 s 蓄力期，
   期间 v 平得跟稳态一样，i=30 就把 0.1326 m/s 判成"稳态"。
   症状：theta=0.10 与 0.15 的 trace **逐位相同**（不同输入不可能同曲线）。
   -> 加 STEADY_MIN_TICKS=80 下限。

判决实验见 04_diag_low_throttle.py：pin+theta=0 全程精确 0 速度
（跑步机无伪影）；pin 与 free 在 theta=0.15 下曲线一致（峰值 2.04 vs 2.24）。

产出（分档落盘，支持断点续跑）
------------------------------
  <out>/runs/hold_<theta>.csv   每档一个文件（已是最终格式，崩了不丢已完成档）
  <out>/runs/coast_0.00.csv
  <out>/meta.json               车型身份（标定表是车型专属的，见 write_meta）
  <out>/summary.csv             汇总
  <out>/throttle_accel_map.png  一族 v-a 曲线 + 阻力曲线

车型（--vehicle）
----------------
默认 vehicle.tesla.model3。**换车型必须重标**：质量/扭矩曲线/风阻/制动全变，
F(θ)、G(v)、B(β) 三项无一同源。用 A 车的表开 B 车不会报错，只会让 ACC
表现微妙地不对（现象指向算法、根因在标定）。

⚠ 不要用 client.load_world() 切地图：实测第二次调用必崩服务端（踩坑③）。
  服务端**启动时是什么图就用什么图**；要换图请重启 CARLA 进程。

跑法（必须用全路径 python + utf-8）：
  PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe 03_calibrate_longitudinal.py
  ... --vehicle vehicle.audi.a2 --out calib/veh_audi_a2 --no-coast --no-brake
"""

import argparse
import csv
import glob
import os

import carla
import matplotlib

# 跑步机原语（speed_of / settle / tick_pinned）与 CARLA 服务端进程管理都在这里，
# 03/06/07 共用一份 —— 见 carla_env.py 顶部注释。
from carla_env import settle, speed_of, tick_pinned  # noqa: E402

matplotlib.use("Agg")  # 无窗口后端，只出文件
import matplotlib.pyplot as plt

# ------------------------- 实验参数 -------------------------
# 0.10~0.20 加密：低速段油门响应有 3~4 s 蓄力期，死区边界要看清
THROTTLE_LEVELS = [0.10, 0.12, 0.15, 0.20, 0.30, 0.40, 0.50,
                   0.60, 0.70, 0.80, 0.90, 1.00]
DT = 0.1            # 固定步长，必须对齐 AccConfig.dt
SMOOTH_WIN = 5      # 滑动平均窗口（tick）
STEADY_TOL = 0.05   # 稳态判据：窗口首尾速度差小于此值 (m/s)
STEADY_WIN = 30     # 稳态判定窗口 (tick) = 3 s
STEADY_A_MAX = 1.0  # 判"稳态"时窗口内 |a| 上限，防抖动被平均掉
STEADY_MIN_TICKS = 80    # 判定稳态前至少跑够的 tick 数 = 8 s
# ↑ 必须有这条下限：低油门起步有 3~4 s 蓄力期，期间 v 平得像稳态，
#   只看窗口首尾速度差会在 i=30 把"还没起步"误判成"已到稳态"（踩过）。
DEAD_V = 0.05       # 低于此速度视为"车没动"（油门死区）
COAST_V0 = 40.0     # coast 阶段先加速到的速度 (m/s)
# ↑ 必须覆盖到 40：只测到 12 m/s 的话，12 以上没有独立的阻力测量，
#   阻力系数只能靠"假设油门线性"反推，F 的非线性会被塞进 v² 项而变成负值。
COAST_FLOOR = 2.0   # coast 阶段滑行到此速度即停 (m/s)
# 刹车扫描：每个 (beta, v0) 组合一条曲线。刹车能力随 v 变，单点测不出曲线，
# 所以分成几个起始速度各跑一遍。
BRAKE_LEVELS = [0.20, 0.40, 0.60, 0.80, 1.00]
BRAKE_V0S = [10.0, 20.0, 30.0, 40.0]
BRAKE_FLOOR = 1.0   # 减到此速度即停（再低会撞上低速段的分段效应）
FIELDS = ["phase", "throttle", "brake", "step", "t", "v", "a", "a_smooth", "dist"]

HOST, PORT = "localhost", 2000


def run_hold(world, bp, spawn_pt, theta, max_steps, v_max):
    """恒定油门从静止跑到稳态 / v_max。返回 (rows, v_ss, note)。"""
    vehicle = world.try_spawn_actor(bp, spawn_pt)
    if vehicle is None:
        return None, 0.0, "spawn 失败（出生点被占）"
    rows, speeds = [], []
    try:
        if not settle(world, vehicle):
            return None, 0.0, "settle 失败（车没停稳）"
        home = vehicle.get_transform()

        for i in range(max_steps):
            v = tick_pinned(world, vehicle, home, theta)
            a = (v - speeds[-1]) / DT if speeds else 0.0
            speeds.append(v)
            rows.append({"phase": "hold", "throttle": theta, "brake": 0.0, "step": i,
                         "t": round(i * DT, 3), "v": round(v, 4), "a": round(a, 4),
                         "a_smooth": 0.0, "dist": 0.0})

            if v >= v_max:
                return rows, v, f"达到 v_max={v_max}（未到稳态）"
            if i >= STEADY_MIN_TICKS and abs(speeds[-1] - speeds[-STEADY_WIN]) < STEADY_TOL:
                win_a = [abs(speeds[j] - speeds[j - 1]) / DT
                         for j in range(i - STEADY_WIN + 1, i + 1)]
                if max(win_a) > STEADY_A_MAX:
                    continue            # 窗口内还在抖，不算稳态
                if v < DEAD_V:
                    return rows, 0.0, "油门死区（车全程未起步）"
                return rows, v, "稳态"
        return rows, speeds[-1], "步数用尽（未到稳态）"
    finally:
        vehicle.destroy()
        world.tick()


def run_coast(world, bp, spawn_pt, v0, max_steps):
    """加速到 v0 后松油门自由滑行，测纯阻力曲线。"""
    vehicle = world.try_spawn_actor(bp, spawn_pt)
    if vehicle is None:
        return None, "spawn 失败"
    rows, speeds = [], []
    try:
        if not settle(world, vehicle):
            return None, "settle 失败"
        home = vehicle.get_transform()
        for _ in range(max_steps):                    # 先加速到 v0
            # 全油门：theta=0.5 稳态只有 13.9 m/s，够不到 v0=40
            if tick_pinned(world, vehicle, home, 1.0) >= v0:
                break
        else:
            return None, f"加不到 v0={v0}（max_steps 用尽）"
        # 记下松油门瞬间的速度，让滑行第一步就有真实 a（否则 step0 只能是构造性的 0，
        # 画在图上就是 v=v0 处一个假零点 —— "油门=0 却在加速"的鬼影）
        speeds.append(speed_of(vehicle))
        for i in range(max_steps):                    # 松油门滑行
            v = tick_pinned(world, vehicle, home, 0.0)
            a = (v - speeds[-1]) / DT
            speeds.append(v)
            rows.append({"phase": "coast", "throttle": 0.0, "brake": 0.0, "step": i,
                         "t": round(i * DT, 3), "v": round(v, 4), "a": round(a, 4),
                         "a_smooth": 0.0, "dist": 0.0})
            if v < COAST_FLOOR:
                break
        return rows, "ok"
    finally:
        vehicle.destroy()
        world.tick()


def run_brake(world, bp, spawn_pt, beta, v0, max_steps):
    """先加速到 v0，再施加恒定刹车 beta，记录减速曲线到底（v < BRAKE_FLOOR）。

    与油门扫描对称：油门给 F(theta)，刹车给 B(beta)。ACC 跟车必然要刹车，
    只标油门是不够的。
    必须分多个起始速度各跑一遍：刹车能力本身随 v 变，单点测不出曲线。
    """
    vehicle = world.try_spawn_actor(bp, spawn_pt)
    if vehicle is None:
        return None, "spawn 失败"
    rows, speeds = [], []
    try:
        if not settle(world, vehicle):
            return None, "settle 失败"
        home = vehicle.get_transform()
        for _ in range(max_steps):                    # 加速到 v0
            # 必须用全油门：theta=0.6 的稳态只有 20 m/s，拿它去够 v0=30/40 永远够不到
            if tick_pinned(world, vehicle, home, 1.0) >= v0:
                break
        else:
            return None, f"加不到 v0={v0}（max_steps 用尽）"
        speeds.append(speed_of(vehicle))
        for i in range(max_steps):
            v = tick_pinned(world, vehicle, home, 0.0, brake=beta)
            a = (v - speeds[-1]) / DT
            speeds.append(v)
            rows.append({"phase": "brake", "throttle": 0.0, "brake": beta, "step": i,
                         "t": round(i * DT, 3), "v": round(v, 4), "a": round(a, 4),
                         "a_smooth": 0.0, "dist": 0.0})
            if v < BRAKE_FLOOR:
                break
        return rows, "ok"
    finally:
        vehicle.destroy()
        world.tick()


def smooth(rows, win=SMOOTH_WIN):
    """对 a 做滑动平均，压掉换挡跳变。就地写回 a_smooth 并返回 rows。"""
    for i, r in enumerate(rows):
        lo = max(0, i - win + 1)
        seg = [x["a"] for x in rows[lo:i + 1]]
        r["a_smooth"] = round(sum(seg) / len(seg), 4)
    return rows


def write_run(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
        f.flush()


def load_acc_dt():
    """读 acc_sim 的 AccConfig.dt，确认与标定步长一致（不 import，避免路径耦合）。"""
    import re
    p = os.path.join(os.path.dirname(__file__), "..", "acc_sim", "acc", "models.py")
    try:
        txt = open(p, encoding="utf-8").read()
        return float(re.search(r"dt:\s*float\s*=\s*([\d.]+)", txt).group(1))
    except Exception:
        return None


def write_meta(out_dir, vehicle, bp, map_name):
    """落一份 meta.json：车型身份 + 可用蓝图属性。

    为什么必须有：标定表是**车型专属**的（质量/扭矩曲线/风阻/制动随车全变），
    拿 A 车的表去开 B 车不会报错，只会让 ACC 表现微妙地不对 —— 现象指向算法、
    根因在标定，极难查。所以 vehicle 字段是**防错的第一道**（不是唯一一道）。

    ⚠ 曾在此写过"蓝图不暴露物理参数 → 无法自动比对"——**只对 blueprint attribute 成立**。
    实测 `bp.get_attribute('mass')` 确实抛异常，但车辆 spawn 之后
    `vehicle.get_physics_control()` 能读出 mass / drag / 扭矩曲线 / 逐挡传动比，
    `apply_physics_control()` 还能写回去 —— 所以**可以**在加载时核对。
    见 `08_probe_physics.py`（探针）与 playbook 第三节。
    """
    import json
    attrs = sorted(a.id for a in bp)      # 蓝图属性（不含物理参数，那些走 get_physics_control）
    try:
        ver = carla.__version__
    except Exception:
        ver = "unknown"
    meta = {"vehicle": vehicle, "carla_version": ver, "map": map_name, "dt": DT,
            "blueprint_attrs": attrs,
            "note": "blueprint attributes do NOT include physics params in CARLA 0.9.15; "
                    "read them via vehicle.get_physics_control() (see 08_probe_physics.py). "
                    "vehicle id alone is NOT the only guard."}
    p = os.path.join(out_dir, "meta.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[out] {p}")
    return meta


def plot(runs_dir, out_png, map_name, vehicle="vehicle.tesla.model3"):
    """读 runs/*.csv 出图。"""
    files = sorted(glob.glob(os.path.join(runs_dir, "*.csv")))
    if not files:
        print("[plot] runs/ 里没有数据，跳过出图")
        return
    fig, ax = plt.subplots(figsize=(9, 6))
    holds = []
    brake_seen = [False]      # 用一个单元素列表当可变标志位（闭包外可见）
    for fp in files:
        rows = list(csv.DictReader(open(fp, encoding="utf-8")))
        if not rows:
            continue
        phase, theta = rows[0]["phase"], float(rows[0]["throttle"])
        xs = [float(r["v"]) for r in rows]
        ys = [float(r["a_smooth"]) for r in rows]
        if phase == "coast":
            ax.plot(xs, ys, "k--", lw=1.8, label="coast (throttle=0)")
        elif phase == "brake":
            # 刹车曲线有 20 条（5 档 beta x 4 个起始速度），全进图例会淹没图。
            # 只画细虚线、共用一个图例条目，具体数值留给拟合脚本报。
            ax.plot(xs, ys, ":", color="crimson", lw=1.0, alpha=0.55,
                    label="brake sweep (5 beta x 4 v0)" if not brake_seen[0] else None)
            brake_seen[0] = True
        else:
            holds.append(theta)
            ax.plot(xs, ys, lw=1.6, label=f"throttle = {theta:.2f}")
    # 对照：acc_sim 的线性假设 throttle = accel / 2.0 -> a = 2.0 * throttle
    for theta in holds:
        ax.plot([0, 45], [0, 2.0 * theta], color="gray", lw=0.7, alpha=0.3, zorder=0)

    ax.axhline(0.0, color="k", lw=0.8, alpha=0.6)
    ax.set_xlabel("speed v  (m/s)")
    ax.set_ylabel("acceleration a  (m/s^2)")
    ax.set_title(f"{vehicle} longitudinal dynamics:  a = f(v, throttle)   [{map_name}]\n"
                 "(treadmill method, 0.1 s step; gray = acc_sim linear assumption a = 2.0*throttle)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    ax.set_xlim(left=0)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"[out] {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="", help="需要时切地图（默认不切）")
    ap.add_argument("--levels", default="", help='只跑指定档位，如 "0.3,0.6,1.0"')
    ap.add_argument("--force", action="store_true", help="忽略已有结果，全部重跑")
    ap.add_argument("--max-steps", type=int, default=800, help="每档最长 tick 数")
    ap.add_argument("--v-max", type=float, default=40.0, help="超过此速度即停止该档")
    ap.add_argument("--spawn-idx", type=int, default=2, help="用哪个出生点（跑步机法只要一块平地）")
    ap.add_argument("--quick", action="store_true", help="冒烟：只跑 3 档")
    ap.add_argument("--no-coast", action="store_true", help="跳过阻力曲线")
    ap.add_argument("--no-brake", action="store_true", help="跳过刹车扫描")
    ap.add_argument("--brake-only", action="store_true", help="只跑刹车扫描")
    ap.add_argument("--plot-only", action="store_true", help="只出图，不跑仿真")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "calib"))
    ap.add_argument("--vehicle", default="vehicle.tesla.model3",
                    help="车型 blueprint id（纵向特性随车型变，标定表不可跨车混用）")
    args = ap.parse_args()

    if args.levels:
        levels = [float(x) for x in args.levels.split(",")]
    elif args.quick:
        levels = [0.30, 0.60, 1.00]
    else:
        levels = THROTTLE_LEVELS
    if args.brake_only:
        levels = []                       # 空列表 = 那个 for 循环自然不跑
    max_steps = 200 if args.quick else args.max_steps

    runs_dir = os.path.join(args.out, "runs")
    os.makedirs(runs_dir, exist_ok=True)
    png_path = os.path.join(args.out, "throttle_accel_map.png")

    if args.plot_only:
        plot(runs_dir, png_path, args.map or "current", args.vehicle)
        return

    acc_dt = load_acc_dt()
    print(f"[cfg] 标定 dt = {DT}s | acc_sim AccConfig.dt = {acc_dt}")
    if acc_dt is not None and abs(acc_dt - DT) > 1e-9:
        print(f"[warn] dt 不一致！标定 {DT} vs ACC {acc_dt}，标定量不可直接使用。")

    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)
    if args.map:
        cur = client.get_world().get_map().name
        world = client.get_world() if args.map in cur else client.load_world(args.map)
    else:
        world = client.get_world()
    map_name = world.get_map().name
    print(f"[carla] 已连接 {HOST}:{PORT} | map = {map_name}")

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DT
    world.apply_settings(settings)

    try:
        bp = world.get_blueprint_library().find(args.vehicle)
    except IndexError:
        print(f"[carla] ❌ 找不到车型 {args.vehicle}")
        return
    spawn_points = world.get_map().get_spawn_points()
    spawn_pt = spawn_points[args.spawn_idx % len(spawn_points)]
    print(f"[carla] 车型 = {args.vehicle} | 出生点 #{args.spawn_idx}"
          f"（跑步机法只需一块平地，不挑直道）")
    write_meta(args.out, args.vehicle, bp, map_name)

    summary = []
    try:
        for k, theta in enumerate(levels, 1):
            rf = os.path.join(runs_dir, f"hold_{theta:.2f}.csv")
            if os.path.exists(rf) and not args.force:
                print(f"[{k}/{len(levels)}] throttle={theta:.2f}  已有结果，跳过")
                continue

            rows, v_ss, note = run_hold(world, bp, spawn_pt, theta, max_steps, args.v_max)
            if not rows:
                print(f"[{k}/{len(levels)}] throttle={theta:.2f}  ❌ {note}")
                summary.append({"phase": "hold", "throttle": theta, "v": 0.0,
                                "status": "FAILED", "note": note})
                continue

            smooth(rows)
            write_run(rf, rows)      # ← 立刻落盘
            print(f"[{k}/{len(levels)}] throttle={theta:.2f}  -> {len(rows)} tick, "
                  f"v_ss={v_ss:.2f} m/s  ({note})")
            summary.append({"phase": "hold_ss", "throttle": theta, "v": round(v_ss, 3),
                            "status": "OK" if note != "油门死区（车全程未起步）" else "DEADZONE",
                            "note": note})

        # ---- coast：松油门测阻力 ----
        cf = os.path.join(runs_dir, "coast_0.00.csv")
        if not args.brake_only and not args.no_coast \
                and not (os.path.exists(cf) and not args.force):
            crows, cnote = run_coast(world, bp, spawn_pt, COAST_V0, max_steps)
            if crows:
                smooth(crows)
                write_run(cf, crows)
                print(f"[coast] {len(crows)} tick, 滑到 {crows[-1]['v']:.2f} m/s")
            else:
                print(f"[coast] ❌ {cnote}")

        # ---- 刹车扫描：beta x v0 ----
        if not args.no_brake:
            combos = [(b, v0) for b in BRAKE_LEVELS for v0 in BRAKE_V0S]
            for k, (beta, v0) in enumerate(combos, 1):
                bf = os.path.join(runs_dir, f"brake_{beta:.2f}_{v0:.0f}.csv")
                if os.path.exists(bf) and not args.force:
                    continue
                brows, bnote = run_brake(world, bp, spawn_pt, beta, v0, max_steps)
                if not brows:
                    print(f"[brake {k}/{len(combos)}] beta={beta:.2f} v0={v0:.0f}  ❌ {bnote}")
                    continue
                smooth(brows)
                write_run(bf, brows)
                print(f"[brake {k}/{len(combos)}] beta={beta:.2f} v0={v0:.0f}  -> "
                      f"{len(brows)} tick, 减到 {brows[-1]['v']:.2f} m/s")
    finally:
        try:
            settings.synchronous_mode = False
            world.apply_settings(settings)
            print("\n[carla] 同步模式已关闭。")
        except Exception as e:
            print(f"\n[carla] ⚠ 收尾失败（服务端可能已崩）：{e}")

    if summary:
        sum_path = os.path.join(args.out, "summary.csv")
        with open(sum_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["phase", "throttle", "v", "status", "note"])
            w.writeheader()
            w.writerows(summary)
        print(f"[out] {sum_path}")

    plot(runs_dir, png_path, map_name, args.vehicle)


if __name__ == "__main__":
    main()
