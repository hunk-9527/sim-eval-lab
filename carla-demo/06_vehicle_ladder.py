"""06_vehicle_ladder.py -- 跨车型纵向特性对照：标定表到底有多"车型专属"？

背景
----
03 标出的表只对 vehicle.tesla.model3 成立。台架常识说换车就得重标，但
"差多少"没人量过。本脚本沿一条质量阶梯跑多台车的恒定油门扫描，回答：

    Q1  同油门不同车，稳态速度差多少？（差一个量级还是差几成）
    Q2  差异是否随油门单调？（小油门段是否几乎一样，大油门段才拉开）
    Q3  死区阈值是否也随车型变？（低油门能不能起步）

只跑 hold 扫描（不跑 coast / brake）：阻力与制动留作下一轮，先把最贵的
那个结论拿到手 —— 表能不能跨车复用。

★ CARLA 会崩（踩坑③ load_world、以及随机的 OOM）。所以本脚本自己管进程：
  每台车开跑前探活，死了就重启并等就绪；03 本身分档落盘，重跑自动续。
  整晚无人值守靠的就是这两条。

用法
----
    PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe 06_vehicle_ladder.py
    ... --smoke                 # 冒烟：每台车只跑 3 档
    ... --vehicles vehicle.audi.a2,vehicle.tesla.model3
"""

import argparse
import csv
import glob
import json
import os
import subprocess
import sys
import time

from carla_env import ensure_carla, kill_carla, slug   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PY = r"D:/TOOL/SIL/envs/sim/python.exe"
CALIB = os.path.join(HERE, "calib")
CALIB_SCRIPT = os.path.join(HERE, "03_calibrate_longitudinal.py")

# 质量阶梯：微型 / 小型 / 中型（基线）/ 大型 SUV / 半挂
# ⚠ 质量数字来自公开规格，**不是本实验测的**，只作为排序假设，不作证据。
DEFAULT_LADDER = [
    ("vehicle.micro.microlino",   "微型两座 (~500 kg 级)"),
    ("vehicle.audi.a2",           "小型两厢 (~1100 kg 级)"),
    ("vehicle.tesla.model3",      "中型轿车（基线，~1800 kg 级）"),
    ("vehicle.nissan.patrol",     "大型 SUV (~2500 kg 级)"),
    ("vehicle.carlamotors.carlacola", "半挂牵引车 (~5500 kg 级)"),
]


# 进程管理（carla_world / kill_carla / launch_carla / ensure_carla）与 slug()
# 已抽到 carla_env.py，03/06/07 共用一份 —— 别再在这里写第二份实现。


# --------------------------- 跑一台车 ---------------------------
def run_vehicle(vid, smoke=False, max_steps=1500, retries=2):
    """调 03 跑一台车的 hold 扫描。03 分档落盘，重跑自动跳过已完成档。"""
    out = os.path.join(CALIB, "veh_" + slug(vid))
    os.makedirs(os.path.join(out, "runs"), exist_ok=True)
    cmd = [PY, CALIB_SCRIPT, "--vehicle", vid, "--out", out,
           "--no-coast", "--no-brake", "--max-steps", str(max_steps)]
    if smoke:
        cmd.append("--quick")
    env = dict(os.environ, PYTHONIOENCODING="utf-8")

    for attempt in range(1, retries + 1):
        ensure_carla()                      # 开跑前探活，死了就重启
        print(f"\n{'='*72}\n[ladder] {vid}  第 {attempt} 次尝试\n{'='*72}")
        # 不用 text=True：CARLA 服务端会往子进程 stderr 里塞 GBK 字节，
        # 交给 subprocess 的 reader thread 解码会抛 UnicodeDecodeError。自己解。
        p = subprocess.run(cmd, cwd=HERE, env=env, capture_output=True)
        out_txt = (p.stdout or b"").decode("utf-8", "replace") \
            + (p.stderr or b"").decode("utf-8", "replace")
        for line in out_txt.splitlines():
            if line.strip() and "streaming client" not in line:
                print("   ", line)
        ok, done = sweep_progress(out)
        print(f"[ladder] {vid}: rc={p.returncode} 完成 {done} 档")
        if ok and p.returncode == 0:
            return out, True
        print(f"[ladder] {vid} 未跑完，重试 ...")
        kill_carla()
        time.sleep(5)
    return out, False


def sweep_progress(out_dir):
    """返回 (是否全部成功, 已完成档数)。"""
    p = os.path.join(out_dir, "summary.csv")
    if not os.path.exists(p):
        return False, 0
    rows = list(csv.DictReader(open(p, encoding="utf-8")))
    ok = bool(rows) and all(r["status"] in ("OK", "DEADZONE") for r in rows)
    return ok, len(rows)


# --------------------------- 汇总额 ---------------------------
# 稳态分类判据
DEAD_V = 0.05
STEADY_WIN = 30       # ★ 必须与 03 的判据同窗，否则"停在哪一拍"和分类对不上
                      # 03 写的是 speeds[-1] vs speeds[-30]，列表长为 n 时
                      # 索引差是 29 不是 30 —— 差这一拍就会把 0.0498 判成 0.0526，
                      # 全部误判为"未达稳态"（实际踩过）。
CY_NET = 0.05         # 窗口首尾速度差上限（同 03 的 STEADY_TOL）
OSC_WIN = 30          # 振荡检查窗口
CY_ABS_A = 0.15       # 窗口内 |a| 均值上限：超了就是限幅振荡
# ↑ 为什么必须和判据同窗：长窗口（试过 100 拍）会把**起步加速段**也圈进来，
#   于是刚起步的车被误判成"振荡"。判据窗口里的这段，是 03 已经用
#   max|a|<=1.0 筛过的安静段，拿它判振荡最干净。
#   实测分离度：真稳态 |a|≈0.02~0.09，限幅振荡 ≈0.16~0.66。


def classify_csv(path):
    """从原始曲线判定这一档的真实结局。返回 dict(v, cls, drift, a_mean)。

    只看"速度不再变"是不够的：换挡狩猎会在 v≈5 附近形成**限幅振荡** ——
    首尾速度差 <0.05（看着像稳态），但 a 在 ±0.5~±2.5 之间持续交替。
    """
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    if not rows:
        return {"v": 0.0, "cls": "空", "drift": 0.0, "a_mean": 0.0}
    v = [float(r["v"]) for r in rows]
    a = [abs(float(r["a"])) for r in rows]
    v_end = v[-1]
    drift = abs(v[-1] - v[-min(STEADY_WIN, len(v))])
    ow = min(OSC_WIN, len(v))
    a_mean = sum(a[-ow:]) / ow
    if max(v) < DEAD_V:
        cls = "死区"
    elif drift >= CY_NET:
        cls = "未达稳态"
    elif a_mean > CY_ABS_A:
        cls = "限幅振荡"
    else:
        cls = "真稳态"
    return {"v": v_end, "cls": cls, "drift": drift, "a_mean": a_mean}


def collect(out_dir):
    """读 meta.json + runs/*.csv，返回 (meta, {throttle: 判定 dict})。

    **以原始曲线为唯一数据源**，不用 summary.csv：03 在"已有结果则跳过"时
    不会补写 summary，跳过档位会在汇总里留洞（实际踩过：audi.a2 缺 3 档）。
    """
    meta = {}
    mp = os.path.join(out_dir, "meta.json")
    if os.path.exists(mp):
        meta = json.load(open(mp, encoding="utf-8"))
    tbl = {}
    for p in glob.glob(os.path.join(out_dir, "runs", "hold_*.csv")):
        tbl[float(os.path.basename(p)[5:-4])] = classify_csv(p)
    return meta, tbl


MARK = {"真稳态": " ", "限幅振荡": "~", "未达稳态": ">", "死区": "z"}


def report(results, out_dir=CALIB):
    """对照表 + 图 + CSV。"""
    levels = sorted({t for _, tbl in results for t in tbl})
    names = [m.get("vehicle", "?") for m, _ in results]
    short = [n.replace("vehicle.", "") for n in names]
    print("\n" + "=" * 104)
    print("跨车型纵向对照   v_ss [m/s]（同一恒定油门，跑步机法，同图同步长）")
    print("  图例: 空=真稳态 | ~=限幅振荡(换挡狩猎，v 不动但 a 持续交替) "
          "| >=未达稳态(撞 v_max) | z=死区")
    print("=" * 104)
    print(f"{'throttle':>9} | " + " | ".join(f"{s:>17}" for s in short))
    print("-" * 104)
    for t in levels:
        cells = []
        for _, tbl in results:
            if t not in tbl:
                cells.append(f"{'--':>17} ")
                continue
            d = tbl[t]
            cells.append(f"{d['v']:15.3f}{MARK.get(d['cls'], '?')}")
        print(f"{t:9.2f} | " + " | ".join(f"{c:>17}" for c in cells))

    # 单值摘要
    print("-" * 104)
    for m, tbl in results:
        name = m.get("vehicle", "?").replace("vehicle.", "")
        real = {t: d["v"] for t, d in tbl.items() if d["cls"] == "真稳态"}
        cyc = sorted(t for t, d in tbl.items() if d["cls"] == "限幅振荡")
        base = real.get(0.50)
        print(f"  {name:22s} 真稳态 {len(real):2d}/12 档 | "
              f"v_ss(θ=0.50) = {'  --  ' if base is None else f'{base:6.2f}'} m/s | "
              f"限幅振荡(卡在换挡点) {cyc if cyc else '无'}")

    # 落盘 CSV（值 + 类别标记）
    csv_p = os.path.join(out_dir, "vehicle_ladder.csv")
    with open(csv_p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["throttle"] + names)
        for t in levels:
            w.writerow([f"{t:.2f}"] + [
                "" if t not in tbl else
                f"{tbl[t]['v']:.3f}{MARK.get(tbl[t]['cls'], '?')}"
                for _, tbl in results])
    print(f"[out] {csv_p}")

    # 落盘判据明细（可审计：每个单元格的 drift / |a|均值）
    mx_p = os.path.join(out_dir, "vehicle_ladder_metrics.csv")
    with open(mx_p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["vehicle", "throttle", "v_end", "class", "drift_29tick", "abs_a_mean"])
        for m, tbl in results:
            for t in sorted(tbl):
                d = tbl[t]
                w.writerow([m.get("vehicle", "?"), f"{t:.2f}", f"{d['v']:.3f}",
                            d["cls"], f"{d['drift']:.4f}", f"{d['a_mean']:.3f}"])
    print(f"[out] {mx_p}")
    return levels, names


def plot(results, levels, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(figsize=(9, 6))
    for meta, tbl in results:
        vid = meta.get("vehicle", "?").replace("vehicle.", "")
        real = sorted(t for t, d in tbl.items() if d["cls"] == "真稳态")
        cyc = sorted(t for t, d in tbl.items() if d["cls"] == "限幅振荡")
        dead = sorted(t for t, d in tbl.items() if d["cls"] == "死区")
        if not real:
            continue
        line, = ax.plot(real, [tbl[t]["v"] for t in real], "o-", lw=1.6, ms=5,
                        label=vid)
        c = line.get_color()
        if cyc:      # 换挡狩猎：卡在 v≈5 的限幅振荡，不是稳态
            ax.plot(cyc, [tbl[t]["v"] for t in cyc], "x", ms=10, mew=2.0, color=c)
        if dead:
            ax.plot(dead, [0.0] * len(dead), "s", ms=5, mfc="none", mec=c)
    ax.axhline(30.0, color="gray", ls="--", lw=1.0)
    ax.text(0.11, 30.6, "acc_sim set_speed = 30 m/s", color="gray", fontsize=9)
    ax.set_xlabel("throttle  θ")
    ax.set_ylabel("speed at end of run  (m/s)")
    ax.set_title("CARLA 跨车型纵向对照：同一恒定油门下的表现\n"
                 "(treadmill 0.1 s; ○=真稳态  ×=限幅振荡(非稳态)  □=死区)")
    ax.set_ylim(-1, 45)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"[out] {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicles", default="",
                    help="逗号分隔的 blueprint id；默认跑完整质量阶梯")
    ap.add_argument("--smoke", action="store_true", help="每台车只跑 3 档（冒烟）")
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--skip-run", action="store_true", help="只汇总已有数据，不跑仿真")
    args = ap.parse_args()

    if args.vehicles:
        ladder = [(v.strip(), "") for v in args.vehicles.split(",") if v.strip()]
    else:
        ladder = DEFAULT_LADDER

    if args.skip_run:
        map_name = "(skip-run)"
        outs = [os.path.join(CALIB, "veh_" + slug(v)) for v, _ in ladder]
    else:
        w, map_name = ensure_carla()
        print(f"[carla] map = {map_name}（★ 不切图：load_world 第二次必崩）")
        outs = []
        for vid, desc in ladder:
            out, ok = run_vehicle(vid, smoke=args.smoke,
                                  max_steps=args.max_steps)
            outs.append(out)
            print(f"[ladder] {'✅' if ok else '⚠ 未完整'} {vid}  {desc}")

    results = [r for r in (collect(o) for o in outs) if r[1]]
    if not results:
        print("[ladder] 没有拿到任何数据")
        return
    levels, _ = report(results)
    plot(results, levels, os.path.join(CALIB, "vehicle_ladder.png"))
    print("\n[ladder] 地图 =", map_name, "（所有车型同一张图、同一步长，可横向比）")


if __name__ == "__main__":
    sys.exit(main())
