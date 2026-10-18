"""09_acc_carla.py -- ACC 闭环：**CARLA 当被控对象，acc_sim 的 ACC 当主车控制**。

这一条闭环把三样东西接起来
--------------------------
    acc_sim 的 ACC（被测 SUT）        <- 独立仓库，按路径 import，不属于本工程
        |  请求加速度 a
    CarlaPedalMap（标定的逆映射）      <- 把 a 翻译成踏板；加载时校验车型 + 迟滞差
        |  throttle/brake
    CARLA 车辆（真·被控对象）          <- 实测速度回到 ACC

★ 为什么要这一步：里程碑 1 发现 **CARLA 的 throttle ≠ 加速度**（恒定 0.5 时，
  1 m/s 给 1.85 m/s²，13 m/s 只剩 0.085 m/s²，差 20 倍）。直接接上去，ACC
  "想加速却加不动"。所以中间必须有 `CarlaPedalMap` 这一步翻译。

台架形态：1-D 纵向台（两台车都钉住位置）
---------------------------------------
每 tick 跑完物理后把两台车各自拉回 home（**保留速度**，见 carla_env.tick_pinned）：
- **纵向动力学仍然完全来自 CARLA**（踏板 → dv/dt 是 CARLA 算的）—— 要测的就是这个
- 位置不积分，改由我们自己按实测速度积 1-D 车距：gap += (v_lead - v_ego) * dt
- 于是**没有横向、没有路口、没有掉出路沿、没有真碰撞、不依赖地图**

⚠ 边界（必须跟结论一起交付，别当成"完整闭环"）：
  ① 无横向动力学 / 无转向 —— 这套台架不测 ALCA/LKA 类功能
  ② 无雷达模型 —— distance 是几何真值，不是传感器输出（所以测不了检出/虚警）
  ③ 无真空间碰撞 —— gap<0 表示"算出来撞上了"，不是物理接触
  ④ 前车是脚本给定的速度剖面，不是真车跟驰（无人类驾驶行为）

跑法：
  PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe 09_acc_carla.py
  ... --set-speed 25 --duration 60 --vehicle vehicle.tesla.model3
"""

import argparse
import csv
import json
import math
import os
import sys

import carla
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

from carla_env import ensure_carla, settle, slug, speed_of, tick_pinned  # noqa: E402
from carla_pedal_map import CarlaPedalMap  # noqa: E402

# ------------------------- acc_sim（被测系统）-------------------------
# acc_sim 是独立仓库，不装到 site-packages；按路径导入。
# ★ 这里是唯一一处"跨仓库"的耦合，放在文件顶上一个显眼的地方。
ACC_SIM_DIR = os.environ.get("ACC_SIM_DIR", r"D:\AI\myProject\acc-sim")
if ACC_SIM_DIR not in sys.path:
    sys.path.insert(0, ACC_SIM_DIR)
from acc.models import AccConfig, FrontVehicle  # noqa: E402
from acc.system import ACCSystem  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB = os.path.join(HERE, "calib")
DT = 0.1

# 场景：前车目标速度剖面 (t_start, v_target)。ACC 参数 set_speed=25 → 前车跑远后回巡航
SCENARIO = [(0.0, 15.0), (20.0, 8.0), (35.0, 25.0), (50.0, 25.0)]
GAP0 = 80.0        # 初始车距 (m) —— 故意 > follow_switch_dist(60)，先看到巡航→跟车切换
EGO_V0 = 15.0      # 主车初速 (m/s)
WARMUP = 100       # ACC 接入前的稳定拍数

FIELDS = ["t", "v_ego", "v_lead", "gap", "gap_set", "a_req", "throttle", "brake",
          "mode", "state", "saturated"]


def lead_target(t, scenario):
    """前车目标速度：阶跃式取最近一个已生效的段落（不插值，确保场景可复现）。"""
    v = scenario[0][1]
    for t_start, v_t in scenario:
        if t >= t_start:
            v = v_t
    return v


def drive_to(pm, world, vehicle, home, v_target, steps, a_max=1.5):
    """用标定表自己做 P 闭环，把车带到 v_target。

    ★ 顺便验证了逆映射是对的 —— 命令一个速度，车真能到那儿。
      （如果表反了/接错了，这里就到不了目标速度，后面全白跑。）
    """
    for _ in range(steps):
        v = speed_of(vehicle)
        a = max(-2.0, min(a_max, 1.2 * (v_target - v)))
        th, br = pm.pedals(a, v)
        tick_pinned(world, vehicle, home, th, br)
    return speed_of(vehicle)


def run(world, pm, vehicle_id, scenario, set_speed, duration, gap0, ego_v0,
        warmup, out_dir):
    bp_lib = world.get_blueprint_library()
    pts = world.get_map().get_spawn_points()
    pt_e = pts[0]
    pt_l = pts[min(3, len(pts) - 1)]

    def make(vid, pt):
        bp = bp_lib.find(vid)
        v = world.try_spawn_actor(bp, pt)
        if v is None:
            raise RuntimeError(f"{vid} spawn 失败")
        return v

    ego, lead = make(vehicle_id, pt_e), make(vehicle_id, pt_l)
    rows = []
    try:
        # ★★ 起步顺序不能省：**先 settle 落地停稳，再把此刻的 transform 当作 home**。
        #   不能用出生点当 home、也不能用 tick_pinned 去"停稳" ——
        #   出生点 z 是半空（车从那儿落下），tick_pinned 每拍把车拽回那个高度，
        #   车永远落不了地，刹车踩满也停不下来（实测 30 拍后仍残留 |v|≈5.2 m/s，
        #   而 03/07 的 settle() 是**不钉位置**的普通 tick，1.1 s 内落地归零）。
        #   踩过的教训见 P-SIM-02：同一件事只留一个实现 —— settle 直接 import，别重写。
        for name, v in (("主车", ego), ("前车", lead)):
            if not settle(world, v):
                raise RuntimeError(f"{name} settle 失败（车没停稳）")
        home_e, home_l = ego.get_transform(), lead.get_transform()
        drive_to(pm, world, ego, home_e, ego_v0, 600)
        drive_to(pm, world, lead, home_l, lead_target(0.0, scenario), 600)
        print(f"[init] 主车 {speed_of(ego):.2f} m/s   前车 {speed_of(lead):.2f} m/s   "
              f"初始车距 {gap0:.1f} m")

        # ---- ACC：注入标定表反解 ----
        cfg = AccConfig(set_speed=set_speed, dt=DT)
        acc = ACCSystem(cfg=cfg, pedal_map=pm.pedals)
        acc.turn_on()
        acc.activate()
        print(f"[acc ] 接入 ACC：set_speed={cfg.set_speed} m/s  time_gap={cfg.time_gap} s  "
              f"min_gap={cfg.min_gap} m  >> 踏板映射 = CarlaPedalMap")

        gap = gap0
        n = int(duration / DT)
        for i in range(n):
            t = i * DT
            v_e, v_l = speed_of(ego), speed_of(lead)

            # 1-D 车距积分（台架的核心：位置由我们自己积，动力学由 CARLA 给）
            gap += (v_l - v_e) * DT

            # ① SUT：ACC 出请求
            target = FrontVehicle(distance=gap, rel_velocity=v_l - v_e)
            out = acc.step(target, v_e)

            # ② 前车：脚本速度剖面 + 同一个标定表做 P 闭环
            v_t = lead_target(t, scenario)
            a_l = max(-2.0, min(1.5, 1.2 * (v_t - v_l)))
            th_l, br_l = pm.pedals(a_l, v_l)
            tick_pinned(world, lead, home_l, th_l, br_l)

            # ③ 主车：ACC 的踏板（翻译已在 ACCSystem 内部用注入的 pedal_map 做完）
            tick_pinned(world, ego, home_e, out.throttle, out.brake)

            if i >= warmup:
                d_set = cfg.min_gap + cfg.time_gap * v_e
                rows.append({"t": round(t, 2), "v_ego": v_e, "v_lead": speed_of(lead),
                             "gap": gap, "gap_set": d_set, "a_req": out.accel,
                             "throttle": out.throttle, "brake": out.brake,
                             "mode": out.mode, "state": str(out.state).split(".")[-1],
                             "saturated": int(abs(out.accel) > 1e-9 and
                                              abs(pm.predict(out.throttle, out.brake, v_e)
                                                  - out.accel) > 0.3)})
                if i % 100 == 0:
                    print(f"  t={t:5.1f}s  v_ego={v_e:5.2f}  v_lead={rows[-1]['v_lead']:5.2f}  "
                          f"gap={gap:6.2f} (设定 {d_set:6.2f})  a_req={out.accel:+5.2f}  "
                          f"thr={out.throttle:.3f} brk={out.brake:.3f}  {out.mode}")
    finally:
        ego.destroy()
        lead.destroy()

    return rows


def report(rows, cfg, out_dir, vehicle_id, pm, map_name):
    if not rows:
        print("[report] 没有数据")
        return None
    gap = [float(r["gap"]) for r in rows]
    d_set = [float(r["gap_set"]) for r in rows]
    err = [g - d for g, d in zip(gap, d_set)]
    v_ego = [float(r["v_ego"]) for r in rows]
    brk_used = sum(1 for r in rows if float(r["brake"]) > 1e-6)
    sat = sum(int(r["saturated"]) for r in rows)

    print("\n" + "=" * 78)
    print("ACC 闭环结果（CARLA 当被控对象）")
    print("=" * 78)
    print(f"  样本 {len(rows)} 拍 / {len(rows) * DT:.0f} s   地图 {map_name}   车型 {vehicle_id}")
    print(f"  车距误差: 均值 {sum(err) / len(err):+6.2f} m   最大偏差 {max(abs(e) for e in err):6.2f} m"
          f"   终值 {err[-1]:+6.2f} m")
    print(f"  最小车距: {min(gap):6.2f} m   " +
          ("★ 发生过追尾（gap<0）" if min(gap) < 0 else "未发生碰撞"))
    print(f"  本车速度: {min(v_ego):.2f} ~ {max(v_ego):.2f} m/s")
    print(f"  ★ 刹车踏板被踩到过的拍数: {brk_used}/{len(rows)} "
          f"({100 * brk_used / len(rows):.1f}%)   —— 其余全是收油/给油")
    print(f"  ★ 请求被表截断（|实际-请求|>0.3）的拍数: {sat}/{len(rows)}")
    print(f"  跟车/巡航拍数: " +
          str({m: sum(1 for r in rows if r["mode"] == m) for m in ("cruise", "follow", "none")}))

    verdict = {"vehicle": vehicle_id, "map": map_name, "n": len(rows),
               "gap_err_mean": sum(err) / len(err), "gap_err_max_abs": max(abs(e) for e in err),
               "gap_min": min(gap), "collision": min(gap) < 0,
               "brake_ticks": brk_used, "saturated_ticks": sat,
               "v_min": min(v_ego), "v_max": max(v_ego)}
    p = os.path.join(out_dir, "acc_loop_verdict.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(verdict, f, ensure_ascii=False, indent=2)
    print(f"\n[out] {p}")
    return verdict


def plot(rows, cfg, out_png, vehicle_id):
    t = [float(r["t"]) for r in rows]
    fig, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    ax[0].plot(t, [float(r["v_ego"]) for r in rows], label="主车 v_ego（CARLA 实测）", lw=1.6)
    ax[0].plot(t, [float(r["v_lead"]) for r in rows], label="前车 v_lead（场景）", lw=1.2, ls="--")
    ax[0].axhline(cfg.set_speed, color="gray", lw=0.8, ls=":", label=f"set_speed={cfg.set_speed}")
    ax[0].set_ylabel("速度 (m/s)")
    ax[0].legend(loc="best", fontsize=8)
    ax[0].set_title(f"ACC 闭环 @ CARLA（{vehicle_id}）—— 纵向动力学来自 CARLA，踏板经标定表反解")

    ax[1].plot(t, [float(r["gap"]) for r in rows], label="实测车距", lw=1.6)
    ax[1].plot(t, [float(r["gap_set"]) for r in rows], label="ACC 设定车距 min_gap + gap·v", lw=1.2, ls="--")
    ax[1].set_ylabel("车距 (m)")
    ax[1].legend(loc="best", fontsize=8)

    ax[2].plot(t, [float(r["a_req"]) for r in rows], label="ACC 请求加速度 a_req", lw=1.6, color="tab:blue")
    ax[2].axhline(0.0, color="k", lw=0.6)
    ax[2].set_xlabel("时间 (s)")
    ax[2].set_ylabel("a_req (m/s²)", color="tab:blue")
    ax[2].tick_params(axis="y", labelcolor="tab:blue")
    # ★ 踏板量级（0~1）比 a_req（±4）大一个量级，同轴画会把 a_req 压成直线 ——
    #   用双轴，各看各的尺度。刹车那条若全程贴 0 本身就是结论（见 docstring 的 G(v) 说明）。
    ax2b = ax[2].twinx()
    ax2b.plot(t, [float(r["throttle"]) for r in rows], label="油门 throttle", lw=1.0,
              color="tab:orange", alpha=0.85)
    ax2b.plot(t, [float(r["brake"]) for r in rows], label="刹车 brake", lw=1.0,
              color="tab:green", alpha=0.85)
    ax2b.set_ylabel("踏板 0~1", color="tab:orange")
    ax2b.set_ylim(-0.05, 1.05)
    h1, l1 = ax[2].get_legend_handles_labels()
    h2, l2 = ax2b.get_legend_handles_labels()
    ax[2].legend(h1 + h2, l1 + l2, loc="best", fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=110)
    print(f"[out] {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="vehicle.tesla.model3")
    ap.add_argument("--set-speed", type=float, default=25.0)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--gap0", type=float, default=GAP0)
    ap.add_argument("--ego-v0", type=float, default=EGO_V0)
    ap.add_argument("--warmup", type=int, default=WARMUP)
    ap.add_argument("--hysteresis-tol", type=float, default=0.5)
    ap.add_argument("--out", default="")
    ap.add_argument("--replot", action="store_true",
                    help="只从已有 acc_loop.csv 重出图（不启 CARLA）")
    args = ap.parse_args()

    slug_v = slug(args.vehicle)
    model = os.path.join(CALIB, "longitudinal_model.json")
    balance = os.path.join(CALIB, f"veh_{slug_v}", "balance_table.csv")

    # ★ 重出图走**同一个 plot()** —— 另写一份画图代码就是 P-SIM-02 的翻版。
    if args.replot:
        out_dir = args.out or os.path.join(CALIB, f"acc_loop_{slug_v}")
        csv_p = os.path.join(out_dir, "acc_loop.csv")
        with open(csv_p, encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        cfg = AccConfig(set_speed=args.set_speed, dt=DT)
        report(rows, cfg, out_dir, args.vehicle, CarlaPedalMap(model, balance_csv=balance),
               "(replot)")
        plot(rows, cfg, os.path.join(out_dir, "acc_loop.png"), args.vehicle)
        return 0

    # ★ 加载即校验：车型对不上、或这张表根本没有唯一平衡点 → 直接拒绝，不"带病开跑"
    pm = CarlaPedalMap(model, expected_vehicle=args.vehicle,
                       hysteresis_tol=args.hysteresis_tol, balance_csv=balance)
    print("[pedal] 标定表已加载并校验通过")
    print("        " + pm.report().replace("\n", "\n        "))

    world, map_name = ensure_carla()
    print(f"[carla] map = {map_name}")

    st = world.get_settings()
    st.synchronous_mode = True
    st.fixed_delta_seconds = DT
    world.apply_settings(st)

    out_dir = args.out or os.path.join(CALIB, f"acc_loop_{slug_v}")
    os.makedirs(out_dir, exist_ok=True)

    try:
        rows = run(world, pm, args.vehicle, SCENARIO, args.set_speed, args.duration,
                   args.gap0, args.ego_v0, args.warmup, out_dir)
    finally:
        st.synchronous_mode = False
        world.apply_settings(st)

    p = os.path.join(out_dir, "acc_loop.csv")
    with open(p, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"[out] {p}")

    cfg = AccConfig(set_speed=args.set_speed, dt=DT)
    report(rows, cfg, out_dir, args.vehicle, pm, map_name)
    plot(rows, cfg, os.path.join(out_dir, "acc_loop.png"), args.vehicle)
    print(f"\n[pickup] ★ 刹车踏板用到的拍数占比 = {100 * sum(1 for r in rows if float(r['brake']) > 1e-6) / max(1, len(rows)):.1f}%"
          f"  —— 这个数本身就是 CARLA 阻力模型的指纹（G(v) 远大于 ACC 的寻常减速请求）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
