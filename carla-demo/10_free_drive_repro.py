"""10_free_drive_repro.py -- 自由行驶复现多稳态（补 07 的软肋）。

要回答的质疑
------------
07 的平衡点扫描建在**跑步机**上（`tick_pinned`：每拍把车拉回出发点、保留速度）。
"每拍钉住位置" 与 "真在路上开" 是否等价，取决于车辆动力学**是否平移不变**。
反对者完全可以说：多稳态是钉位置钉出来的伪影，真车不会这样。

本脚本让车**真在高速路上开**（一次都不钉位置），做同一件事：
    up   路径：从静止起，固定油门 θ 一路开，看它停在哪
    down 路径：先冲到比 up 落点高的速度，再阶跃降到 θ，看它停在哪
两条路径落点若不同 —— 多稳态在自由行驶下同样成立，钉位置不是成因。

★ 守门（自由行驶才有、跑步机没有的失效模式）
   ① 车心不能离开车道中心线超过半个车道宽（出车道 = 换路面 = 阻力变了）
   ② 不能发生碰撞（settle 阶段不计）
   ③ 车不能腾空
   ④ 不能跑出直路余量
   每次运行都报这几个量，不合规标 `invalid`，**不悄悄当有效数据用**。

★ 为什么窗口比 07 短
   07 用 warmup 400 + window 500 拍 = 90 s。自由行驶下 90 s × 15 m/s = 1350 m，
   而 Town04 最长直路只有 715 m —— 跑不下。
   本脚本用 warmup 150 + window 60 拍（= 21 s），并在**同一次运行里**
   用同样的窗口把跑步机对照也重跑一遍。★ 不复用 07 的旧数字：
   窗口不同则 Δ 不可比（P-SIM-02 判据同源）。

跑法
----
    PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe 10_free_drive_repro.py
    ... --vehicles vehicle.carlamotors.carlacola --levels 0.30,0.40,0.50

⚠ 需要地图 Town04（长直路）。命令行地图参数在 0.9.15 无效，换图见 carla_map.py。
"""

import argparse
import csv
import importlib.util
import json
import math
import os
import time

import carla

from carla_env import ensure_carla, settle, slug, speed_of

HERE = os.path.dirname(os.path.abspath(__file__))


def _load07():
    """把 07 当模块加载 —— 统计与分类**只留一份实现**（P-SIM-02）。"""
    p = os.path.join(HERE, "07_low_speed_balance.py")
    spec = importlib.util.spec_from_file_location("bal07", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


bal07 = _load07()
DT = bal07.DT

# ------------------------- 实验参数 -------------------------
MAP_REQUIRED = "Town04"
STRAIGHT_START = (-364.3, 37.1)   # 探针找到的 715 m 直路起点（_probe_straight.py）

WARMUP = 150      # 15 s 跑合（07 是 400/40 s，这里受路长限制）
WINDOW = 60       # 6 s 统计窗口（07 是 500/50 s）
MAX_DIST = 640.0  # 直路 715 m，留 75 m 余量；超了判 invalid（车已不在直路上）
SPIN_DIST_FRAC = 0.50   # 冲刺阶段最多用掉 50% 路程预算

PRE_SPIN_THETA = 0.80
# ★ 冲刺目标必须**高过迟滞环的上分支**，否则根本进不了高分支 ——
#   第一版我按"v_up + 2"自适应，carlacola 的 v_up=2.6 → 目标 4.6 m/s，
#   结果降速路径也落在 4.31 m/s，看起来"没有迟滞"——其实是我们没把车开到环的另一侧。
#   07 的冲刺目标是固定的 25 m/s。
# ★★ 冲刺目标还必须是**自由行驶在路程预算内够得到**的速度：
#   取 16 m/s / 200 拍时，自由行驶 20 s 只到 14.5 m/s（跑步机同一条件 3 s 就到 16）——
#   两条路径的初条件不同，落点差里混进了冲刺速度差。**修法不是抬目标值，而是配对**
#   （见 main 的降速段：跑步机用自由行驶**实测**的冲刺末速当自己的目标）。
# ⚠ 抬到 17 m/s 试过，两头都塌：冲刺段吃掉 313 m 路程，210 拍测量跑不完 640 m 直路；
#   而且 17 m/s 之后降速路径**根本没稳定**（θ=0.30 从 17.0 先降到 15.6 再一路爬到 18.7）——
#   等于进了另一个区制，测的不是同一个平衡点。所以固定回 16 m/s。
PRE_SPIN_V = 16.0
PRE_SPIN_STEPS = 200      # 冲刺上限 20 s（自由行驶约走 215 m）

# 默认靶子（数字取自 07 的 balance_table，两侧都是"稳定平衡"的那几档）
DEFAULT_VEHICLES = "vehicle.carlamotors.carlacola,vehicle.tesla.model3"
DEFAULT_LEVELS = "0.30,0.40,0.50"

FIELDS = ["mode", "throttle", "step", "t", "v", "a", "dist", "lat"]


def pick_spawn(world):
    """挑离 STRAIGHT_START 最近的出生点。"""
    sps = world.get_map().get_spawn_points()
    tgt = carla.Location(x=STRAIGHT_START[0], y=STRAIGHT_START[1])
    best = min(sps, key=lambda s: s.location.distance(tgt))
    return best, best.location.distance(tgt)


def align_yaw(world, vehicle, steps=8):
    """把车的朝向对齐到所在车道的切线（只改朝向，不动位置、不动速度）。

    ★ 为什么必须做：出生点的朝向与路面切线**实测差约 0.4°**。
      0.4° 在 100 m 上是 0.7 m 横向偏移（看不出来），在 600 m 上就是 4 m ——
      车会自己飘出车道，然后被误判成"自由行驶不成立"。
      这是**实验装置的装配误差**，不是被测物理，必须消掉而不是容忍。

    返回对齐前后的朝向差（度），写进产出便于复核。
    """
    m = world.get_map()
    t = vehicle.get_transform()
    wp = m.get_waypoint(t.location)
    f = wp.transform.get_forward_vector()
    yaw = math.degrees(math.atan2(f.y, f.x))
    err = abs((yaw - t.rotation.yaw + 180) % 360 - 180)
    t.rotation.yaw = yaw
    vehicle.set_transform(t)
    for _ in range(steps):
        world.tick()
    return err


def run_free(world, bp, spawn_pt, theta, mode, warmup, window,
             pre_spin_theta=PRE_SPIN_THETA, pre_spin_v=PRE_SPIN_V):
    """自由行驶一次。mode: up（从静止）| down（先冲高速再阶跃到 theta）。

    ★ 与 07 的 run_balance 唯一的差别：**没有 tick_pinned** ——
      每拍只 apply_control + world.tick()，位置由物理自己积分。
    """
    vehicle = world.try_spawn_actor(bp, spawn_pt)
    if vehicle is None:
        return None, "spawn 失败（出生点被占）", {}

    hits = []
    sensor = None
    try:
        col_bp = world.get_blueprint_library().find("sensor.other.collision")
        sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=vehicle)
        sensor.listen(lambda e: hits.append(e))
        # ★ 碰撞要记下**撞的是谁**：只报次数的话，
        #   "车蹭了护栏" 和 "车被别的 actor 推着走" 看起来一模一样，无法定位。

        if not settle(world, vehicle):
            return None, "settle 失败（车没停稳）", {}
        yaw_err = align_yaw(world, vehicle)
        hits.clear()          # 落地/对齐那一下的接触不算碰撞

        m = world.get_map()
        w0 = m.get_waypoint(vehicle.get_location())
        lane_w = w0.lane_width

        rows, speeds = [], []
        dist, max_lat, airborne = 0.0, 0.0, False
        nonlocal_off_road = [False]      # 离开机动车道（路面变了）
        prev = vehicle.get_location()

        def step(throttle, brake=0.0):
            nonlocal dist, prev, max_lat, airborne
            ctrl = carla.VehicleControl()
            ctrl.throttle, ctrl.brake, ctrl.steer = throttle, brake, 0.0
            vehicle.apply_control(ctrl)
            world.tick()
            loc = vehicle.get_location()
            dist += math.hypot(loc.x - prev.x, loc.y - prev.y)
            prev = loc
            wp = m.get_waypoint(loc)
            lat = math.hypot(loc.x - wp.transform.location.x,
                             loc.y - wp.transform.location.y)
            max_lat = max(max_lat, lat)
            if wp.lane_type != carla.LaneType.Driving:
                nonlocal_off_road[0] = True
            if loc.z - wp.transform.location.z > 0.5:
                airborne = True
            return speed_of(vehicle)

        # ---- 降速路径：先冲刺，制造"从高速过来"的初条件 ----
        spin = 0
        if mode == "down":
            for _ in range(PRE_SPIN_STEPS):
                spin += 1
                if step(pre_spin_theta) >= pre_spin_v:
                    break
                if dist > MAX_DIST * SPIN_DIST_FRAC:
                    break
        speeds.append(speed_of(vehicle))
        spin_v = speeds[0]        # ★ 冲刺阶跃瞬间的真实速度 —— 配对对照必须用它

        total = warmup + window
        aborted = None
        for i in range(total):
            v = step(theta)
            a = (v - speeds[-1]) / DT
            speeds.append(v)
            rows.append({"mode": mode, "throttle": round(theta, 3), "step": i,
                         "t": round(i * DT, 3), "v": round(v, 5), "a": round(a, 5),
                         "dist": round(dist, 2), "lat": round(max_lat, 3)})
            if dist > MAX_DIST:
                aborted = f"跑出直路（{dist:.0f} m > {MAX_DIST:.0f} m，第 {i} 拍）"
                break
        if not rows:
            return None, aborted or "没有采样", {}

        info = {"dist": dist, "max_lat": max_lat, "lane_w": lane_w,
                "collisions": len(hits), "airborne": airborne, "yaw_err": yaw_err,
                "off_road": nonlocal_off_road[0],
                "spin": spin, "steps": len(rows), "spin_target": pre_spin_v,
                "spin_v": spin_v,
                "hit_by": sorted({(e.other_actor.type_id if e.other_actor else "?")
                                  for e in hits})}
        if aborted:
            info["invalid"] = aborted
        elif hits:
            info["invalid"] = f"发生碰撞（{len(hits)} 次）"
        elif nonlocal_off_road[0]:
            info["invalid"] = "离开机动车道（路面变了，阻力不再是同一条）"
        elif max_lat > 2 * lane_w:
            info["invalid"] = f"横向偏移 {max_lat:.2f} m 超过 2 个车道宽 {2*lane_w:.2f} m"
        elif airborne:
            info["invalid"] = "车辆腾空"
        elif len(rows) < warmup + window:
            info["invalid"] = f"采样不足（{len(rows)} < {warmup + window} 拍）"
        return rows, "ok", info
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass
            sensor.destroy()
        vehicle.destroy()
        world.tick()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicles", default=DEFAULT_VEHICLES)
    ap.add_argument("--levels", default=DEFAULT_LEVELS)
    ap.add_argument("--warmup", type=int, default=WARMUP)
    ap.add_argument("--window", type=int, default=WINDOW)
    ap.add_argument("--skip-pinned", action="store_true", help="不跑跑步机对照")
    ap.add_argument("--skip-free", action="store_true", help="只跑跑步机对照")
    args = ap.parse_args()

    vehicles = [v.strip() for v in args.vehicles.split(",") if v.strip()]
    levels = [float(x) for x in args.levels.split(",") if x.strip()]
    kinds = (["pinned"] if args.skip_free else
             ["free"] if args.skip_pinned else ["free", "pinned"])

    world, map_name = ensure_carla(map_name=MAP_REQUIRED)
    short = map_name.split("/")[-1]
    if short != MAP_REQUIRED:
        print(f"❌ 当前图 {short}，需要 {MAP_REQUIRED}（换图见 carla_map.py）")
        return 1
    print(f"[carla] map = {map_name}")

    stt = world.get_settings()
    stt.synchronous_mode = True
    stt.fixed_delta_seconds = DT
    world.apply_settings(stt)

    spawn_pt, d = pick_spawn(world)
    print(f"[carla] 直路起点出生点偏差 {d:.1f} m；warmup={args.warmup} "
          f"window={args.window} 拍（{DT}s/拍）；预算 {MAX_DIST:.0f} m")

    out_dir = os.path.join(HERE, "calib", "free_drive_repro")
    os.makedirs(out_dir, exist_ok=True)

    results = []
    n = len(vehicles) * len(levels) * len(kinds) * 2
    k, t_start = 0, time.time()

    for vid in vehicles:
        try:
            bp = world.get_blueprint_library().find(vid)
        except IndexError:
            print(f"❌ 找不到车型 {vid}")
            continue
        for theta in levels:
            rec = {"vehicle": vid, "throttle": theta}
            v_up = None
            # ---------- up ----------
            for kind in kinds:
                k += 1
                t0 = time.time()
                if kind == "free":
                    rows, note, info = run_free(world, bp, spawn_pt, theta, "up",
                                                args.warmup, args.window)
                else:
                    rows, note = bal07.run_balance(
                        world, bp, spawn_pt, theta, "up", args.warmup, args.window)
                    info = {"note": note}
                if rows is None:
                    print(f"[{k}/{n}] {slug(vid):<24} θ={theta:.2f} up   {kind:<6} ❌ {note}")
                    continue
                s = bal07.window_stats(rows, window=args.window)
                cls = bal07.classify(s)
                if kind == "free":
                    v_up = s["v_mean"]
                _dump(out_dir, "up", kind, vid, theta, rows)
                print(f"[{k}/{n}] {slug(vid):<24} θ={theta:.2f} up   {kind:<6} "
                      f"v={s['v_mean']:6.2f} ±{s['v_std']:.3f} [{cls}] {_tail(info)}"
                      f" ({time.time()-t0:.0f}s)")
                rec[f"up_{kind}"] = {"v": s["v_mean"], "std": s["v_std"],
                                     "class": cls, "info": info}
            # ---------- down ----------
            # ★ 冲刺速度配对：自由行驶先跑，把**实测冲刺末速**交给跑步机当冲刺目标。
            #   只给同一个名义目标值是不够的 —— 自由行驶可能够不到目标、
            #   把 300 拍跑光就停（此时冲刺末速低于目标），两条路径初条件又不同了。
            #   用它实测到的速度当对照的目标，配对就是精确的。
            spin_v_free = None
            for kind in kinds:
                k += 1
                t0 = time.time()
                if kind == "free":
                    rows, note, info = run_free(world, bp, spawn_pt, theta, "down",
                                                args.warmup, args.window)
                    if rows is not None:
                        spin_v_free = info.get("spin_v")
                else:
                    target = spin_v_free or PRE_SPIN_V
                    rows, note = bal07.run_balance(
                        world, bp, spawn_pt, theta, "down", args.warmup, args.window,
                        pre_spin_v=target)
                    info = {"note": note, "spin_target": target}
                    if rows:
                        # 跑步机冲刺循环在**首次** v>=目标 时跳出 → 真实冲刺末速
                        # 落在 [target, target+一拍加速度]，偏差 < 0.15 m/s
                        info["spin_v"] = target
                if rows is None:
                    print(f"[{k}/{n}] {slug(vid):<24} θ={theta:.2f} down {kind:<6} ❌ {note}")
                    continue
                s = bal07.window_stats(rows, window=args.window)
                cls = bal07.classify(s)
                _dump(out_dir, "down", kind, vid, theta, rows)
                print(f"[{k}/{n}] {slug(vid):<24} θ={theta:.2f} down {kind:<6} "
                      f"v={s['v_mean']:6.2f} ±{s['v_std']:.3f} [{cls}] {_tail(info)}"
                      f" ({time.time()-t0:.0f}s)")
                rec[f"down_{kind}"] = {"v": s["v_mean"], "std": s["v_std"],
                                       "class": cls, "info": info}
            results.append(rec)

    _report(results)
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"map": map_name, "warmup": args.warmup, "window": args.window,
                   "max_dist": MAX_DIST, "straight_start": STRAIGHT_START,
                   "results": results}, f, ensure_ascii=False, indent=2)
    print(f"\n总耗时 {time.time()-t_start:.0f}s → {out_dir}")
    return 0


def _tail(info):
    if info.get("invalid"):
        return "❌ " + info["invalid"]
    if "dist" in info:
        return f"走 {info['dist']:.0f} m"
    return info.get("note", "")


def _dump(out_dir, mode, kind, vid, theta, rows):
    p = os.path.join(out_dir, f"{mode}_{kind}_{slug(vid)}_{theta:.2f}.csv")
    with open(p, "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=FIELDS)
        wr.writeheader()
        wr.writerows(rows)


def _report(results):
    print("\n" + "=" * 100)
    print(f"{'车型':<26}{'θ':>5}{'自由up':>9}{'自由down':>10}{'Δ自由':>8}"
          f"{'跑步机up':>10}{'跑步机down':>11}{'Δ跑步机':>9}{'一致?':>7}")
    print("-" * 100)
    def _bad(x):
        """该次运行被判定无效（跑出直路 / 碰撞 / 离开机动车道 …）。"""
        return bool(x and x["info"].get("invalid"))

    def _v(x, w, mark_bad=False):
        if x is None:
            return f"{'—':>{w}}"
        if mark_bad and _bad(x):
            return f"{('✗' + format(x['v'], '.2f')):>{w}}"
        return f"{x['v']:>{w}.2f}"

    for r in results:
        fu, fd = r.get("up_free"), r.get("down_free")
        pu, pd = r.get("up_pinned"), r.get("down_pinned")
        # ★★ 无效的自由行驶**绝不允许进 Δ**：
        #    tesla θ=0.40 的 up 撞护栏卡住（v=0.62），拿它算 Δ自由=6.19，
        #    会把"单速车没有多稳态"这条对照结论**反过来** —— 而且不报错。
        #    （同族：P-SIM-03 计数型代理量把分类弄反。）
        du = (abs(fu["v"] - fd["v"])
              if (fu and fd and not _bad(fu) and not _bad(fd)) else None)
        dp = abs(pu["v"] - pd["v"]) if pu and pd else None
        flag = ("⚠无效" if (_bad(fu) or _bad(fd))
                else "✅" if None not in (du, dp) else "-")
        print(f"{slug(r['vehicle']):<26}{r['throttle']:>5.2f}"
              + _v(fu, 9, True) + _v(fd, 10, True)
              + (f"{du:>8.2f}" if du is not None else f"{'—':>8}")
              + _v(pu, 10) + _v(pd, 11)
              + (f"{dp:>9.2f}" if dp is not None else f"{'—':>9}")
              + f"{flag:>8}")
    print("  ✗ = 该次自由行驶无效（见下），已从 Δ自由 中剔除；⚠无效 = 该行 Δ自由 不可用")
    print("\n冲刺末速配对（降速路径的初条件，必须两条路径相同）：")
    for r in results:
        fd = r.get("down_free")
        if not fd:
            continue
        fv = fd["info"].get("spin_v")
        pv = r.get("down_pinned", {}).get("info", {}).get("spin_v")
        dv = (fv - pv) if (fv and pv) else float("nan")
        print(f"  {slug(r['vehicle']):<24} θ={r['throttle']:.2f}  "
              f"自由冲刺末速 {fv if fv else float('nan'):5.2f}  "
              f"跑步机 {pv if pv else float('nan'):5.2f}  "
              f"差 {dv:+5.2f} m/s")

    print("\n自由行驶有效性：")
    for r in results:
        for m in ("up_free", "down_free"):
            if m in r:
                i = r[m]["info"]
                bad = i.get("invalid")
                tag = f"❌ {bad}" if bad else "✅ 有效"
                who = f"  撞 {','.join(i['hit_by'])}" if i.get("hit_by") else ""
                print(f"  {slug(r['vehicle']):<24} θ={r['throttle']:.2f} {m:<10} {tag}"
                      f"   走 {i.get('dist', 0):.0f} m  横向 {i.get('max_lat', 0):.2f} m"
                      f"  碰撞 {i.get('collisions', 0)}{who}  冲刺 {i.get('spin', 0)} 拍")


if __name__ == "__main__":
    raise SystemExit(main())
