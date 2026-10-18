"""11_grade_sweep.py -- S0′：只换钉住位置，量平衡车速随「测点局部坡度」怎么变。

要回答的问题
------------
07 的迟滞对照是在**一个固定出生点**上做的。跑步机法每拍把车拉回出发点，
于是它不只钉住了位置，也**把该点的路面坡度一起冻住了** —— 车一直在同一段
上坡/下坡上跑，重力的分量被算进了"动力总成"。

那么：同一台车、同一油门、同一冲刺初速，**只换钉住位置**，落点会变吗？

本脚本沿 Town04 长直道取 4 个落点（0 / 200 / 400 / 600 m），每点：
    ① 量该点的**局部纵坡**（前后各 5 m 的高差比）
    ② 用 07 的 run_balance 跑 up / down 两条路径（同一 θ）
    ③ 记落点、Δ、局部坡度

★ 这个实验的价值不在数字，在**结论的方向**：
  如果落点随位置大幅摆动，那"平衡车速"就**不是车的属性**，而是
  「车 × 测点局部坡度」的属性 —— 报一个不带坡度的稳态速度是没有意义的。

★ 也是对自己的一次证伪：之前写下的"跑步机系统性偏高"若在这里站不住，
  就该被撤回，而不是被继续引用。

跑法
----
    PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe 11_grade_sweep.py
    ... --theta 0.40 --positions 0,200,400,600 --vehicle vehicle.tesla.model3

⚠ 需要地图 Town04。0.9.15 打包版**忽略命令行地图参数**，
  换图只能改 DefaultEngine.ini 后重启：`python carla_map.py set Town04`
"""

import argparse
import csv
import importlib.util
import json
import math
import os
import time

import carla

from carla_env import ensure_carla, slug, speed_of

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_REQUIRED = "Town04"
STRAIGHT_START = (-364.3, 37.1)   # 与 10_free_drive_repro.py 同一个直道起点
GRADE_SPAN = 5.0                  # 前后各 5 m 估坡度


def _load07():
    """统计与跑步机原语**只留一份实现**（P-SIM-02）。"""
    p = os.path.join(HERE, "07_low_speed_balance.py")
    spec = importlib.util.spec_from_file_location("bal07", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


bal07 = _load07()
DT = bal07.DT


# ------------------------- 直道几何 -------------------------
def heading(wp):
    f = wp.transform.get_forward_vector()
    return math.degrees(math.atan2(f.y, f.x))


def walk_along(wp, offsets, step=5.0):
    """沿**同一车道**前进，返回 {offset: waypoint}；直路断了就提前返回。"""
    out, pos, cur = {}, 0.0, wp
    for target in sorted(offsets):
        while pos < target - 1e-6:
            d = min(step, target - pos)
            nxt = cur.next(d)
            same = [w for w in nxt if w.lane_id == cur.lane_id]
            if not same:
                return out, f"在 {pos:.0f} m 处车道断了"
            cur = same[0]
            pos += d
        out[target] = cur
    return out, "ok"


def local_grade(wp, span=GRADE_SPAN):
    """局部纵坡 (%)：取前后 span 米的高度差 / 水平距离。正值 = 上坡。"""
    fwd = [w for w in wp.next(span) if w.lane_id == wp.lane_id]
    back = [w for w in wp.previous(span) if w.lane_id == wp.lane_id]
    if not fwd or not back:
        return None
    a, b = back[0].transform.location, fwd[0].transform.location
    dx = math.hypot(b.x - a.x, b.y - a.y)
    return (b.z - a.z) / dx * 100.0 if dx > 1e-6 else None


def wp_pitch(wp):
    """waypoint 自带 pitch（度），与局部坡度互为校验。

    ★ 必须归一到 (-180, 180]：CARLA 的 rotation.pitch 会回绕成 +360 / +356 这种，
      直接打印出来像"翻了个跟头"，会让人误判成几何取错。
    """
    return (wp.transform.rotation.pitch + 180.0) % 360.0 - 180.0


# ------------------------- 主流程 -------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="vehicle.tesla.model3")
    ap.add_argument("--theta", type=float, default=0.40)
    ap.add_argument("--positions", default="0,200,400,600")
    ap.add_argument("--warmup", type=int, default=bal07.WARMUP)
    ap.add_argument("--window", type=int, default=bal07.WINDOW)
    ap.add_argument("--pre-spin-theta", type=float, default=bal07.PRE_SPIN_THETA)
    ap.add_argument("--pre-spin-v", type=float, default=bal07.PRE_SPIN_V)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    offsets = [int(x) for x in args.positions.split(",") if x.strip()]
    out_dir = args.out or os.path.join(HERE, "calib", "grade_sweep")
    os.makedirs(out_dir, exist_ok=True)

    world, map_name = ensure_carla(map_name=MAP_REQUIRED)
    short = map_name.split("/")[-1]
    if short != MAP_REQUIRED:
        print(f"❌ 当前图 {short}，需要 {MAP_REQUIRED}（换图见 carla_map.py）")
        return 1
    print(f"[carla] map = {map_name}")

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DT
    world.apply_settings(settings)

    # ★ 冲刺初速必须记进 summary：降速落点**依赖它**（实测 16/20/25 m/s 给出
    #   10.93 / 17.10 / 17.64）。不记初条件的"落点"不可复现 —— 这条比数字本身重要。
    summary = {"map": map_name, "vehicle": args.vehicle, "theta": args.theta,
               "warmup": args.warmup, "window": args.window,
               "pre_spin_theta": args.pre_spin_theta,
               "pre_spin_v": args.pre_spin_v,
               "pre_spin_steps": bal07.PRE_SPIN_STEPS,
               "straight_start": list(STRAIGHT_START), "points": []}

    try:
        try:
            bp = world.get_blueprint_library().find(args.vehicle)
        except IndexError:
            print(f"❌ 找不到车型 {args.vehicle}")
            return 1

        # 起点：离 STRAIGHT_START 最近的出生点所在车道
        m = world.get_map()
        tgt = carla.Location(x=STRAIGHT_START[0], y=STRAIGHT_START[1])
        sp0 = min(m.get_spawn_points(), key=lambda s: s.location.distance(tgt))
        wp0 = m.get_waypoint(sp0.location)
        print(f"[geom] 直道起点 ({sp0.location.x:.1f}, {sp0.location.y:.1f}) "
              f"lane={wp0.lane_id} 航向={heading(wp0):.1f}°")

        wps, note = walk_along(wp0, offsets)
        if note != "ok":
            print(f"⚠ {note}；拿到 {sorted(wps)}")
        for off in offsets:
            if off not in wps:
                continue
            wp = wps[off]
            grade = local_grade(wp)
            pitch = wp_pitch(wp)
            tf = wp.transform
            tf.location.z += 0.3          # 抬高一点避免 spawn 卡进地面

            print(f"\n===== 落点 {off} m | 坡度 {grade:+.2f}% "
                  f"(pitch {pitch:+.2f}°) =====")
            rec = {"offset_m": off, "grade_pct": round(grade, 3),
                   "wp_pitch_deg": round(pitch, 3),
                   "loc": [round(tf.location.x, 2), round(tf.location.y, 2),
                           round(tf.location.z, 2)]}

            for mode in ("up", "down"):
                t0 = time.time()
                rows, note2 = bal07.run_balance(
                    world, bp, tf, args.theta, mode, args.warmup, args.window,
                    pre_spin_theta=args.pre_spin_theta,
                    pre_spin_v=args.pre_spin_v)
                if not rows:
                    print(f"  {mode:<4} ❌ {note2}")
                    rec[mode] = {"v": None, "note": note2}
                    continue
                rf = os.path.join(out_dir, f"g{off:03d}_{mode}.csv")
                with open(rf, "w", newline="", encoding="utf-8") as f:
                    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                    w.writeheader()
                    w.writerows(rows)
                s = bal07.window_stats(rows, args.window)
                rec[mode] = {"v": round(s["v_mean"], 4),
                             "std": round(s["v_std"], 4),
                             "class": bal07.classify(s),
                             "n": len(rows), "note": note2}
                print(f"  {mode:<4} v={s['v_mean']:7.3f} σ={s['v_std']:6.3f} "
                      f"{bal07.classify(s):<6} [{note2}]  {time.time()-t0:.0f}s")

            up = rec.get("up", {}).get("v")
            dn = rec.get("down", {}).get("v")
            rec["delta"] = round(dn - up, 4) if (up is not None and dn is not None) else None
            summary["points"].append(rec)

    finally:
        try:
            settings.synchronous_mode = False
            world.apply_settings(settings)
            print("\n[carla] 同步模式已关闭。")
        except Exception as e:
            print(f"[carla] ⚠ 收尾失败（服务端可能已崩）：{e}")

    if not summary["points"]:
        print("[11] 没有拿到任何数据")
        return 1

    jf = os.path.join(out_dir, "summary.json")
    with open(jf, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 66)
    print(f"{'位置':>6} {'坡度%':>8} {'升速':>8} {'降速':>8} {'Δ':>8}")
    for r in summary["points"]:
        print(f"{r['offset_m']:>6} {r['grade_pct']:>+8.2f} "
              f"{str(r.get('up', {}).get('v')):>8} "
              f"{str(r.get('down', {}).get('v')):>8} {str(r.get('delta')):>8}")
    dn = [r["down"]["v"] for r in summary["points"]
          if r.get("down", {}).get("v") is not None]
    if dn:
        print(f"\n降速落点 {min(dn):.2f} ~ {max(dn):.2f} m/s"
              f"（极差 {max(dn)-min(dn):.2f}）")
    print(f"[out] {jf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
