"""08_probe_physics.py -- 把 CARLA 内置车辆模型的**真实参数**读出来。

为什么要这个
------------
03 的 write_meta 里写过一句"CARLA 0.9.15 蓝图不暴露物理参数（mass/drag/torque）"。
那句话**只对 blueprint attribute 成立** —— 实测 `bp.get_attribute('mass')` 抛异常。

但车辆生成之后还有一条路：`vehicle.get_physics_control()`。
本脚本就是去核实它是空的还是有真东西。这个区别很重要：

  拿不到  -> CARLA 的车是个黑盒，标定表只能"描述它"，
             无法回答"这个车模型像不像实车"
  拿得到  -> 可以①判断内置模型的保真度 ②把实车参数**写进去**替换

回答的问题
----------
  Q1 CARLA 的车辆动力学模型是它自带的吗？参数在哪？
  Q2 它的参数是真车量级，还是随手编的？
  Q3 换车型时，是"换一组参数"还是"换一个模型"？

跑法：
  PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe 08_probe_physics.py
  ... --vehicles vehicle.tesla.model3,vehicle.nissan.patrol
"""

import argparse
import json
import os
import sys

import carla
from carla_env import HOST, PORT, slug

# 直接读 get_physics_control() 的标量字段名。用 getattr 取，
# 缺字段就记 None —— 不猜、不编，读不到就是读不到。
SCALARS = [
    "mass",                 # 整车质量 kg
    "drag_coefficient",     # 风阻系数（配合 default 的迎风面积用）
    "max_rpm",              # 发动机/电机最高转速
    "moi",                  # 转动惯量
    "final_ratio",          # 主减速比
    "gear_switch_time",     # 换挡耗时 s
    "clutch_strength",      # 离合器强度
    "use_gear_autobox",     # 是否自动挡
    "damping_rate_full_throttle",
    "damping_rate_zero_throttle_clutch_engaged",
    "damping_rate_zero_throttle_clutch_disengaged",
]

DEFAULT = ["vehicle.micro.microlino", "vehicle.audi.a2", "vehicle.tesla.model3",
           "vehicle.nissan.patrol", "vehicle.carlamotors.carlacola"]


def read_one(world, vid, spawn_pt):
    """spawn 一台车，读出 physics control，立刻销毁。"""
    try:
        bp = world.get_blueprint_library().find(vid)
    except IndexError:
        return {"vehicle": vid, "error": "蓝图不存在"}
    v = world.try_spawn_actor(bp, spawn_pt)
    if v is None:
        return {"vehicle": vid, "error": "spawn 失败"}
    try:
        pc = v.get_physics_control()
        out = {"vehicle": vid}
        for k in SCALARS:
            out[k] = getattr(pc, k, None)
        # 挡位：这个直接决定"同一油门多个平衡点"这类现象
        fg = getattr(pc, "forward_gears", None)
        out["n_forward_gears"] = len(fg) if fg else 0
        out["gear_ratios"] = [round(g.ratio, 3) for g in fg] if fg else []
        # 扭矩曲线：取包络（最大扭矩 + 峰值转速）
        tc = getattr(pc, "torque_curve", None)
        if tc:
            out["torque_peak_nm"] = round(max(p.y for p in tc), 1)
            out["torque_peak_at_rpm"] = round(
                max(tc, key=lambda p: p.y).x, 0)
            out["torque_curve_pts"] = len(tc)
        # 轮胎：单一 mu，没有 Pacejka 之类的模型
        ws = getattr(pc, "wheels", None)
        if ws:
            out["n_wheels"] = len(ws)
            out["tire_friction"] = [round(w.tire_friction, 2) for w in ws]
            out["wheel_radius_m"] = round(ws[0].radius, 3)
            out["max_brake_torque"] = round(ws[0].max_brake_torque, 1)
        return out
    finally:
        v.destroy()
        world.tick()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicles", default="", help="逗号分隔；默认跑质量阶梯 5 台")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "calib"))
    args = ap.parse_args()
    vids = [v.strip() for v in args.vehicles.split(",") if v.strip()] or DEFAULT

    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)
    world = client.get_world()
    print(f"[carla] map = {world.get_map().name}")

    # 探针不需要 tick；显式切到异步，免得服务端还停在同步模式里等 tick
    st = world.get_settings()
    was_sync = st.synchronous_mode
    st.synchronous_mode = False
    world.apply_settings(st)

    spawn_pt = world.get_map().get_spawn_points()[0]
    rows = []
    try:
        for vid in vids:
            r = read_one(world, vid, spawn_pt)
            rows.append(r)
            print(f"[probe] {vid}: " +
                  ("❌ " + r["error"] if "error" in r else "ok"))
    finally:
        st.synchronous_mode = was_sync
        world.apply_settings(st)

    ok = [r for r in rows if "error" not in r]
    if not ok:
        print("[probe] 一台都没读到")
        return 1

    print("\n" + "=" * 108)
    print("CARLA 内置车辆模型：get_physics_control() 实测")
    print("=" * 108)
    # ⚠ 单位陷阱：WheelPhysicsControl.radius 用的是**厘米**，不是米
    #   （tesla 报 37 -> 0.37 m，正好是 Model 3 的轮胎半径；按米读就成了 37 米的轮子）
    hdr = ["vehicle", "mass(kg)", "drag", "max_rpm", "gears", "final",
           "扭矩峰值(Nm)", "@rpm", "胎mu", "轮半径(cm)", "刹车力矩"]
    print(f"{hdr[0]:<30}" + "".join(f"{h:>11}" for h in hdr[1:]))
    print("-" * 108)
    for r in ok:
        mu = r.get("tire_friction") or []
        print(f"{r['vehicle'].replace('vehicle.',''):<30}"
              f"{r.get('mass', 0):>11.0f}"
              f"{r.get('drag_coefficient', 0):>11.3f}"
              f"{r.get('max_rpm', 0):>11.0f}"
              f"{r.get('n_forward_gears', 0):>11d}"
              f"{r.get('final_ratio', 0):>11.2f}"
              f"{r.get('torque_peak_nm', 0):>11.0f}"
              f"{r.get('torque_peak_at_rpm', 0):>11.0f}"
              f"{(mu[0] if mu else 0):>11.2f}"
              f"{r.get('wheel_radius_m', 0):>11.3f}"
              f"{r.get('max_brake_torque', 0):>11.0f}")

    print("\n挡位传动比（「同一油门多个平衡点」的直接嫌疑对象）：")
    for r in ok:
        print(f"  {r['vehicle'].replace('vehicle.',''):<26} "
              f"自动挡={r.get('use_gear_autobox')}  "
              f"换挡耗时={r.get('gear_switch_time')}s  "
              f"传动比={r.get('gear_ratios')}")

    p = os.path.join(args.out, "carla_vehicle_physics.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump({"map": world.get_map().name, "vehicles": rows}, f,
                  ensure_ascii=False, indent=2)
    print(f"\n[out] {p}")
    print("\n判读提示：")
    print("  · torque_curve_pts 少（几个点）= 分段线性拟合，不是真扭矩曲线")
    print("  · 每轮只有一个 tire_friction = 简化轮胎模型（无滑移刚度/松弛长度/温度）")
    print("  · n_forward_gears > 1 + use_gear_autobox = 挡位是**内部状态**，")
    print("    踏板→速度因此可以不是单值映射（本项目的 16 m/s 多稳态）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
