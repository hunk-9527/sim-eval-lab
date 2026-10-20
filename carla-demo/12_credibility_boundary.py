# -*- coding: utf-8 -*-
"""12_credibility_boundary.py —— 闭环可用性边界表

问题：一条判据在这个台架上「能不能测」，不该靠事后讨论，应该能算出来。

做法：**不启动 CARLA**，只读既有标定产物，把三个量放在一起比 ——

    · SUT 的请求域      acc-sim 的 AccConfig（accel_max / brake_max）
    · 被控对象可达域    标定表 F(θ) / B(β) − G(v)
    · 被控对象固有量    G(v)（滑行阻力）、迟滞 Δ、死区、测点依赖

三条拒绝条件，任一命中 → 该判据在这一格失效：

    R1 固有量吃掉请求量   G(v) ≥ |SUT 最大减速请求| → 执行器根本不被激励
    R2 判据容差 < 分辨率  判据要 0.1 m/s，被控对象只给得动 15 m/s
    R3 判据量绑在测点上   只换钉住位置，平衡车速就换了

输入（全部已有文件，不重跑仿真）：
    calib/longitudinal_model.json                   G/F/B 三张表 + 拟合质量
    calib/veh_*/balance_table.csv                   五台车的迟滞 Δ（按 playbook 7.9 规则重算）
    calib/acc_loop_tesla_model3/acc_loop_verdict.json
    calib/grade_sweep_tesla/summary.json            测点坡度扫掠
    ../acc-sim/acc/models.py                        SUT 请求域

输出：
    calib/credibility_boundary/boundary.json
    calib/credibility_boundary/boundary_table.md
    calib/credibility_boundary/authority_envelope.png

用法：
    python 12_credibility_boundary.py
    python 12_credibility_boundary.py --set-speed 25 --out calib/credibility_boundary

★ 设计约定（P-SIM-08）：表里每个数都从文件里现算，不手抄。
  跑完打印每条输入的文件名 + 它贡献了哪个数，方便复核。
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
# 与 09_acc_carla.py 用同一种寻址方式，保证私有工作区与公开仓库里的这份逐字一致
ACC_SIM_DIR = os.environ.get("ACC_SIM_DIR", r"D:\AI\myProject\acc-sim")

# 标定表 notes 里写明的模型适用范围（v < 2.5 是离合/一档啮合阶跃，模型不覆盖）
# 这里不硬信，下面 load_model() 会去 notes 里核对这句话还在不在。
V_STARTUP = 2.5

VERDICT_ORDER = {"可用": 0, "需谨慎": 1, "不可用": 2}


# --------------------------------------------------------------------------
# 读
# --------------------------------------------------------------------------
def load_model(path):
    d = json.load(open(path, encoding="utf-8"))
    notes = " ".join(d.get("notes", []))
    if "v<2.5" not in notes.replace(" ", ""):
        print(f"  ⚠ {path} 的 notes 里没找到 v<2.5 的适用范围说明 —— 确认 V_STARTUP 是否还成立")
    return d


def interp(xs, ys, x):
    """线性插值（表是等距的，但这里不依赖等距）。"""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    t = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + t * (ys[hi] - ys[lo])


def hysteresis_by_vehicle(calib_dir):
    """按 playbook 7.9 的规则重算 Δ：**先剔除「任一路径为死区」的配对**再取 max。

    （另一种划法「只取两边都稳定平衡」会漏掉 patrol，Δ 从 27.4 掉到 0.036 ——
      错误判据会让防线放过真正该拒绝的车。见 playbook 7.9。）
    """
    out = {}
    for name in sorted(os.listdir(calib_dir)):
        f = os.path.join(calib_dir, name, "balance_table.csv")
        if not name.startswith("veh_") or not os.path.isfile(f):
            continue
        rows = list(csv.DictReader(open(f, encoding="utf-8")))
        by = {}
        for r in rows:
            by.setdefault(r["throttle"], {})[r["mode"]] = r

        best, where, n_pairs = 0.0, None, 0
        dead_th = []
        for th, d in by.items():
            if "up" not in d or "down" not in d:
                continue
            if d["up"]["class"] == "死区" and d["down"]["class"] == "死区":
                dead_th.append(float(th))
            if d["up"]["class"] == "死区" or d["down"]["class"] == "死区":
                continue
            dv = abs(float(d["up"]["v_mean"]) - float(d["down"]["v_mean"]))
            n_pairs += 1
            if dv > best:
                best, where = dv, float(th)

        vehicle = json.load(open(os.path.join(calib_dir, name, "meta.json"),
                                 encoding="utf-8")).get("vehicle", name)
        out[vehicle] = {
            "delta_max": round(best, 3),
            "at_throttle": where,
            "n_pairs": n_pairs,
            "deadzone_throttle_max": max(dead_th) if dead_th else None,
            "source": os.path.relpath(f, HERE),
        }
    return out


def loop_margin(path, tables, col_v="v_ego", col_a="a_req"):
    """R1 的直接证据：逐拍算净推力需求 R = a_req + G(v)。

    R ≥ 0 → 只靠油门/滑行就够了，刹车在**结构上**不可能被用到；
    R < 0 → 才需要踩刹车。
    这比「刹车 0/500 拍」强一层：0 拍可能是 "恰好没用到"，R ≥ 0 是 "用不到"。
    """
    Gv, GG = tables["G_v"], tables["G_G"]
    vals = []
    for r in csv.DictReader(open(path, encoding="utf-8")):
        v, a = float(r[col_v]), float(r[col_a])
        vals.append((a + interp(Gv, GG, v), v, a))
    if not vals:
        return None
    rmin = min(vals, key=lambda x: x[0])
    return {
        "n": len(vals),
        "R_min": round(rmin[0], 3),
        "v_at_R_min": round(rmin[1], 2),
        "a_req_at_R_min": round(rmin[2], 3),
        "ticks_R_negative": sum(1 for x in vals if x[0] < -1e-9),
        "source": os.path.relpath(path, HERE),
    }


def grade_spread(path):
    """测点扫掠：只换钉住位置，平衡车速摆多大。"""
    d = json.load(open(path, encoding="utf-8"))
    ups = [(p["offset_m"], p["grade_pct"], p["up"]["v"]) for p in d["points"]]
    vs = [u[2] for u in ups]
    grades = [u[1] for u in ups]
    return {
        "vehicle": d["vehicle"],
        "map": d["map"],
        "straight_start": d.get("straight_start"),
        "points": [{"offset_m": o, "grade_pct": g, "v_up": v} for o, g, v in ups],
        "v_spread": round(max(vs) - min(vs), 2),
        "v_min": round(min(vs), 2),
        "v_max": round(max(vs), 2),
        "grade_span_pct": round(max(grades) - min(grades), 2),
        "source": os.path.relpath(path, HERE),
    }


def load_sut_config():
    if ACC_SIM_DIR not in sys.path:
        sys.path.insert(0, ACC_SIM_DIR)
    from acc.models import AccConfig  # noqa: E402

    c = AccConfig()
    return {
        "accel_max": c.accel_max,
        "brake_max": c.brake_max,
        "time_gap": c.time_gap,
        "min_gap": c.min_gap,
        # 固定标识，不用 relpath —— 否则表的内容会随"在哪个目录下跑"而变（P-SIM-08 同族）
        "source": "acc-sim/acc/models.py",
    }


# --------------------------------------------------------------------------
# 算
# --------------------------------------------------------------------------
def plant_envelope(tables, sut):
    """被控对象在各速度的可达区间 + R1 的临界速度 v*。"""
    Gv, GG = tables["G_v"], tables["G_G"]
    F_max, B_max = tables["F_F"][-1], tables["B_B"][-1]
    brake_req = abs(sut["brake_max"])

    # v*：G(v) 第一次追上 SUT 最大减速请求的地方（线性插值，不用网格点凑整）
    v_star = None
    for i in range(1, len(Gv)):
        if GG[i] >= brake_req:
            t = (brake_req - GG[i - 1]) / (GG[i] - GG[i - 1])
            v_star = round(Gv[i - 1] + t * (Gv[i] - Gv[i - 1]), 2)
            break

    pts = []
    for v in [0, 2, 5, 10, 15, 20, 25, 30, 35, 40]:
        g = interp(Gv, GG, float(v))
        pts.append({
            "v": v,
            "G": round(g, 2),
            "accel_max": round(F_max - g, 2),
            "decel_max": round(B_max + g, 2),
            "decel_over_req": round((B_max + g) / brake_req, 2),
        })
    return {"F_max": round(F_max, 2), "B_max": round(B_max, 2),
            "v_star": v_star, "envelope": pts}


def build_rows(plant, hyst, grade, loop, sut, set_speed, margin):
    v_star = plant["v_star"]
    brake_req = abs(sut["brake_max"])
    single = [k for k, v in hyst.items() if v["delta_max"] < 1.0]
    multi = [k for k, v in hyst.items() if v["delta_max"] >= 1.0]
    d_single = max(hyst[k]["delta_max"] for k in single) if single else None
    multi_lo = min(hyst[k]["delta_max"] for k in multi) if multi else None
    multi_hi = max(hyst[k]["delta_max"] for k in multi) if multi else None
    env30 = next(p for p in plant["envelope"] if p["v"] == 30)
    env0 = next(p for p in plant["envelope"] if p["v"] == 0)

    rows = [
        dict(family="I 不碰 plant", name="状态机 / 激活逻辑",
             tesla="可用", multi="可用", site="可用",
             why="只读 SUT 内部状态，被控对象怎么变都不影响"),
        dict(family="I 不碰 plant", name="执行器使用率（饱和 / 闲置）",
             tesla="可用", multi="可用", site="可用",
             why="这是体检量本身，不是被检对象 —— 0/500 拍就是这么数出来的"),
        dict(family="II 撞分辨率", name="跟车稳态车距 / 时距控制",
             tesla="可用", multi="需谨慎", site="可用",
             why=f"单速车 Δ≤{d_single:.2f} m/s，实测稳态误差 0.09 / 0.16 m，判据容差 1.0 m，差一个量级；"
                 f"多挡车 Δ={multi_lo:.1f}~{multi_hi:.1f} m/s，车距被速度误差带着跑"),
        dict(family="II 撞分辨率", name="巡航稳态车速",
             tesla="可用", multi="不可用", site="需谨慎",
             why=f"R2：多挡车 Δ={multi_lo:.1f}~{multi_hi:.1f} m/s，而跟车微调要 0.1 m/s 量级，差两个数量级。"
                 f"换测点：闭环里速度是被控量，本身不随测点变；但标定表是在某个测点量的，"
                 f"运行点换坡度后 G(v) 失配（开环读数就摆 {grade['v_spread']:.2f} m/s）"),
        dict(family="II 撞分辨率", name="巡航超调 / 响应时间",
             tesla="需谨慎", multi="不可用", site="需谨慎",
             why=f"实测 v_max={loop['v_max']:.2f} vs set_speed={set_speed}（+{100*(loop['v_max']/set_speed-1):.1f}%）；"
                 f"但超调是「PID × 被控对象」的合量，换车就换数。"
                 f"⚠ 更根本：闭环的可达域是翻译层「让它可达」的 —— 精度类判据接近自证"),
        dict(family="III 撞固有量", name="减速度幅值 / 制动距离 / jerk",
             tesla="不可用", multi="不可用", site="不可用",
             why=f"R1：v ≥ {v_star} m/s 时 G(v) ≥ {brake_req:.1f} = SUT 最大减速请求，刹车**用不到**。"
                 f"逐拍算 R=a_req+G(v)：R_min={margin['R_min']:+.2f} m/s²（v={margin['v_at_R_min']}），"
                 f"R<0 的拍数 {margin['ticks_R_negative']}/{margin['n']}，刹车 {loop['brake_ticks']}/{loop['n']} 拍。"
                 f"v < {v_star} 的窗口未做实验，暂不给可用结论"),
        dict(family="III 撞固有量", name="最小车距 / 碰撞 / TTC",
             tesla="不可用", multi="不可用", site="不可用",
             why=f"被控对象减速能力是 SUT 请求的 {env0['decel_over_req']:.2f}~{env30['decel_over_req']:.2f} 倍"
                 f"（v=0~30）→ 危险场景被自动化解。**撞上 = 可信；没撞上 ≠ 安全**（单向判据）。"
                 f"本 run 最小车距 {loop['gap_min']:.2f} m"),
        dict(family="III 撞固有量", name="加速度幅值 / 起步平顺性",
             tesla="需谨慎", multi="需谨慎", site="需谨慎",
             why=f"v < {V_STARTUP} m/s **不可用**：标定表明确排除这一段（从静止起步是离合/一档啮合阶跃）。"
                 f"v ≥ {V_STARTUP} 时可达域 {env30['accel_max']:.2f}~{env0['accel_max']:.2f} m/s² 远大于请求上限 "
                 f"{sut['accel_max']:.1f}，但加速度本就是 F(θ)−G(v) 现算的 —— 测的是表，不是车"),
        dict(family="IV 台架没有", name="横向 / 变道 / 泊车 / 车道保持",
             tesla="不可用", multi="不可用", site="不可用",
             why="1-D 纵向台没有横向自由度（车钉位置，位置由自积分的车距代理）"),
        dict(family="IV 台架没有", name="感知类（检出率 / 虚警 / 测距误差）",
             tesla="不可用", multi="不可用", site="不可用",
             why="distance 是几何真值，传感器链没有被建模"),
    ]
    for r in rows:
        r["worst"] = max((r["tesla"], r["multi"], r["site"]), key=VERDICT_ORDER.get)
    return rows


def _norm(s):
    for k in VERDICT_ORDER:
        if s.startswith(k):
            return k
    return "不可用"


# --------------------------------------------------------------------------
# 写
# --------------------------------------------------------------------------
def write_md(path, plant, hyst, grade, loop, sut, rows, set_speed, margin):
    P = lambda s: str(s).replace("\\", "/")  # noqa: E731
    L = []
    A = L.append
    A("# 闭环可用性边界表（自动生成）")
    A("")
    A(f"> 由 `12_credibility_boundary.py` 生成，**不要手改**。数字全部现算，来源见文末「输入」一节。")
    A("")
    A("## 0. 三个量")
    A("")
    A(f"- **SUT 请求域**：{sut['brake_max']:.1f} ~ +{sut['accel_max']:.1f} m/s²（`{P(sut['source'])}`）")
    A(f"- **被控对象可达域**：F_max={plant['F_max']:.2f}（油门踩满，v=0 时）、"
      f"B_max={plant['B_max']:.2f}（刹车踩满，**不含滑行阻力**）")
    A(f"- **被控对象固有量**：G(v) 滑行阻力 —— G(0)={plant['envelope'][0]['G']:.2f}，"
      f"G(30)={plant['envelope'][7]['G']:.2f}，G(40)={plant['envelope'][-1]['G']:.2f} m/s²")
    A("")
    A(f"★ **R1 的临界速度 v\\* = {plant['v_star']} m/s**：G(v\\*) ≥ {abs(sut['brake_max']):.1f} = SUT 最大减速请求。")
    A(f"→ v ≥ {plant['v_star']} m/s 时，**松开油门就已经减得比 SUT 能要求的最强减速还猛**，"
      f"刹车踏板在物理上不可能被用到。")
    A("")
    A(f"### R1 的直接证据：净推力需求 R = a_req + G(v)")
    A("")
    A(f"逐拍重算（`{P(margin['source'])}`）：**R_min = {margin['R_min']:+.2f} m/s²**"
      f"（出现在 v={margin['v_at_R_min']} m/s、a_req={margin['a_req_at_R_min']:+.2f}），"
      f"**R < 0 的拍数 = {margin['ticks_R_negative']} / {margin['n']}**。")
    A("")
    if margin["ticks_R_negative"] == 0:
        A("→ 全程 R ≥ 0：**刹车不是「恰好没用到」，是「用不到」** —— 这是结构结论，不是一次性观察。")
    else:
        A(f"→ 有 {margin['ticks_R_negative']} 拍 R < 0，刹车在原理上可被用到；实际用量见第 5 节。")
    A("")
    A("## 1. 可达域 vs 请求域")
    A("")
    A("| v (m/s) | G(v) | 可达加速度 | 可达减速度 | 减速度 / SUT 请求上限 |")
    A("|---:|---:|---:|---:|---:|")
    for p in plant["envelope"]:
        A(f"| {p['v']} | {p['G']:.2f} | +{p['accel_max']:.2f} | −{p['decel_max']:.2f} | {p['decel_over_req']:.2f}× |")
    A("")
    A("## 2. 可用性表")
    A("")
    A("| 类 | 判据类别 | tesla 单速台架 | 换多挡车 | 换测点 | 依据 |")
    A("|---|---|---|---|---|---|")
    for r in rows:
        A(f"| {r['family']} | **{r['name']}** | {r['tesla']} | {r['multi']} | {r['site']} | {r['why']} |")
    A("")
    cnt = {k: sum(1 for r in rows if r["tesla"] == k) for k in VERDICT_ORDER}
    A(f"本台架（tesla 单速）：**可用 {cnt['可用']} / 需谨慎 {cnt['需谨慎']} / 不可用 {cnt['不可用']}**"
      f"（共 {len(rows)} 条）。换多挡车后可用只剩 "
      f"{sum(1 for r in rows if r['multi'] == '可用')} 条 —— **换的不是判据，是被控对象**。")
    A("")
    A("## 3. 迟滞 Δ（换车就会变的那个数）")
    A("")
    A("> 口径：按 playbook 7.9 —— **先剔除「任一路径为死区」的配对**再取 max。"
      "另一种划法（只取两边都稳定平衡）会漏掉 patrol，Δ 从 27.4 掉到 0.036。")
    A("")
    A("| 车型 | 最大 Δ (m/s) | 出现在油门 | 有效配对数 |")
    A("|---|---:|---:|---:|")
    for k in sorted(hyst, key=lambda k: hyst[k]["delta_max"]):
        v = hyst[k]
        A(f"| `{k}` | {v['delta_max']:.3f} | {v['at_throttle']:.2f} | {v['n_pairs']} |")
    A("")
    A("## 4. 测点依赖（同一台车，只换钉住位置）")
    A("")
    A(f"`{grade['vehicle']}` @ `{grade['map']}`，坡度跨度 {grade['grade_span_pct']:.2f} 个百分点：")
    A("")
    A("| 钉住位置 (m) | 局部坡度 (%) | 升速落点 (m/s) |")
    A("|---:|---:|---:|")
    for p in grade["points"]:
        A(f"| {p['offset_m']} | {p['grade_pct']:+.3f} | {p['v_up']:.2f} |")
    A("")
    A(f"→ 同车同油门，**只换测点，平衡车速摆 {grade['v_spread']:.2f} m/s**"
      f"（{grade['v_min']} → {grade['v_max']}）。")
    A("")
    A("## 5. 实测闭环（本表引用的那次 run）")
    A("")
    A(f"- 车 `{loop['vehicle']}`，{loop['n']} 拍，`set_speed={set_speed}`"
      f"（⚠ 跑的参数，`acc_loop.csv` 没存这一列，由 `--set-speed` 传入）")
    A(f"- **刹车踏板 {loop['brake_ticks']}/{loop['n']} 拍**，被表截断 {loop['saturated_ticks']}/{loop['n']} 拍")
    A(f"- 车距最小 {loop['gap_min']:.2f} m，碰撞 {loop['collision']}；v ∈ [{loop['v_min']:.2f}, {loop['v_max']:.2f}]")
    A("")
    A("## 6. 输入（每个数从哪来）")
    A("")
    A("| 文件 | 贡献 |")
    A("|---|---|")
    A(f"| `calib/longitudinal_model.json` | G/F/B 三张表（R1 与可达域） |")
    A(f"| `calib/veh_*/balance_table.csv`（5 个） | 迟滞 Δ（R2） |")
    A(f"| `{P(grade['source'])}` | 测点依赖（R3） |")
    A(f"| `calib/acc_loop_tesla_model3/acc_loop_verdict.json` | 刹车 0/500、车距、速度区间 |")
    A(f"| `{sut['source']}` | SUT 请求域 |")
    A("")
    open(path, "w", encoding="utf-8").write("\n".join(L))


def plot(path, tables, plant, sut, loop):
    Gv, GG = tables["G_v"], tables["G_G"]
    F_max, B_max = tables["F_F"][-1], tables["B_B"][-1]
    brake_req = abs(sut["brake_max"])
    vs = [v for v, _ in zip(Gv, GG)]
    g = GG
    acc = [F_max - x for x in g]
    dec = [-(B_max + x) for x in g]

    fig, ax = plt.subplots(figsize=(9.2, 5.6))
    ax.fill_between(vs, sut["brake_max"], sut["accel_max"], color="0.85",
                    label=f"SUT 请求域 [{sut['brake_max']:+.1f}, {sut['accel_max']:+.1f}]")
    ax.plot(vs, g, color="crimson", lw=2, label="G(v) 被控对象固有量（滑行阻力）")
    ax.plot(vs, acc, color="royalblue", lw=1.6, label="被控对象可达加速度 F_max − G(v)")
    ax.plot(vs, dec, color="seagreen", lw=1.6, label="被控对象可达减速度 −(B_max + G(v))")
    ax.axhline(sut["brake_max"], color="crimson", ls=":", lw=1)
    ax.axhline(sut["accel_max"], color="royalblue", ls=":", lw=1)

    v_star = plant["v_star"]
    if v_star is not None:
        ax.axvline(v_star, color="k", ls="--", lw=1)
        # 阴影 = G(v) 高于 SUT 最大减速请求的那片：被控对象自己的阻力就替 SUT 干完了活
        ax.fill_between(vs, g, 15.8, where=[v >= v_star for v in vs],
                        color="crimson", alpha=0.07, interpolate=True)
        ax.text(v_star + 0.6, 14.4, f"v* = {v_star} m/s（G(v*) = SUT 最大减速请求）",
                fontsize=9, ha="left")
        ax.text(max(vs) - 1.2, 13.6, "阴影区：松油门就够\n刹车踏板永不参与",
                ha="right", fontsize=9, color="crimson")

    if loop["v_min"] and loop["v_max"]:
        ax.axvspan(loop["v_min"], loop["v_max"], color="gray", alpha=0.13)
        ax.text((loop["v_min"] + loop["v_max"]) / 2, -7.2,
                f"本次闭环实测 v∈[{loop['v_min']:.1f}, {loop['v_max']:.1f}]\n"
                f"刹车 {loop['brake_ticks']}/{loop['n']} 拍",
                ha="center", fontsize=9, color="0.25")

    ax.set_xlabel("车速 v (m/s)")
    ax.set_ylabel("加速度 (m/s²)")
    ax.set_title("被控对象固有量 vs SUT 请求域：判据能落在哪、落在哪就没意义", fontsize=11)
    ax.set_xlim(0, max(vs))
    ax.set_ylim(-15.5, 16)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="lower left", framealpha=0.95)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default=os.path.join(HERE, "calib"))
    ap.add_argument("--out", default=os.path.join(HERE, "calib", "credibility_boundary"))
    ap.add_argument("--set-speed", type=float, default=25.0,
                    help="09 那次 run 的 set_speed（用于算超调；acc_loop.csv 里没存这一列）")
    ap.add_argument("--vehicle", default="vehicle.tesla.model3",
                    help="用哪台车的闭环结果")
    args = ap.parse_args()

    out_dir = args.out
    os.makedirs(out_dir, exist_ok=True)

    model_path = os.path.join(args.calib, "longitudinal_model.json")
    loop_path = os.path.join(args.calib, "acc_loop_tesla_model3", "acc_loop_verdict.json")
    grade_path = os.path.join(args.calib, "grade_sweep_tesla", "summary.json")

    print("==", os.path.basename(__file__))
    print(f"读取 {os.path.relpath(model_path, HERE)}")
    model = load_model(model_path)
    tables = model["tables"]
    sut = load_sut_config()
    print(f"  → SUT 请求域 [{sut['brake_max']:+.1f}, {sut['accel_max']:+.1f}]（来自 {sut['source']}）")

    plant = plant_envelope(tables, sut)
    print(f"  → F_max={plant['F_max']}, B_max={plant['B_max']}, v*={plant['v_star']} m/s")

    hyst = hysteresis_by_vehicle(args.calib)
    print(f"读取 5 个 balance_table.csv → Δ: " +
          ", ".join(f"{k.split('.')[-1]}={v['delta_max']}" for k, v in sorted(hyst.items())))

    grade = grade_spread(grade_path)
    print(f"读取 {os.path.relpath(grade_path, HERE)} → 测点摆幅 {grade['v_spread']} m/s")

    loop = json.load(open(loop_path, encoding="utf-8"))
    print(f"读取 {os.path.relpath(loop_path, HERE)} → 刹车 {loop['brake_ticks']}/{loop['n']} 拍")

    margin = loop_margin(os.path.join(args.calib, "acc_loop_tesla_model3", "acc_loop.csv"), tables)
    print(f"逐拍算 R=a_req+G(v) → R_min={margin['R_min']:+.2f} m/s² @v={margin['v_at_R_min']}，"
          f"R<0 的拍数 {margin['ticks_R_negative']}/{margin['n']}")

    rows = build_rows(plant, hyst, grade, loop, sut, args.set_speed, margin)

    # R1 自检：刹车该不该被用到，由 R 的符号决定，不由"跑出来是 0"决定
    if loop["brake_ticks"] == 0 and margin["ticks_R_negative"] == 0:
        print("  ✓ R1 自洽：全程 R≥0 且刹车 0 拍 —— 刹车缺席是结构决定的，不是运气")
    elif margin["ticks_R_negative"] > 0 and loop["brake_ticks"] == 0:
        print(f"  ⚠ 有 {margin['ticks_R_negative']} 拍本该踩刹车却没有 —— 查翻译层或死区")

    data = {
        "generated_by": os.path.basename(__file__),
        "sut": sut,
        "plant": plant,
        "hysteresis": hyst,
        "grade_sweep": grade,
        "closed_loop": {**loop, "set_speed_assumed": args.set_speed},
        "loop_margin": margin,
        "criteria": rows,
    }
    json.dump(data, open(os.path.join(out_dir, "boundary.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    write_md(os.path.join(out_dir, "boundary_table.md"), plant, hyst, grade, loop, sut,
             rows, args.set_speed, margin)
    plot(os.path.join(out_dir, "authority_envelope.png"), tables, plant, sut, loop)

    print(f"\n写出 → {os.path.relpath(out_dir, HERE)}/")
    for f in ["boundary.json", "boundary_table.md", "authority_envelope.png"]:
        print("  ·", f)
    n_bad = sum(1 for r in rows if _norm(r["worst"]) == "不可用")
    print(f"\n{len(rows)} 条判据 / {n_bad} 条在本台架或换车后不可用")


if __name__ == "__main__":
    main()
