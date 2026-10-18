"""carla_pedal_map.py -- 把标定表变成「要多少加速度 → 踩多少踏板」的映射，并**在加载时校验**。

要解决的问题
------------
acc_sim 的 `_map_pedals` 假设 `throttle = accel / accel_max`（线性）。在 CARLA 里这个假设**不成立**：
里程碑 1 实测恒定油门 0.5，车速 1 m/s 时 a≈1.85 m/s²，13 m/s 时只剩 0.085 m/s²，**差 20 倍**。
所以 ACC 直接接上去会"想加速却加不动"。本模块用 03/05 标出来的模型反解踏板：

    a = F(theta) - B(beta) - G(v)
    ->  令 R = a + G(v)（净推力需求）
        R >= 0 : beta=0, theta = F^-1(R)
        R <  0 : theta=0, beta  = B^-1(-R)

★ 术语：本文的「**被控对象**」= plant = **被 ACC 操纵的那台车**（CARLA 的车型），
  与「**被测对象 SUT**」（= ACC 算法）是两个东西。混起来会"以为在测 A，其实在测 B"。

★ 这个式子有个反直觉的推论，**不是笔误**：低速/中速时 G(v) 很大（10 m/s 就 4.6 m/s²），
  所以 ACC 请求 "a = -0.5" 时 R = -0.5 + 4.6 = +4.1 ≥ 0 → **要收油，不要踩刹车**。
  "请求负加速度" ≠ "踩刹车" —— 在这个被控对象上，光抬脚就已经减速 4.6 m/s² 了。

为什么要在加载时校验（本模块存在的第二个理由）
--------------------------------------------
标定表是**车型专属**的，而且（见 playbook 第七节）**不是每台车都有可用的表**：
多挡车的踏板→速度不是单值映射（实测迟滞差 15~27 m/s），03/06 给多挡车出的那张表
只是"多条分支里的一条"。拿它来接 ACC，不会报错 —— 只会让车在一个看起来正常、
实际没有意义的映射上跑，**结论归属不清**（分不清是 ACC 坏了还是被控对象换分支了）。

所以 `CarlaPedalMap` 在 __init__ 里关两道门：
  ① `vehicle` 必须与 `expected_vehicle` 一致（防止拿错表）
  ② 用 `balance_table.csv` **自己重算**迟滞差 Δ，超过 `hysteresis_tol` 就拒绝加载
     —— ★ 重算而不是读一个自称的字段：表说自己能标不算数，得拿数据验。

用法
----
    pm = CarlaPedalMap("calib/longitudinal_model.json",
                       expected_vehicle="vehicle.tesla.model3",
                       hysteresis_tol=0.5)
    thr, brk = pm.pedals(accel=-1.0, v=12.0)
    pm.report()        # 一眼看清这张表的可信区间与已知盲区
"""

import csv
import json
import os

DEFAULT_TOL = 0.5        # m/s，允许多大迟滞差。单速车实测 ≈0.22 通过；多挡车 15~27 必拒
V_MIN_TRUSTED = 2.5      # 05 的 V_MIN：以下是从静止起步的离合器瞬态，模型不适用
DEFAULT_DEAD_THETA = 0.10  # θ≤0.10 死区（实测 8 s 纹丝不动），表里 F=0


def _load_json(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def _read_balance(balance_csv):
    """读 balance_table.csv -> {throttle: {"up": v_mean, "down": v_mean}}。"""
    by_th, out = {}, {}
    with open(balance_csv, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            th = float(row["throttle"])
            by_th.setdefault(th, {})[row["mode"]] = {
                "v_mean": float(row["v_mean"]),
                "v_std": float(row["v_std"]),
                "a_abs_mean": float(row["a_abs_mean"]),
                "class": row["class"],
            }
    for th, modes in by_th.items():
        if "up" in modes and "down" in modes:
            out[th] = modes
    return out


def hysteresis_max(paired):
    """★ 自己算迟滞差，不接受任何人自称的值。

    只有同时有 up / down 两条路径的油门才有意义 —— 迟滞的定义就是"同一输入两条路径的差"，
    缺一条就无从比较（这不是 0，是**没测**）。

    ★★ 这里有个必须分开的区分（实测踩到，2026-10-15）：
    把所有配对油门一起取 max，会对 tesla 给出 Δ=0.71 —— 但那**不是**"表不单值"，
    而是 θ=0.10 处「**起步阈值 ≠ 停车阈值**」（升速路径死区纹丝不动，降速路径稳定蠕行 0.715）。
    这是**阈值型现象**，不是"同一平衡点分叉"。

    所以先剔掉任一路径为「死区」的配对再取 max —— 理由事先写明、不是为了让谁通过：
    **死区意味着车根本没动，"速度"在那里不是一个可测的平衡点**，拿它的 v_mean 去算迟滞
    等于把"没测到"当成"测到了另一条分支"。剔掉的那几个**照实列出**（dead_zone），不隐藏。

    五台车实测（排除死区配对后，单速 vs 多挡零重叠）：
        microlino 0.084 | tesla 0.218  ||  audi a2 15.03 | carlacola 18.78 | patrol 27.37
    """
    rows, dead = [], []
    for th in sorted(paired):
        m = paired[th]
        dv = abs(m["down"]["v_mean"] - m["up"]["v_mean"])
        r = {"throttle": th, "v_up": m["up"]["v_mean"], "v_down": m["down"]["v_mean"],
             "dv": dv, "class_up": m["up"]["class"], "class_down": m["down"]["class"]}
        (dead if ("死区" in m["up"]["class"] or "死区" in m["down"]["class"])
         else rows).append(r)
    dv_max = max((r["dv"] for r in rows), default=None)
    return dv_max, rows, dead


def _inv_interp(xs, ys, y):
    """给定 y 求 x（ys 单调不减）。返回 (x, saturated)。"""
    if y <= ys[0]:
        return xs[0], True
    if y >= ys[-1]:
        return xs[-1], True
    lo, hi = 0, len(ys) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ys[mid] <= y:
            lo = mid
        else:
            hi = mid
    span = ys[hi] - ys[lo]
    x = xs[lo] if span == 0 else xs[lo] + (xs[hi] - xs[lo]) * (y - ys[lo]) / span
    return x, False


class Infeasible(Exception):
    """请求的加速度这张表给不出来（例如要求比满油门还强的推力）。"""


class CarlaPedalMap:
    def __init__(self, model_json, expected_vehicle=None, hysteresis_tol=DEFAULT_TOL,
                 balance_csv=None, strict=True):
        self.path = model_json
        m = _load_json(model_json)
        self.raw = m
        self.dt = 0.1                     # 与 AccConfig.dt 对齐（json 的 source 里写着）

        # ---- 防线 ① 车型 ----
        self.vehicle = m.get("vehicle")
        if not self.vehicle:
            # 老格式（只有 source 自由文本）不猜、不放过 —— 让它显式失败，
            # 否则"校验"就成了摆设。重跑 05 即可补上该字段。
            raise ValueError(
                f"{model_json} 没有 'vehicle' 字段：无法校验车型。"
                f"重跑 05_fit_calibration.py 生成带 vehicle 字段的模型。"
                f"(source={m.get('source')!r})")
        if expected_vehicle and self.vehicle != expected_vehicle:
            raise ValueError(f"标定表属于 {self.vehicle}，当前车是 {expected_vehicle}")

        # ---- 表 ----
        t = m["tables"]
        self.g_v, self.g_G = list(t["G_v"]), list(t["G_G"])
        self.f_t, self.f_F = list(t["F_theta"]), list(t["F_F"])
        self.b_b, self.b_B = list(t["B_beta"]), list(t["B_B"])
        self.gamma_max = self.f_F[-1]          # 满油门推力
        self.beta_max_cap = self.b_B[-1]       # 满刹车制动力

        # ---- 防线 ② 迟滞差（有没有资格叫"表"）----
        self.dv_max, self.dv_rows, self.dead_rows = None, [], []
        if balance_csv and os.path.exists(balance_csv):
            paired = _read_balance(balance_csv)
            self.dv_max, self.dv_rows, self.dead_rows = hysteresis_max(paired)
        self.hysteresis_tol = hysteresis_tol
        self.balance_csv = balance_csv
        if self.dv_max is not None and self.dv_max > hysteresis_tol:
            msg = (f"{self.vehicle} 迟滞差 Δ={self.dv_max:.2f} m/s > 容差 {hysteresis_tol} m/s："
                   f"踏板→速度**不是单值映射**，这张表只是多条分支里的一条，不能当映射用。"
                   f"（Δ 最大的油门见 dv_rows）")
            if strict:
                raise ValueError(msg)
            self.warn = msg
        else:
            self.warn = None

        self.saturations = 0     # 统计：有多少拍请求被表截断（要交付的边界，不是报错）

    # ------------------------- 查询 -------------------------
    def G(self, v):
        return float(_interp(self.g_v, self.g_G, v))

    def pedals(self, accel, v):
        """要 accel (m/s²) 时，踏板怎么踩。返回 (throttle, brake)。"""
        R = accel + self.G(v)              # 净推力需求
        if R >= 0.0:
            th, sat = _inv_interp(self.f_t, self.f_F, R)
            if sat:
                self.saturations += 1
            return th, 0.0
        be, sat = _inv_interp(self.b_b, self.b_B, -R)
        if sat:
            self.saturations += 1
        return 0.0, be

    def achievable(self, accel, v):
        """这张表实际能给到的加速度（把饱和考虑进去）。"""
        th, br = self.pedals(accel, v)
        return self.predict(th, br, v)

    def predict(self, throttle, brake, v):
        """正问题：给踏板和速度，算加速度（=05 的模型式）。"""
        F = float(_interp(self.f_t, self.f_F, throttle))
        B = float(_interp(self.b_b, self.b_B, brake))
        return F - B - self.G(v)

    def trusted(self, v):
        """这个速度上模型可不可信（05 剔了 v<2.5 的起步瞬态）。"""
        return v >= V_MIN_TRUSTED

    # ------------------------- 交付边界 -------------------------
    def report(self):
        L = []
        L.append(f"车型        : {self.vehicle}")
        L.append(f"满油门推力  : {self.gamma_max:.2f} m/s²   满刹车制动力: {self.beta_max_cap:.2f} m/s²")
        L.append(f"G(v) 滑行阻力: v=2 -> {self.G(2):.2f}  10 -> {self.G(10):.2f}  "
                 f"30 -> {self.G(30):.2f} m/s²")
        if self.dv_max is None:
            L.append("迟滞差 Δ    : **未测**（没给 balance_csv —— 不知道这张表够不够格，"
                     "不等于它是单值的）")
        else:
            tag = "OK" if self.dv_max <= self.hysteresis_tol else "★拒绝"
            L.append(f"迟滞差 Δmax : {self.dv_max:.3f} m/s (容差 {self.hysteresis_tol})  [{tag}]  "
                     f"共 {len(self.dv_rows)} 个双路径油门")
        if self.dead_rows:
            L.append(f"⚠ 已剔除    : {len(self.dead_rows)} 个「任一路径为死区」的油门 "
                     f"（车没动 ⇒ 那里没有可测平衡点）:")
            for r in self.dead_rows:
                L.append(f"              θ={r['throttle']:.2f}  up={r['class_up']}"
                         f"({r['v_up']:.2f})  down={r['class_down']}({r['v_down']:.2f})"
                         f"  Δ={r['dv']:.2f}")
            L.append("              ★ 这是**起步阈值 ≠ 停车阈值**，与「表不单值」是两回事；"
                     "它是真实工况，ACC 用例要单独覆盖，但不该算进 Δ")
        L.append(f"可信速度区间: v >= {V_MIN_TRUSTED} m/s （v<{V_MIN_TRUSTED} 是起步瞬态，模型不适用）")
        L.append(f"死区        : theta <= {DEFAULT_DEAD_THETA} 推力为 0（实测 8 s 不动）")
        L.append("★ 已知推论 : G(v) 远大于 ACC 的寻常减速请求 → 请 a<0 常常是**收油**而不是踩刹车；"
                 "刹车只在 |a| 超过 G(v) 时才用得上")
        return "\n".join(L)


def _interp(xs, ys, x):
    """线性插值（xs 单调递增），端点外取端点值。"""
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
    span = xs[hi] - xs[lo]
    return ys[lo] if span == 0 else ys[lo] + (ys[hi] - ys[lo]) * (x - xs[lo]) / span


def main():
    """自检：加载 tesla 的表 + 拒掉多挡车的表（两条防线都要能看到真的拦住）。"""
    import glob
    here = os.path.dirname(os.path.abspath(__file__))
    calib = os.path.join(here, "calib")
    model = os.path.join(calib, "longitudinal_model.json")

    print("=" * 78)
    print("① 正常路径：tesla 单速车")
    print("=" * 78)
    pm = CarlaPedalMap(model, expected_vehicle="vehicle.tesla.model3",
                       balance_csv=os.path.join(calib, "veh_tesla_model3", "balance_table.csv"))
    print(pm.report())

    print("\n  踏板映射抽查（看 a<0 时踩的是哪个）")
    print(f"  {'v':>5} {'请求 a':>7} {'G(v)':>7} {'净需求R':>8} {'油门':>7} {'刹车':>7} {'实际给到':>9}")
    for v in [3.0, 10.0, 20.0, 30.0]:
        for a in [+1.0, 0.0, -0.5, -2.0, -4.0]:
            th, br = pm.pedals(a, v)
            print(f"  {v:>5.1f} {a:>+7.1f} {pm.G(v):>7.2f} {a + pm.G(v):>8.2f} "
                  f"{th:>7.3f} {br:>7.3f} {pm.predict(th, br, v):>9.2f}")

    print("\n" + "=" * 78)
    print("② 防线①：拿错表（expected_vehicle 不匹配）")
    print("=" * 78)
    try:
        CarlaPedalMap(model, expected_vehicle="vehicle.nissan.patrol",
                      balance_csv=os.path.join(calib, "veh_tesla_model3", "balance_table.csv"))
        print("  ❌ 没拦住 —— 防线失效")
    except ValueError as e:
        print(f"  ✅ 拦住了：{e}")

    print("\n" + "=" * 78)
    print("③ 防线②：多挡车的表必须被拒（它根本没有合法映射）")
    print("=" * 78)
    for csvp in sorted(glob.glob(os.path.join(calib, "veh_*", "balance_table.csv"))):
        slug = os.path.basename(os.path.dirname(csvp))
        # 用 tesla 的模型配各车的 balance 表，只为演一遍防线②的行为
        try:
            CarlaPedalMap(model, balance_csv=csvp, hysteresis_tol=0.5)
            print(f"  {slug:<28} 通过（Δ 在容差内）")
        except ValueError as e:
            d = str(e).split("：")[0]
            print(f"  {slug:<28} ★拒绝 —— {d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
