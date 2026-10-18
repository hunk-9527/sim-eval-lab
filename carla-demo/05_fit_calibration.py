"""05_fit_calibration.py -- 把 03 扫出来的曲线变成可用的 CARLA 车辆纵向模型。

最终模型形式（三张标定表，加性）
------------------------------
    a(v, theta, beta) = F(theta) - B(beta) - G(v)

    G(v)  阻力        <- coast 曲线直接读
    F(theta) 油门推力 <- 由稳态速度点反解（a=0 时 F(θ)=G(v_ss)，最干净）
    B(beta)  制动力   <- 刹车曲线读

为什么用查表而不是解析式
----------------------
**因为 CARLA 的阻力不服从教科书的二次律。** 实测 coast 曲线：

    v    :  2     5     10    15    20    25    30    32.5
    G(v) :  0.92  2.80  4.60  6.60  8.70  10.3  11.4  11.6   (m/s^2)

低速陡（dG/dv≈0.62）、高速平（dG/dv≈0.07）—— 与风阻 c·v² 的形状**正好相反**，
反而像随速度饱和的滚阻+限幅。硬套 c0+c1v+c2v² 的后果实测过：初版拟合出
c2 = -0.0057（阻力曲线 34 m/s 见顶），换成两段式后 c0 又变成 -1.56
（车静止会自己加速），都是**拿错误形状去迁就数据**产生的伪参数。
查表既保住了数据本身的形状，也保住了每个数字的物理含义。

同理 F(theta) 不强行取线性：初版用 k·max(0,θ-θ0) 时，θ=0.50 的残差明显偏离
线性趋势（ΔF/Δθ 只有 2.9，相邻档位是 11~13）。

可辨识性
--------
两个自由度必须分开定，不能一起丢进最小二乘：
  - 12 m/s 以上的阻力如果没有独立测量（coast 只跑到 12），
    就只能靠"假设油门线性"反推，油门的非线性会被全部塞进 v² 项 -> 伪 c2。
  - 所以 coast 必须覆盖到 40 m/s（03 里 COAST_V0=40），且残差不含过渡拍。
参见 03 的 run_coast 与这里的 COAST_SKIP。

数据过滤
--------
v < V_MIN 丢掉：v<2.5 是 CARLA 离合器/一档啮合瞬态，阶跃不是连续动力学
    （theta=0.30 前 13 拍 v 恒 0.001，第 13 拍直接跳 1.25 m/s，a=12.5）。
coast 前 COAST_SKIP 拍丢掉：松油门后约 0.4 s 的发动机制动建立过渡。

产出
----
  calib/longitudinal_model.json   三张表 + 拟合质量 + 验证结果
  calib/fit_quality.png           实测 vs 预测 + 反查表 + 稳态速度验证
"""

import csv
import glob
import json
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 不出图例乱码（Windows 自带中文字体，DejaVu Sans 没有 CJK 字形）
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB = os.path.join(HERE, "calib")
RUNS = os.path.join(CALIB, "runs")
OUT_JSON = os.path.join(CALIB, "longitudinal_model.json")
OUT_PNG = os.path.join(CALIB, "fit_quality.png")

# ★ 标定表是**车型专属**的（θ=0.50 稳态速度跨车型差 9 倍）。
#   以前车型只出现在 source 那串自由文本里，加载方要校验就得去正则抠字符串 ——
#   所以这里显式写一个字段。加载时用它比对，比解析注释可靠。
VEHICLE = "vehicle.tesla.model3"

V_MIN = 2.5        # 低于此速度属于起步瞬态，不参与拟合
SKIP_FIRST = 2     # 每条曲线丢掉前几 tick（踩/松踏板的那一拍）
COAST_SKIP = 6     # coast 额外丢掉松油门后的过渡拍
V_MAX_TAB = 41.0   # 表格速度上限
V_DV = 0.5         # 表格速度步长
TH_MAX = 1.0
B_MAX = 1.0

ACC_A_LO, ACC_A_HI = -4.0, 2.0   # acc_sim 的 ACC 只会在这个带里要加速度


# ------------------------------ 读数据 ------------------------------
def load(fp):
    return list(csv.DictReader(open(fp, encoding="utf-8")))


def collect():
    """-> V,T,B,A（全体） + Vc,Ac（coast，已丢过渡拍） + vss {theta: v_steady}"""
    V, T, B, A = [], [], [], []
    vc, ac = [], []
    for fp in sorted(glob.glob(os.path.join(RUNS, "*.csv"))):
        rows = load(fp)
        if not rows:
            continue
        phase = rows[0]["phase"]
        for r in rows:
            i = int(r["step"])
            # 用**原始 a** 而不是 a_smooth：a_smooth 是滑动尾窗均值，在每段曲线
            # 开头会严重滞后（coast step1: raw=-3.28 而 smooth=-0.24，因为窗口里
            # 还混着加速段的 +2.8）。拿它算残差会把高速段头几拍全污染成假负值，
            # 实测表现为"踩刹车比滑行减速还慢"这种不可能的结果。
            v, a = float(r["v"]), float(r["a"])
            if i < SKIP_FIRST or v < V_MIN:
                continue
            if phase == "coast" and i < COAST_SKIP:   # 松油门后的发动机制动建立过渡
                continue
            V.append(v)
            T.append(float(r["throttle"]))
            # .get 默认 0：早于 brake 列写入的 hold/coast 文件也要能读
            B.append(float(r.get("brake", 0.0)))
            A.append(a)
            if phase == "coast":
                vc.append(v)
                ac.append(a)

    # 稳态速度点：a=0 处 F(θ) 恰好等于 G(v_ss)，是最干净的 F 来源。
    # 只有 v_ss > V_MIN 的档位能当锚点；其余（0.12/0.15，稳态速度太低）
    # 留作样本外验证 —— 拿锚点自己验证等于自己考自己。
    vss, vss_all = {}, {}
    sump = os.path.join(CALIB, "summary.csv")
    if os.path.exists(sump):
        for r in csv.DictReader(open(sump, encoding="utf-8")):
            if r["note"] != "稳态":
                continue
            vss_all[float(r["throttle"])] = float(r["v"])
            if float(r["v"]) > V_MIN:
                vss[float(r["throttle"])] = float(r["v"])

    print(f"[data] 样本 {len(A)} 个（hold+brake+coast）| coast 有效点 {len(vc)} | "
          f"稳态锚点 {len(vss)} 个 + 样本外 {len(vss_all) - len(vss)} 个")
    return (np.array(V), np.array(T), np.array(B), np.array(A),
            np.array(vc), np.array(ac), vss, vss_all)


# --------------------------- 三张标定表 ---------------------------
def build_G(vc, ac):
    """coast -> 阻力表 G(v)，强制单调不减。"""
    o = np.argsort(vc)
    v, g = vc[o], -ac[o]
    grid = np.arange(0.0, V_MAX_TAB + 1e-9, V_DV)
    tab = np.interp(grid, v, g)
    tab = np.maximum.accumulate(tab)       # 单调不减（阻力不该随速度下降）
    tab[0] = tab[1] if len(tab) > 1 else tab[0]   # v=0 处不搞成 0
    return grid, tab


def build_F(V, T, B, A, vss, G):
    """油门表 F(theta)。

    优先用稳态锚点：a=0 时 F(θ) = G(v_ss)，不受过渡拍和换挡坑影响。
    没有稳态锚点的档位（0.8~1.0，稳态速度超过 v_max）退回曲线中位数。
    """
    th_grid = np.array([0.0, 0.10] + [x / 100 for x in range(12, 101, 2)])
    F = np.zeros_like(th_grid)
    for i, th in enumerate(th_grid):
        if th in vss:                                   # 稳态锚点
            F[i] = np.interp(vss[th], G[0], G[1])
        else:                                           # 曲线中位数
            m = (np.abs(T - th) < 1e-9) & (B == 0)
            F[i] = np.median(A[m] + np.interp(V[m], G[0], G[1])) if m.sum() else np.nan
    # 用锚点做一次单调回归，再对缺口线性补齐
    known = ~np.isnan(F)
    if known.sum() >= 2:
        F[~known] = np.interp(th_grid[~known], th_grid[known], F[known])
    F = np.maximum.accumulate(F)
    F[th_grid <= 0.10] = 0.0                            # 实测死区：θ=0.10 纹丝不动
    return th_grid, F


def build_B(V, T, B, A, G):
    """刹车表 B(beta)：B = -(a + G(v))。"""
    be_grid = np.array([0.0] + [x / 100 for x in range(10, 101, 10)])
    Bt = np.zeros_like(be_grid)
    for i, be in enumerate(be_grid):
        if be == 0.0:
            continue
        m = np.abs(B - be) < 1e-9
        Bt[i] = np.median(-(A[m] + np.interp(V[m], G[0], G[1]))) if m.sum() else 0.0
    Bt = np.maximum.accumulate(Bt)
    Bt[0] = 0.0
    return be_grid, Bt


# ------------------------------ 模型 ------------------------------
def predict(tb, V, T, B):
    Gv = np.interp(V, tb["G_v"], tb["G_G"])
    Fv = np.interp(T, tb["F_t"], tb["F_F"])
    Bv = np.interp(B, tb["B_b"], tb["B_B"])
    return Fv - Bv - Gv


def inverse(tb, v, a_des):
    """(速度, 期望加速度) -> (油门, 刹车)。这就是替换 acc_sim._map_pedals 的东西。

    分三段，不是一条线性公式：
      a_des >= 0          只踩油门：F = a_des + G(v)
      -G(v) <= a_des < 0  松油门滑行就够（阻力已经超过需求），踏板全松
      a_des < -G(v)       需要主动刹车：B = -a_des - G(v)
    """
    Gv = float(np.interp(v, tb["G_v"], tb["G_G"]))
    need = a_des + Gv
    if need >= 0:
        th = float(np.interp(need, tb["F_F"], tb["F_t"]))
        return th, 0.0
    if need >= -Gv:                       # 注意 need<0 且 |need|<=G(v) 时滑行即可
        return 0.0, 0.0
    be = float(np.interp(-need, tb["B_B"], tb["B_b"]))
    return 0.0, be


def steady_of(tb, theta):
    """反查：给定油门，稳态速度是多少（a=0 即 G(v)=F(θ)）。这是 ACC 最关心的指标。"""
    F = float(np.interp(theta, tb["F_t"], tb["F_F"]))
    G = tb["G_G"]
    if F <= G[0]:
        return 0.0
    if F >= G[-1]:
        return float("inf")             # 表格速度上限内到不了稳态
    return float(np.interp(F, G, tb["G_v"]))


# ------------------------------ 主流程 ------------------------------
def main():
    V, T, B, A, vc, ac, vss, vss_all = collect()
    gv, gG = build_G(vc, ac)
    ft, fF = build_F(V, T, B, A, vss, (gv, gG))
    bb, bB = build_B(V, T, B, A, (gv, gG))
    tb = {"G_v": gv, "G_G": gG, "F_t": ft, "F_F": fF, "B_b": bb, "B_B": bB}

    pred = predict(tb, V, T, B)
    err = pred - A
    rmse = float(np.sqrt(np.mean(err ** 2)))
    r2 = 1 - float(np.sum(err ** 2)) / float(np.sum((A - A.mean()) ** 2))

    print("\n=== 标定表 ===")
    print("  v (m/s) :", "  ".join(f"{x:5.1f}" for x in gv[::8]))
    print("  G(v)    :", "  ".join(f"{x:5.2f}" for x in gG[::8]), "  (阻力, m/s²)")
    print("\n  theta   :", "  ".join(f"{x:5.2f}" for x in ft[::4]))
    print("  F(theta):", "  ".join(f"{x:5.2f}" for x in fF[::4]), "  (油门推力)")
    print("\n  beta    :", "  ".join(f"{x:5.2f}" for x in bb))
    print("  B(beta) :", "  ".join(f"{x:5.2f}" for x in bB), "  (制动力)")

    print(f"\n=== 拟合质量（n={len(A)}）===")
    print(f"  全体 RMSE = {rmse:.3f} m/s²    R² = {r2:.4f}")
    m = (A >= ACC_A_LO) & (A <= ACC_A_HI)
    e = err[m]
    band_rmse = float(np.sqrt(np.mean(e ** 2)))
    band_max = float(np.max(np.abs(e)))
    print(f"  ★ ACC 工作带 a∈[{ACC_A_LO}, {ACC_A_HI}]：n={m.sum()}, "
          f"RMSE={band_rmse:.3f} m/s², 偏置={np.mean(e):+.3f}, "
          f"最大={band_max:.3f}")
    # 误差几乎全部来自换挡坑那一段：大油门冲过 ~5 m/s 时 CARLA 升档，
    # a 瞬间掉到 0.5 而模型仍给 8。这不是拟合问题而是换挡快变，模型不该拟合它。
    # 单独报一下，免得整体指标被这一段拖得看起来不能用。
    shift = (V >= 4.5) & (V <= 6.5) & (T >= 0.4) & m
    band_rmse_ex = band_max_ex = None
    if shift.sum() > 5:
        e2 = err[m & ~shift]
        band_rmse_ex = float(np.sqrt(np.mean(e2 ** 2)))
        band_max_ex = float(np.max(np.abs(e2)))
        print(f"    去掉换挡坑(v∈[4.5,6.5] 且 θ≥0.4, n={shift.sum()}): "
              f"RMSE={band_rmse_ex:.3f} m/s², 最大={band_max_ex:.3f}")

    print("\n=== 关键验证：给定油门，稳态速度算得对不对 ===")
    print(f"{'theta':>6} {'实测 v_ss':>10} {'模型 v_ss':>10} {'误差':>8}   来源")
    verr, verr_oos = [], []
    for th in sorted(vss_all):
        vm = steady_of(tb, th)
        d = vm - vss_all[th]
        anchor = th in vss
        (verr if anchor else verr_oos).append(abs(d))
        src = "锚点(用于定F)" if anchor else "★样本外(留出)"
        print(f"{th:>6.2f} {vss_all[th]:>10.2f} {vm:>10.2f} {d:>+8.2f}   {src}")
    if verr:
        print(f"  -> 锚点平均绝对误差 {np.mean(verr):.3f} m/s（循环，只能验证反查表机制）")
    if verr_oos:
        print(f"  -> ★样本外平均绝对误差 {np.mean(verr_oos):.3f} m/s"
              f"（这两个档位没参与定 F 表，是真正的留出检验）")

    print("\n=== 设计结论：ACC 要的减速，车给得起吗 ===")
    print(f"{'v(m/s)':>7} {'G(v) 滑行':>10} {'a=-4 需要 beta':>15} {'够不够':>10}")
    for v in [2, 5, 10, 20, 30, 40]:
        Gv = float(np.interp(v, gv, gG))
        need = 4.0 - Gv
        be = float(np.interp(need, bB, bb)) if need > 0 else 0.0
        cap = float(np.interp(1.0, bb, bB))
        ok = "OK" if need <= cap + 1e-9 else "★踩满也不够"
        print(f"{v:>7} {Gv:>10.2f} {be:>15.2f} {ok:>10}")
    print(f"  刹车最大能力 B(1.0) = {float(np.interp(1.0, bb, bB)):.2f} m/s²，"
          f"叠上阻力后在 40 m/s 可达 "
          f"{float(np.interp(1.0, bb, bB)) + float(np.interp(40, gv, gG)):.2f} m/s²")

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({
            "vehicle": VEHICLE,
            "source": "03_calibrate_longitudinal.py (treadmill, CARLA 0.9.15, "
                      f"Town04, {VEHICLE}), dt=0.1s",
            "model": "a(v,theta,beta) = F(theta) - B(beta) - G(v)   [lookup tables]",
            "tables": {"G_v": gv.tolist(), "G_G": gG.tolist(),
                       "F_theta": ft.tolist(), "F_F": fF.tolist(),
                       "B_beta": bb.tolist(), "B_B": bB.tolist()},
            "quality": {"rmse_all": rmse, "r2": r2, "n_samples": int(len(A)),
                        "acc_band_rmse": band_rmse if m.sum() else None,
                        "acc_band_max_err": band_max if m.sum() else None,
                        "acc_band_rmse_ex_shift": band_rmse_ex,
                        "acc_band_max_err_ex_shift": band_max_ex,
                        "steady_speed_mae_in_sample": float(np.mean(verr)) if verr else None,
                        "steady_speed_mae_out_of_sample": float(np.mean(verr_oos)) if verr_oos else None},
            "notes": [
                "阻力非二次律：低速陡、高速平（见 fit_quality.png 中图），故用查表而非 c0+c1v+c2v²",
                "theta<=0.10 死区：实测 8 秒纹丝不动（v 精确 0），表里 F(θ<=0.10)=0",
                "v<2.5 起步瞬态已剔除：该段是离合器/一档啮合阶跃（θ=0.30 前 13 拍 v 恒 0.001，"
                "第 13 拍直接跳 1.25 m/s），模型不适用于从静止起步",
                "coast 前 0.4s 为发动机制动建立过渡，已剔除",
                "★ 大油门冲过 v≈5 时 CARLA 升档，加速度瞬间从 ~8 掉到 ~0.5。"
                "该段模型必然偏高，ACC 带内最大误差 7.8 m/s² 全部来自这里；"
                "把它剔除后 ACC 带 RMSE 见 quality.acc_band_rmse_ex_shift",
                "★ 刹车力残差在 v>=35 偏低 0.3~1.4 m/s²，疑为 G(v) 高速段略偏高；"
                "ACC 常规工作速度（set_speed=30 m/s）基本用不到该区间",
                "★ 样本外验证（θ=0.12/0.15，未参与定 F 表）：MAE 0.891 m/s。"
                "其中 θ=0.12 模型预测不动、实测蠕行到 1.38 m/s —— 因为该档位全部数据都落在"
                "被剔除的起步瞬态区。低油门 + 低速（v<3）这一角是模型的已知盲区。",
            ],
        }, f, ensure_ascii=False, indent=2)
    print(f"\n[out] {OUT_JSON}")

    # ------------------------------ 出图 ------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    ax = axes[0]
    for mask, c, lab in [((T > 0), "tab:blue", "throttle"),
                         ((T == 0) & (B == 0), "k", "coast"),
                         ((B > 0), "crimson", "brake")]:
        ax.scatter(A[mask], pred[mask], s=5, alpha=0.35, color=c, label=lab)
    lo = min(A.min(), pred.min()) - 0.5
    hi = max(A.max(), pred.max()) + 0.5
    ax.plot([lo, hi], [lo, hi], "g-", lw=1.0, label="1:1")
    ax.set_xlabel("measured a (m/s²)")
    ax.set_ylabel("model a (m/s²)")
    ax.set_title(f"fit quality  RMSE={rmse:.3f}  R²={r2:.4f}")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    o = np.argsort(vc)
    ax.plot(vc[o], -ac[o], "k.", ms=4, label="coast 实测 G(v)")
    ax.plot(gv, gG, "r-", lw=1.5, label="标定表 G(v)")
    # 示意用的二次律，缩放到同一量级只为对比形状（不缩放会把 y 轴撑到 250）
    ax.plot(gv, 0.008 * gv ** 2, "b--", lw=1.0, label="若按教科书 c·v²（形状反了）")
    ax.set_ylim(0, 14)
    ax.set_xlabel("v (m/s)")
    ax.set_ylabel("G(v)  (m/s²)")
    ax.set_title("CARLA 阻力曲线：低速陡、高速平\n→ 不是二次律，故用查表")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[2]
    vg = np.linspace(0, 40, 200)
    ax.plot(vg, [inverse(tb, v, 2.0)[0] for v in vg], label="throttle for a=+2.0")
    ax.plot(vg, [inverse(tb, v, 1.0)[0] for v in vg], label="throttle for a=+1.0")
    ax.plot(vg, [inverse(tb, v, -2.0)[1] for v in vg], color="crimson",
            label="brake for a=-2.0")
    ax.plot(vg, [inverse(tb, v, -4.0)[1] for v in vg], color="darkred", ls="--",
            label="brake for a=-4.0")
    ax.axhline(0.10, color="gray", ls=":", lw=1)
    ax.text(0.5, 0.115, "油门死区 θ=0.10", fontsize=8, color="gray")
    ax.set_xlabel("v (m/s)")
    ax.set_ylabel("pedal (0..1)")
    ax.set_ylim(0, 1.05)
    ax.set_title("反查表：要得到某加速度，得踩多少踏板")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=130)
    print(f"[out] {OUT_PNG}")


if __name__ == "__main__":
    main()
