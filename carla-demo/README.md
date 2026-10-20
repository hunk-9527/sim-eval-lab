# carla-demo — CARLA 0.9.15 纵向实验台与 ACC 闭环

本目录是 `sim-eval-lab` 的实验主体。所有数字来自下面这套脚本，`calib/` 里是它们的原始输出。

---

## 1. 环境

| 项 | 值 |
|---|---|
| 仿真器 | **CARLA 0.9.15**，Windows 原生（非 Docker） |
| Python | 3.11（`carla` 0.9.15 wheel + numpy / pandas / matplotlib） |
| 步长 | `0.1 s`，同步模式，固定步长 |
| 地图 | 自由行驶 / 测点实验：**Town04**；五车迟滞对照：**Town10HD_Opt** |
| SUT | [hunk-9527/acc-sim](https://github.com/hunk-9527/acc-sim)，按路径导入 |

```bat
set ACC_SIM_DIR=D:\path\to\acc-sim
```

跑任何脚本：

```bat
set PYTHONIOENCODING=utf-8
python 09_acc_carla.py
```

> Windows 上如果裸 `python` 落到应用商店别名，会**静默失败、退出码 49**。请用解释器全路径。

---

## 2. 换地图（0.9.15 的一个坑）

**打包版忽略命令行地图参数** —— `CarlaUE4.exe ... Town04` 里的地图名从没生效过。
换图只能改 `DefaultEngine.ini` 后重启进程：

```bat
python carla_map.py set Town04      :: 首次自动备份
python carla_map.py current
python carla_map.py restore
```

另外 **`client.load_world()` 第二次调用必崩服务端**。"就绪"（端口通）也 ≠ "图对"，
所以 `carla_env.ensure_carla(map_name=...)` 会重新校验当前图。
遇到图不对，先怀疑这两条，不要怀疑自己的代码。

---

## 3. 跑法（按实验目的）

```bat
:: ① 踏板 → 加速度标定 + 零油门滑行阻力 G(v)
python 03_calibrate_longitudinal.py --quick          :: 冒烟
python 03_calibrate_longitudinal.py --force

:: ② 拟合 + 质量图（图 2）
python 05_fit_calibration.py

:: ③ 五车油门阶梯（图 4 左）
python 06_vehicle_ladder.py

:: ④ 迟滞对照 S0（图 4 左的 Δ）
python 07_low_speed_balance.py --vehicle vehicle.tesla.model3

:: ④′ 测点坡度扫掠 S0′（反例 3）—— 换四个钉住位置各跑一次
python 11_grade_sweep.py --vehicle vehicle.carlamotors.carlacola --theta 0.40

:: ⑤ ACC 闭环（图 1，刹车 0/500 拍）
python 09_acc_carla.py --vehicle vehicle.tesla.model3 --set-speed 25 --duration 60

:: ⑥ 自由行驶复现（图 4 右）
python 10_free_drive_repro.py

:: ⑦ 可用性边界表（不启动 CARLA，秒级；读上面几步的产物）
python 12_credibility_boundary.py
```

`--skip-run` / `--plot-only` / `--replot` 用于**只汇总不出仿真**，改图时不必重跑。

---

## 4. ★ 测点坐标（可复现性的关键）

第 3 个反例（测点依赖）**必须带上这一行**，否则别人跑出来的平衡车速一定和本文不一样：

| 项 | 值 |
|---|---|
| 地图 | Town04 |
| 直道起点 | `(−364.3, 37.1)` |
| 测点 | 沿直道 **0 / 200 / 400 / 600 m** 四点 |
| 各点实测坡度 | `+0.01% / +5.18% / 0.00% / −6.00%` |

**平衡车速 = f(车, 测点坡度)**。同车同油门只换测点，**升速落点 0.10 → 4.88 m/s**。
任何稳态车速数字**不带坡度和位置就没有意义**。

---

## 5. 数字 → 文件对照

| 结论 | 脚本 | 数据 |
|---|---|---|
| 标定拟合 R² / RMSE | `05_fit_calibration.py` | `calib/longitudinal_model.json` |
| 阻力曲线 `G(10)`、`G(30)` | `03_calibrate_longitudinal.py` | `calib/longitudinal_model.json` → `tables.G_v` |
| 五车油门阶梯 | `06_vehicle_ladder.py` | `calib/vehicle_ladder.csv`、`calib/veh_*/summary.csv` |
| 五车迟滞差 Δ | `07_low_speed_balance.py` | `calib/veh_*/balance_table.csv`（`up` vs `down` 两行） |
| 测点坡度扫掠 | `11_grade_sweep.py` | `calib/grade_sweep_carlacola/`、`grade_sweep_tesla/`、`grade_sweep_prespin/` |
| 刹车 0/500 拍 | `09_acc_carla.py` | `calib/acc_loop_tesla_model3/acc_loop.csv`、`acc_loop_verdict.json` |
| 自由行驶 vs 跑步机（图 4 右） | `10_free_drive_repro.py` | `calib/free_drive_repro/summary.json` |
| 车辆物理参数 | `08_probe_physics.py` | `calib/carla_vehicle_physics.json` |

`calib/` 只保留 json / csv / log；**png 由脚本重新生成，不入库**。

### 反例 3 的复现结果（2026-10-18 补跑）

`11_grade_sweep.py` 在 Town04 直道 0 / 200 / 400 / 600 m 四点各跑一次 up/down。
局部坡度**逐位复现**：`+5.18% / 0.00% / −6.00%`（0 m 点为 `+0.01%`，
与早期记录的 `+0.29%` 略有差异，属落点微调）。

**carlacola，θ=0.40，冲刺初速 25 m/s：**

| 位置 | 局部坡度 | 升速落点 | 降速落点 |
|---|---|---|---|
| 0 m | +0.01% | 3.40 | 17.64（未达平衡） |
| 200 m | +5.18% | **0.098** | 1.09（极限环） |
| 400 m | 0.00% | 3.42 | 17.75（未达平衡） |
| 600 m | −6.00% | **4.88** | 34.09（未达平衡，窗口结束仍在爬，上限 `v_max=40`） |

**两件事要说清楚：**

- ✅ **升速落点复现良好**（早期记录 3.30 / 0.10 / 3.32 / 4.87，最大偏差 0.10 m/s）。
  同一台车、同一油门，只换测点，落点从 0.098 摆到 4.88 m/s ——
  "平衡车速 = f(车, 测点坡度)"成立。
- ⚠ **降速落点不可复现，而且原因已定位**：它依赖**冲刺初速**。
  同一落点（0 m）实测 10.93（16 m/s）/ 17.10（20 m/s）/ 17.64（25 m/s）。
  早期记录 12.44 落在 16 与 20 之间，而**当时的冲刺初速没有记录下来**。

→ 这不是"跑错了"，是更硬的结论：**不记录初始条件的"落点"本身就不是可迁移的数**。
  所以本仓库所有降速数据都把 `pre_spin_v` 写进 `summary.json`。

**tesla（单速车）同一扫掠**（`grade_sweep_tesla/`）：up 与 down 在每个落点都重合
（Δ < 0.004），落点随坡度单调：`6.28 → 11.06 m/s`。
说明**测点效应与迟滞无关** —— 单速车没有迟滞，照样摆 1.76 倍。

---

## 6. 两个测量法，不要混用

| | 跑步机（`tick_pinned`） | 自由行驶（`10_`） |
|---|---|---|
| 做法 | 每 tick 跑完物理把车拉回出发点，**保留速度** | 车真的在路上跑 |
| 优点 | 无横向、无路口、不挑地图 | 接近真实使用 |
| 代价 | **把测点坡度冻在原地** | 会冲出车道、要挑直道 |
| 适用 | 踏板→加速度这种**局部**映射 | 平衡点、多稳态这种**整体**性质 |

⚠ 跑步机**不是**"停稳"原语。拿它当刹车测试的收尾会让车永远落不了地，
刹车踩满仍残留 `|v| ≈ 5.2 m/s`。要落地用 `carla_env.settle()`。

⚠ `set_transform(home)` 会**连朝向一起复位** = 一个隐形车道保持器，
会掩盖偏航发散。纵向可用，**横向 / 操控判据不可用**。

---

## 7. 可用性边界表 —— 判据能不能测，是可以算出来的

`12_credibility_boundary.py` **不启动 CARLA**，只读前面几步留下的标定产物，把三个量放在一起比：

- **SUT 请求域**：`acc-sim` 的 `AccConfig`（−4.0 ~ +2.0 m/s²）
- **被控对象可达域**：标定表 F(θ) / B(β) − G(v)
- **被控对象固有量**：G(v) 滑行阻力、迟滞 Δ、测点依赖

三条拒绝条件，任一命中，这条判据在这个台架上就失效：

| | 条件 | 本次的数值 |
|---|---|---|
| **R1** | 固有量 ≥ SUT 请求量 → 执行器不被激励 | G(v) ≥ 4.0 当 v ≥ **9.54 m/s**；闭环逐拍 `R = a_req + G(v)` → **R_min = +2.81 m/s²，R < 0 的拍数 0 / 500** |
| **R2** | 判据容差 < 被控对象分辨率 | 五车 Δ = 0.08 / 0.22 / 15.03 / 18.78 / 27.37 m/s；跟车微调要 0.1 m/s |
| **R3** | 判据量绑在测点上 | 只换钉住位置，平衡车速摆 **4.78 m/s**（6.28 → 11.06） |

结果：10 条判据 × 3 种工况（本台架 / 换多挡车 / 换测点），
本台架 **可用 4 / 需谨慎 2 / 不可用 4**；换多挡车后可用只剩 2 条 —— **换的不是判据，是被控对象**。
完整表（含每条的依据）：`calib/credibility_boundary/boundary_table.md`，由脚本生成，不手写。

> ★ `R < 0 的拍数 = 0` 是关键一步：它把「刹车 0/500 拍」从**一次观察**变成**结构结论** ——
> 不是恰好没用到刹车，是**用不到**。

两条顺带的结论：

- **碰撞类判据是单向的**。被控对象的减速能力是 SUT 请求的 0.99~3.53 倍，危险场景会被它自动化解
  → **撞上了 = 可信；没撞上 ≠ 安全。**
- **精度类判据在这个闭环里接近自证**。翻译层用的就是同一张标定表，"命令多少得到多少"在定义上成立。
  稳态车距误差 0.09 / 0.16 m 里，一半是 SUT 好、一半是执行器被理想化，**两者在这个台架上分不开**。
  → 这类数字可以当回归基线，不能当"控制精度高"或"像实车"的证据。

```bat
python 12_credibility_boundary.py
```

---

## 8. 文件

| 文件 | 作用 |
|---|---|
| `carla_env.py` | 服务端进程管理 + 跑步机原语（`speed_of` / `settle` / `tick_pinned`） |
| `carla_map.py` | 改 `DefaultEngine.ini` 换图 |
| `carla_pedal_map.py` | 标定表逆映射：`a → (throttle, brake)`，加载时校验车型与迟滞差 |
| `lane_keep.py` | 自由行驶用的简易车道保持（仅 `10_` 需要） |
| `12_credibility_boundary.py` | 可用性边界表生成器（不启动 CARLA），产物在 `calib/credibility_boundary/` |
| `01_`–`11_` | 见第 3、5 节 |
