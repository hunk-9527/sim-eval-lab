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

:: ⑤ ACC 闭环（图 1，刹车 0/500 拍）
python 09_acc_carla.py --vehicle vehicle.tesla.model3 --set-speed 25 --duration 60

:: ⑥ 自由行驶复现（图 4 右）
python 10_free_drive_repro.py
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
| 各点实测坡度 | `+0.29% / +5.18% / 0.00% / −6.00%` |

**平衡车速 = f(车, 测点坡度)**。同车同油门只换落点，实测 **5.84 → 19.02 m/s**。
任何稳态车速数字**不带坡度和位置就没有意义**。

---

## 5. 数字 → 文件对照

| 结论 | 脚本 | 数据 |
|---|---|---|
| 标定拟合 R² / RMSE | `05_fit_calibration.py` | `calib/longitudinal_model.json` |
| 阻力曲线 `G(10)`、`G(30)` | `03_calibrate_longitudinal.py` | `calib/longitudinal_model.json` → `tables.G_v` |
| 五车油门阶梯 | `06_vehicle_ladder.py` | `calib/vehicle_ladder.csv`、`calib/veh_*/summary.csv` |
| 五车迟滞差 Δ | `07_low_speed_balance.py` | `calib/veh_*/balance_table.csv`（`up` vs `down` 两行） |
| 测点坡度扫掠 5.84→19.02 | ⚠ `11_grade_sweep.py`（**待补**） | ⚠ **原始数据未落盘**，见下方"已知缺口" |
| 刹车 0/500 拍 | `09_acc_carla.py` | `calib/acc_loop_tesla_model3/acc_loop.csv`、`acc_loop_verdict.json` |
| 自由行驶 vs 跑步机（图 4 右） | `10_free_drive_repro.py` | `calib/free_drive_repro/summary.json` |
| 车辆物理参数 | `08_probe_physics.py` | `calib/carla_vehicle_physics.json` |

`calib/` 只保留 json / csv / log；**png 由脚本重新生成，不入库**。

### 已知缺口（诚实标注）

**测点坡度扫掠（反例 3）的逐次运行数据没有留存。** 当时是一次性实验，
只把汇总值写进了 `calibration-playbook.md` 的 S0′ 表：

| 位置 | 局部坡度 | 升速落点 | 降速落点 | Δ |
|---|---|---|---|---|
| 0 m | +0.29% | 3.30 | 12.44 | 9.14 |
| 200 m | **+5.18%** | **0.10** | **5.84** | 5.75 |
| 400 m | 0.00% | 3.32 | 12.47 | 9.15 |
| 600 m | **−6.00%** | **4.87** | **19.02** | 14.15 |

→ **待补** `11_grade_sweep.py`：沿 Town04 直道四个落点各跑一次 S0 对照，
把逐次曲线落到 `calib/grade_sweep/`。补齐前，这一条**只能算"报告过的观测"，
不能算"可复现"**。其余两条（反例 1、2）数据齐全。

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

## 7. 文件

| 文件 | 作用 |
|---|---|
| `carla_env.py` | 服务端进程管理 + 跑步机原语（`speed_of` / `settle` / `tick_pinned`） |
| `carla_map.py` | 改 `DefaultEngine.ini` 换图 |
| `carla_pedal_map.py` | 标定表逆映射：`a → (throttle, brake)`，加载时校验车型与迟滞差 |
| `lane_keep.py` | 自由行驶用的简易车道保持（仅 `10_` 需要） |
| `01_`–`10_` | 见第 3、5 节 |
