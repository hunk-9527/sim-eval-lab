"""carla_env.py -- CARLA 服务端进程管理 + 跑步机基础原语（03 / 06 / 07 共用）。

为什么要有单独一份
------------------
CARLA 0.9.15 会崩，而且是**必然崩**和**随机崩**两种：
  - 必然：`client.load_world()` 第二次调用打死服务端 —— 见经验库 P-SIM-01
  - 随机：低画质也偶发 OOM
所以任何无人值守的长跑都必须**自己管进程生死**：开跑前探活，死了重启并等就绪。
这段逻辑 06 / 07 都要用，留一份唯一实现。

跑步机法（treadmill）
--------------------
每个 tick 跑完物理后，把车 transform 拉回出发点（**保留速度**）：
位置不动，只让速度积分。三个原语就是它的全部 —— speed_of / settle / tick_pinned。
03 的标定测量台、04 的判决实验、07 的平衡点扫描都建在它们上面。

为什么值得：开车法（让车真的在路上跑）踩了一串坑，跑步机全躲开 ——
不用 lane_keep（位置不动，无需转向）、不会撞车掉路沿（实测开车法 theta=0.7~0.9
都在高速段冲出车道，a 出现 -8 ~ -12 m/s² 的乱跳）、不用挑地图挑出生点。

★ 不要用 `client.load_world()` 切地图 —— 服务端**启动时是什么图就用什么图**；
  要换图请重启进程（`ensure_carla()` 会替你重启）。

跑法：`PYTHONIOENCODING=utf-8 D:/TOOL/SIL/envs/sim/python.exe <脚本>`
"""

import math
import os
import subprocess
import time

import carla

# ------------------------- 服务端 -------------------------
HOST, PORT = "localhost", 2000
CARLA_DIR = r"D:\TOOL\CARLA\WindowsNoEditor"
CARLA_EXE = os.path.join(CARLA_DIR, "CarlaUE4.exe")
CARLA_IMAGES = ("CarlaUE4.exe", "CarlaUE4-Win64-Shipping.exe")

# ------------------------- 跑步机 -------------------------
SETTLE_STEPS = 30     # 起步前踩刹车停稳的 tick 数（固定值，见 settle() 注释）
SETTLE_V = 0.05       # settle 判据：速度低于此值视为停稳 (m/s)


def slug(vid):
    """vehicle.tesla.model3 -> tesla_model3（做目录名）。"""
    return vid.replace("vehicle.", "").replace(".", "_")


# ------------------------- 进程管理 -------------------------
def carla_world(timeout=10.0):
    """连得上就返回 world，否则 None（不抛异常 —— 探活要能用）。"""
    try:
        c = carla.Client(HOST, PORT)
        c.set_timeout(timeout)
        return c.get_world()
    except Exception:
        return None


def kill_carla():
    # ★ 不要加 text=True：CARLA 服务端会往 stderr 塞 GBK 字节，
    #   subprocess 的 reader thread 解码会抛 UnicodeDecodeError
    #   （异常在子线程里，主流程只看到一行刺眼的 traceback，很容易当噪音放过去）。
    #   拿 bytes 直接丢掉就行 —— 这里也根本不需要它的输出。
    for img in CARLA_IMAGES:
        subprocess.run(["taskkill", "/F", "/IM", img], capture_output=True)


def launch_carla(map_name=None):
    """低画质启动：我们只要物理，画质越省显存越不容易 OOM 崩。

    map_name 走**启动参数**而不是 `load_world()` —— 后者第二次调用必崩服务端
    （P-SIM-01）。要换图只能重启进程带图启动。
    """
    kill_carla()
    time.sleep(2)
    DETACHED = 0x00000008
    NEW_GROUP = 0x00000200
    argv = [CARLA_EXE, "-carla-rpc-port=2000", "-quality-level=Low",
            "-nosound", "-windowed", "-ResX=960", "-ResY=540"]
    if map_name:
        argv.append(map_name)
    print(f"[carla] 启动 {CARLA_EXE} {'map=' + map_name if map_name else ''} ...")
    subprocess.Popen(
        argv, cwd=CARLA_DIR, creationflags=DETACHED | NEW_GROUP,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL)


def ensure_carla(wait_s=240, map_name=None):
    """确保服务端可用；不在就启动并等就绪。返回 (world, map_name)。

    ★ map_name 给了就必须**当前图也匹配**才算就绪 —— 否则重启。
      不给 = 用当前图（老行为，03~09 不受影响）。
    """
    w = carla_world()
    if w is not None:
        cur = w.get_map().name.split("/")[-1]
        if map_name is None or cur == map_name:
            return w, w.get_map().name
        print(f"[carla] 当前 {cur} != {map_name}，重启换图")
        kill_carla()
        time.sleep(3)
        w = None
    if w is None:
        launch_carla(map_name)
    t0 = time.time()
    while time.time() - t0 < wait_s:
        time.sleep(10)
        w = carla_world()
        if w is not None:
            # 端口通了不等于世界加载完，再稳一拍
            time.sleep(5)
            w = carla_world() or w
            print(f"[carla] 就绪 ({int(time.time() - t0)}s)  map = {w.get_map().name}")
            return w, w.get_map().name
        print(f"[carla] 等待中 ... {int(time.time() - t0)}s")
    raise RuntimeError("CARLA 起不来")


# ------------------------- 跑步机原语 -------------------------
def speed_of(vehicle):
    """车身速度标量 (m/s)。"""
    v = vehicle.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def settle(world, vehicle, steps=SETTLE_STEPS):
    """踩刹车把车停稳，消除 spawn 下落造成的假初速。

    ★ 必须**跑满固定步数**再判，不能一见 v 小就返回 —— spawn 那一瞬车还在空中，
    v 本来就是 0，过早返回会把 transform 抓在半空中；之后整条标定曲线都建在
    一个悬空的车上，油门加不动（实测 theta=0.15 只爬到 0.13 m/s 就冻住，
    而正常应该是 2.0 m/s）。
    """
    for _ in range(steps):
        ctrl = carla.VehicleControl()
        ctrl.throttle, ctrl.brake, ctrl.steer = 0.0, 1.0, 0.0
        vehicle.apply_control(ctrl)
        world.tick()
    return speed_of(vehicle) < SETTLE_V


def tick_pinned(world, vehicle, home, throttle, brake=0.0):
    """跑步机的一步：施加油门/刹车 -> tick -> 拉回原位。返回 tick 后的速度。"""
    ctrl = carla.VehicleControl()
    ctrl.throttle, ctrl.brake, ctrl.steer = throttle, brake, 0.0
    vehicle.apply_control(ctrl)
    world.tick()
    vehicle.set_transform(home)     # ← 关键：位置不动，速度保留
    return speed_of(vehicle)
