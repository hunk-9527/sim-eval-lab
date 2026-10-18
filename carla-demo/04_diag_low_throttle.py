"""04_diag_low_throttle.py -- 判决：跑步机法在低速段是否引入了伪影。

疑点
----
03 标定扫描里 theta=0.10 与 theta=0.15 两条速度 trace **逐位相同**
（0.0924 -> 0.1326，然后冻在 0.1326 不动），theta=0.20 的前 12 步也完全一样。
不同油门给出相同曲线 = 前 12 步是**与油门无关**的东西，之后才冻住。
而且这跟昨天开车法测到的"theta=0.15 能走到 2.17 m/s"直接矛盾。

两种假设，本脚本判决：
  H1 跑步机伪影 —— set_transform 每 tick 回拉，物理引擎把它读成运动，
     造出一个 ~0.13 m/s 的假底速，低速段被它淹没
  H2 真死区 —— 0.15 以下油门真的推不动车（但解释不了"相同 trace"）

四组对照
  A  pin  + throttle=0.00    -> 若也爬到 0.13 则 H1 成立（油门=0 不可能有速度）
  B  pin  + throttle=0.15
  C  free + throttle=0.15    -> 开车法基准，验昨天的 2.17 m/s
  D  free + throttle=0.20
"""

import carla
import math

HOST, PORT = "localhost", 2000
DT = 0.1
SPAWN_IDX = 2


def speed_of(v):
    u = v.get_velocity()
    return math.sqrt(u.x ** 2 + u.y ** 2 + u.z ** 2)


def make(world):
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    sp = world.get_map().get_spawn_points()[SPAWN_IDX]
    return world.try_spawn_actor(bp, sp)


def settle(world, veh, n=25):
    for _ in range(n):
        c = carla.VehicleControl()
        c.throttle, c.brake = 0.0, 1.0
        veh.apply_control(c)
        world.tick()
    return speed_of(veh)


def run(world, veh, throttle, steps, pin):
    home = veh.get_transform()
    out = []
    for _ in range(steps):
        c = carla.VehicleControl()
        c.throttle, c.brake, c.steer = throttle, 0.0, 0.0
        veh.apply_control(c)
        world.tick()
        if pin:
            veh.set_transform(home)
        out.append(speed_of(veh))
    return out


def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DT
    world.apply_settings(settings)

    cases = [
        ("A  pin  theta=0.00", 0.00, 60, True),
        ("B  pin  theta=0.15", 0.15, 60, True),
        ("C  free theta=0.15", 0.15, 150, False),
        ("D  free theta=0.20", 0.20, 150, False),
    ]
    try:
        for name, th, steps, pin in cases:
            veh = make(world)
            if veh is None:
                print(f"{name}: spawn 失败")
                continue
            try:
                v_after_settle = settle(world, veh)
                vs = run(world, veh, th, steps, pin)
                marks = [(i, vs[i]) for i in (0, 5, 10, 20, 40, 60, 100, 149)
                         if i < len(vs)]
                line = "  ".join(f"t={i*DT:.1f}s {v:.4f}" for i, v in marks)
                print(f"{name} | settle 后 v={v_after_settle:.4f} | {line}")
                print(f"{'':>22} max v = {max(vs):.4f}")
            finally:
                veh.destroy()
                world.tick()
    finally:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("\n[sync] off")


if __name__ == "__main__":
    main()
