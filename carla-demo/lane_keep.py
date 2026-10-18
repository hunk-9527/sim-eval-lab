"""lane_keep.py -- 沿车道中心线行驶的简易控制器（纯追踪 / pure pursuit）。

为什么需要它
------------
CARLA 里主车如果 steer=0 直线开，路一弯就压护栏。实测 Town04 上直着开
90~140 m 必撞 static.guardrail。标定实验、ACC 闭环都要求主车能长时间
沿路行驶，所以单独抽一个可复用的横向控制器。

做法
----
每个控制周期：
  1. 用车辆当前位置在地图上找到所在车道 -> get_waypoint
  2. 取前方 lookahead 米处的航点作为追踪目标（路口有多条时，选航向变化
     最小的那条 = "继续往前开"，避免在每个路口随机转弯）
  3. 把目标点变换到车体坐标系，用它的方位角算转向量（比例控制）

用法
----
    from lane_keep import LaneKeeper
    lk = LaneKeeper(world.get_map(), lookahead=6.0)
    steer = lk.steer(vehicle)          # 每 tick 调用，返回 -1..1

自检（确认 STEER_SIGN 方向没搞反）
    PYTHONIOENCODING=utf-8 python lane_keep.py --probe
"""

import argparse
import math

import carla

# CARLA 的 VehicleControl.steer 正负号与"目标在车体哪一侧"的对应关系。
# 不同文档/教程说法不一，靠 --probe 实测确定，不靠记忆。
STEER_SIGN = 1.0

THROTTLE_PROBE = 0.5
PROBE_STEPS = 60


def wrap_deg(deg):
    """把角度归一化到 (-180, 180]。"""
    return (deg + 180.0) % 360.0 - 180.0


def speed_of(vehicle):
    """车身速度标量 (m/s)。"""
    v = vehicle.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def vehicle_local(origin, point):
    """把 point(Location) 投到 origin(Transform) 的车体坐标系，返回 (前向, 右向)。

    不用 Transform.inverse_transform() —— 这个 CARLA 版本没实现该方法。
    用 get_forward_vector / get_right_vector 自己投影，避免手推坐标系约定出错。
    """
    f = origin.get_forward_vector()
    r = origin.get_right_vector()
    dx = point.x - origin.location.x
    dy = point.y - origin.location.y
    return dx * f.x + dy * f.y, dx * r.x + dy * r.y


class LaneKeeper:
    """前瞻距离随车速增长（pure pursuit 的标准做法）。

    固定 6 m 前瞻在高速下会画龙、甚至掉出路面 —— 实测 theta=0.8/0.9 两档
    在 25~32 m/s 出现 a = -8 m/s^2 的乱跳，就是掉出路沿造成的。
    改成 lookahead = max(min_lookahead, gain_la * v) 后高速段才稳。
    """

    def __init__(self, carla_map, lookahead=6.0, gain_la=0.6,
                 gain=1.4, max_steer=0.5):
        self.map = carla_map
        self.lookahead = lookahead
        self.gain_la = gain_la
        self.gain = gain
        self.max_steer = max_steer

    def target(self, vehicle):
        """返回前方 lookahead 处的目标 transform（找不到返回 None）。"""
        tr = vehicle.get_transform()
        wp = self.map.get_waypoint(tr.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        la = max(self.lookahead, self.gain_la * speed_of(vehicle))
        nxt = wp.next(la)
        if not nxt:
            return None
        # 路口会有多条候选，挑航向变化最小的（= 相当于继续直行）
        best = min(nxt, key=lambda w: abs(wrap_deg(w.transform.rotation.yaw
                                                   - tr.rotation.yaw)))
        return best.transform

    def lateral_offset(self, vehicle):
        """车辆相对所在车道中心线的横向偏移 (m)，带符号；用来判断有没有跑偏。"""
        tr = vehicle.get_transform()
        wp = self.map.get_waypoint(tr.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        _, right_m = vehicle_local(wp.transform, tr.location)
        return right_m

    def steer(self, vehicle):
        tgt = self.target(vehicle)
        if tgt is None:
            return 0.0
        tr = vehicle.get_transform()
        fwd_m, right_m = vehicle_local(tr, tgt.location)
        if fwd_m <= 0.1:                       # 目标在正后方，别硬拐
            return 0.0
        ang = math.atan2(right_m, fwd_m)
        s = STEER_SIGN * ang * self.gain
        return max(-self.max_steer, min(self.max_steer, s))


# --------------------------- 自检探针 ---------------------------
def probe(map_name="Town04"):
    """实测 CARLA 的 steer 正负号：施加恒定正转向，看车往哪一侧偏。

    为什么不能"让车跑一段看偏移收不收敛"：出生点若是直道，车根本不偏，
    偏移全程 ~0.03 m（纯噪声），判不出方向 —— 第一版探针就是这么翻车的。

    改成主动施加已知输入：
      1. 直线加速到 ~8 m/s（此段无弯曲干扰，保证后续位移归因于转向）
      2. 施加恒定 steer=+0.3 跑 40 tick，看横向偏移往哪边走
    偏移变正 = 正转向把车推向车道中心线右侧
    -> "目标在右侧就给正转向" 成立 -> STEER_SIGN = +1。
    """
    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    print(f"[probe] map = {world.get_map().name} (期望 {map_name})")

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1
    world.apply_settings(settings)

    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    vehicle = world.try_spawn_actor(bp, world.get_map().get_spawn_points()[0])
    lk = LaneKeeper(world.get_map())
    try:
        for _ in range(20):                       # settle
            c = carla.VehicleControl()
            c.throttle, c.brake = 0.0, 1.0
            vehicle.apply_control(c)
            world.tick()

        for _ in range(60):                       # 直线加速，不转向
            c = carla.VehicleControl()
            c.throttle, c.brake, c.steer = 0.5, 0.0, 0.0
            vehicle.apply_control(c)
            world.tick()
        v0 = math.sqrt(sum(x * x for x in (lambda u: (u.x, u.y, u.z))(
            vehicle.get_velocity())))
        print(f"[probe] 直线加速结束，v = {v0:.2f} m/s")

        yaw0 = vehicle.get_transform().rotation.yaw
        offs = []
        for i in range(40):                       # 恒定正转向
            c = carla.VehicleControl()
            c.throttle, c.brake, c.steer = 0.3, 0.0, +0.3
            vehicle.apply_control(c)
            world.tick()
            offs.append(lk.lateral_offset(vehicle))
            if i % 10 == 0:
                print(f"  step={i:3d}  横向偏移={offs[-1]:+7.3f} m")
        yaw1 = vehicle.get_transform().rotation.yaw
    finally:
        vehicle.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)

    d_off, d_yaw = offs[-1] - offs[0], wrap_deg(yaw1 - yaw0)
    print(f"\n[probe] 恒定 steer=+0.3 :  偏移 {offs[0]:+.3f} -> {offs[-1]:+.3f} m "
          f"(Δ={d_off:+.3f}) | Δyaw = {d_yaw:+.1f}°")
    if abs(d_off) < 0.3:
        print("[probe] ⚠ 位移太小，判不出来 —— 换个起点或加大转向再试")
        return
    sign = 1.0 if d_off > 0 else -1.0
    print(f"[probe] 结论：正转向使车向{'右' if sign > 0 else '左'}偏 "
          f"-> STEER_SIGN 应为 {sign:+.0f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--map", default="Town04")
    a = ap.parse_args()
    if a.probe:
        probe(a.map)
    else:
        print(__doc__)
