"""02_spawn_vehicle.py -- spawn 一辆主车，同步模式让它跑起来。"""
import math
import carla

HOST, PORT ="LOCALHOST",2000
STEPS =200  #200步 x 0.1s =20 秒仿真时间
THROTTLE = 0.5 #恒定油门

# -------------------连上服务端----------------------
client = carla.Client(HOST, PORT)
client.set_timeout(20.0)
world = client.get_world()

#---------------------切同步模式--------------------
settings = world.get_settings()
settings.synchronous_mode =True
settings.fixed_delta_seconds = 0.1
world.apply_settings(settings)

vehicle = None
try:
    #--------------------------1. spawn 主车 ----------------------
    bp =world.get_blueprint_library().find('vehicle.tesla.model3')
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(bp,spawn_point)
    print(f"spawned{vehicle.type_id} id ={vehicle.id}")

    #===========================2. 主循环 -------------------------
    for i in range(STEPS):
        world.tick()
        # 镜头跟在车后上方（车后 8m、抬高 6m、下俯 20°）
        t = vehicle.get_transform()
        loc = t.location - t.get_forward_vector() * 8.0
        loc.z += 6.0
        world.get_spectator().set_transform(
            carla.Transform(loc, carla.Rotation(pitch=-20.0, yaw=t.rotation.yaw))
        )

        v = vehicle.get_velocity()
        speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

        ctrl = carla.VehicleControl()
        ctrl.throttle = THROTTLE
        ctrl.brake = 0.0
        ctrl.steer = 0.0
        vehicle.apply_control(ctrl)

        if i % 20 == 0:
            print(f"step {i:3d}  speed = {speed:6.2f} m/s")

finally:
    #-----------------------3.收尾 ---------------------------------
    if vehicle is not None:
        vehicle.destroy()
    settings.synchronous_mode = False
    world.apply_settings(settings)
    print("[OK] cleaned up, sync mode off.")    
