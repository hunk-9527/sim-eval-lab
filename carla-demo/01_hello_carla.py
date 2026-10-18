"""
01_hello_carla.py —— 第一次连上 CARLA。

做三件事：
  1. 连上仿真服务端，问它是什么版本、当前是哪张地图
  2. 在场景里放一个摄像头传感器
  3. 取一帧图像存成 PNG，让你看见 CARLA 渲染出来是什么样

运行前：CarlaUE4.exe 必须已经开着（服务端），本脚本是"客户端"。
"""

import queue

import carla

HOST = "localhost"
PORT = 2000

# ---------- 1. 连接 ----------
client = carla.Client(HOST, PORT)
client.set_timeout(20.0)

print("[1] server version :", client.get_server_version())
print("[2] client version :", client.get_client_version())

world = client.get_world()
carla_map = world.get_map()

print("[3] current map    :", carla_map.name)
print("[4] spawn points   :", len(carla_map.get_spawn_points()))
print("[5] actors alive   :", len(world.get_actors()))
print("[6] available maps :", len(client.get_available_maps()))

weather = world.get_weather()
print("[7] weather        : cloudiness=%.1f  sun_altitude=%.1f  rain=%.1f"
      % (weather.cloudiness, weather.sun_altitude_angle, weather.precipitation))

# ---------- 2. 放一个摄像头 ----------
spawn_points = carla_map.get_spawn_points()
if not spawn_points:
    raise RuntimeError("this map has no spawn points")

# 取第 1 个出生点，抬高 2 米模拟驾驶员视线高度，镜头略微下俯
cam_transform = carla.Transform(
    carla.Location(
        x=spawn_points[0].location.x,
        y=spawn_points[0].location.y,
        z=spawn_points[0].location.z + 2.0,
    ),
    carla.Rotation(pitch=-5.0, yaw=spawn_points[0].rotation.yaw, roll=0.0),
)

cam_bp = world.get_blueprint_library().find("sensor.camera.rgb")
cam_bp.set_attribute("image_size_x", "1280")
cam_bp.set_attribute("image_size_y", "720")
cam_bp.set_attribute("fov", "90")

image_queue = queue.Queue()
camera = world.spawn_actor(cam_bp, cam_transform)
camera.listen(image_queue.put)
print("[8] camera spawned : id=%s  @ (%.1f, %.1f, %.1f)"
      % (camera.id, cam_transform.location.x, cam_transform.location.y, cam_transform.location.z))

# ---------- 3. 取一帧 ----------
image = image_queue.get(timeout=30.0)
out_path = "D:/AI/myProject/learning/projects/carla-demo/out/first_frame.png"
image.save_to_disk(out_path)
print("[9] frame captured : #%d  %dx%d" % (image.frame, image.width, image.height))
print("[10] saved to      :", out_path)

# 顺手把 CarlaUE4 那个窗口的"上帝视角"也移到同一位置，
# 这样你切过去看到的画面和这张 PNG 是同一处
spectator = world.get_spectator()
spectator.set_transform(carla.Transform(
    cam_transform.location + carla.Location(z=12.0),
    carla.Rotation(pitch=-30.0, yaw=cam_transform.rotation.yaw, roll=0.0),
))

camera.destroy()
print("[OK] done.")
