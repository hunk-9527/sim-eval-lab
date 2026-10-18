"""carla_map.py -- 切换 CARLA 默认地图（0.9.15 打包版只认 DefaultEngine.ini）。

★ 为什么需要这个
----------------
`start_carla.bat` 里的 `%~1` 地图参数**一直是空转** —— CARLA 0.9.15 的打包版
**忽略命令行地图参数**。实测三种写法全部无效，启动后仍是 ini 里写死的图：
    CarlaUE4.exe ... Town04
    CarlaUE4.exe ... /Game/Carla/Maps/Town04
    （带不带 -carla-rpc-port 都一样）
→ 换图只能**改 ini + 重启进程**。（见踩坑库 P-SIM-06）

★ 为什么不用 load_world()
------------------------
`client.load_world()` 第二次调用必崩服务端（P-SIM-01：一进程一图）。所以换图 = 重启。

用法（改完必须重启 CARLA 才生效）
--------------------------------
    python carla_map.py current
    python carla_map.py set Town04      # 首次会备份
    python carla_map.py restore         # 还原到备份
"""

import os
import shutil
import sys

CARLA_DIR = r"D:\TOOL\CARLA\WindowsNoEditor"
INI = os.path.join(CARLA_DIR, "CarlaUE4", "Config", "DefaultEngine.ini")
BAK = INI + ".bak-map"

KEYS = ("EditorStartupMap", "GameDefaultMap", "ServerDefaultMap")


def _lines(path):
    return open(path, encoding="utf-8").read().splitlines(keepends=True)


def read_map():
    for ln in _lines(INI):
        s = ln.strip()
        if s.startswith("GameDefaultMap="):
            return s.split("=", 1)[1].strip()
    return None


def _patch(text, map_name):
    """把三处 map 键替换成 /Game/Carla/Maps/<map>.<map>，返回 (新文本, 命中数)。"""
    target = f"/Game/Carla/Maps/{map_name}.{map_name}"
    out, hits = [], 0
    for ln in text.splitlines(keepends=True):
        s = ln.strip()
        key = next((k for k in KEYS if s.startswith(k + "=")), None)
        if key:
            eol = "\r\n" if ln.endswith("\r\n") else "\n"
            out.append(f"{key}={target}{eol}")
            hits += 1
        else:
            out.append(ln)
    return "".join(out), hits


def set_map(map_name):
    raw = open(INI, encoding="utf-8").read()
    new, hits = _patch(raw, map_name)
    if hits == 0:
        raise RuntimeError(f"{INI} 里没找到 {KEYS} —— 格式变了，别硬改")
    if not os.path.exists(BAK):
        shutil.copyfile(INI, BAK)
        print(f"[map] 已备份 -> {BAK}")
    open(INI, "w", encoding="utf-8", newline="").write(new)
    print(f"[map] 已设 {map_name}（命中 {hits} 处）：{read_map()}")
    print("[map] ★ 重启 CARLA 才生效")


def restore():
    if not os.path.exists(BAK):
        print("[map] 没有备份，跳过"); return 1
    shutil.copyfile(BAK, INI)
    print(f"[map] 已还原：{read_map()}")
    print("[map] ★ 重启 CARLA 才生效")
    return 0


def main(argv):
    cmd = argv[1] if len(argv) > 1 else "current"
    if cmd == "current":
        print(read_map()); return 0
    if cmd == "set":
        if len(argv) < 3:
            print("用法: carla_map.py set <MapName>"); return 1
        set_map(argv[2]); return 0
    if cmd == "restore":
        return restore()
    print(__doc__); return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
