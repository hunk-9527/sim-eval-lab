@echo off
REM ============================================================
REM  启动 CARLA 服务端（标定 / SIL 闭环用）
REM
REM  为什么参数是这样：
REM    -quality-level=Low   我们只用物理，低画质大幅省显存 -> 降低 OOM 崩溃率
REM    -nosound             少一个可能出问题的子系统
REM    -windowed -ResX/Y    小窗口，同上
REM
REM  ★ 不要用 client.load_world() 切地图：实测第二次调用必崩服务端。
REM    要换图请先关掉 CARLA，再用本脚本带地图名启动。
REM ============================================================

setlocal
set CARLA_DIR=D:\TOOL\CARLA\WindowsNoEditor
if not "%~1"=="" (
    echo [start_carla] map = %~1
    start "" "%CARLA_DIR%\CarlaUE4.exe" -carla-rpc-port=2000 -quality-level=Low -nosound -windowed -ResX=960 -ResY=540 %~1
) else (
    echo [start_carla] map = 默认图
    start "" "%CARLA_DIR%\CarlaUE4.exe" -carla-rpc-port=2000 -quality-level=Low -nosound -windowed -ResX=960 -ResY=540
)
echo [start_carla] 已发起启动，等待服务端就绪（约 20-60 s）...
echo [start_carla] 就绪判据： netstat -ano ^| findstr :2000
endlocal
