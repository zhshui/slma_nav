#!/bin/bash
# ============================================
# go2_nav Web 自启动脚本
# 启动顺序: roscore → rosbridge → gateway → vite
# ============================================
LOG_DIR="/home/robot/go2_nav/logs"
mkdir -p "$LOG_DIR"

echo "[startup] $(date) ====== 启动 go2_nav web 服务 ======"

# 0. 清理残留端口（防止上次异常退出后端口被占用）
echo "[startup] 0/4 清理残留端口 ..."
for port in 11311 9090 8080 5173; do
    PID=$(ss -tlnp | grep ":${port} " | grep -oP 'pid=\K\d+' 2>/dev/null)
    if [ -n "$PID" ]; then
        echo "[startup]   清理端口 ${port} (pid=${PID})"
        kill -9 "$PID" 2>/dev/null
    fi
done
sleep 1

# 1. roscore
echo "[startup] 1/4 启动 roscore ..."
source /opt/ros/noetic/setup.bash
pkill -f rosmaster 2>/dev/null
sleep 1
roscore &>"$LOG_DIR/roscore.log" &
sleep 3

# 检查 roscore (11311)
for i in $(seq 1 10); do
    if ss -tlnp | grep -q 11311; then
        echo "[startup]   roscore OK"
        break
    fi
    sleep 1
done
if ! ss -tlnp | grep -q 11311; then
    echo "[startup]   roscore FAILED!"
    exit 1
fi

# 2. rosbridge
echo "[startup] 2/4 启动 rosbridge ..."
roslaunch rosbridge_server rosbridge_websocket.launch &>"$LOG_DIR/rosbridge.log" &
sleep 2

for i in $(seq 1 15); do
    if ss -tlnp | grep -q 9090; then
        echo "[startup]   rosbridge OK (0.0.0.0:9090)"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 9090; then
    echo "[startup]   rosbridge FAILED!"
    exit 1
fi

# 3. gateway
echo "[startup] 3/4 启动 gateway ..."
cd /home/robot/go2_nav/ros_web_gui_app/gateway
npm run dev &>"$LOG_DIR/gateway.log" &
sleep 2

for i in $(seq 1 15); do
    if ss -tlnp | grep -q 8080; then
        echo "[startup]   gateway OK (0.0.0.0:8080)"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 8080; then
    echo "[startup]   gateway FAILED!"
    exit 1
fi

# 4. vite
echo "[startup] 4/4 启动 vite ..."
cd /home/robot/go2_nav/ros_web_gui_app
npm run dev &>"$LOG_DIR/vite.log" &
sleep 2

for i in $(seq 1 15); do
    if ss -tlnp | grep -q 5173; then
        echo "[startup]   vite OK (0.0.0.0:5173)"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 5173; then
    echo "[startup]   vite FAILED!"
    exit 1
fi

# 获取 WiFi IP（优先无线网卡，否则取第一个 IP）
WIFI_IP=$(ip -o -4 addr show | grep -E 'wl[apn]' | awk '{print $4}' | cut -d/ -f1 | head -1)
if [ -z "$WIFI_IP" ]; then
    WIFI_IP=$(hostname -I | awk '{print $1}')
fi

echo "[startup] $(date) ====== 全部启动完成 ====="
echo "[startup] 其他设备访问: http://${WIFI_IP}:8080"
