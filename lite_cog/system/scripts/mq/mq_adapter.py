#!/usr/bin/env python3
"""
MQ Adapter v3: RabbitMQ/MQTT ↔ ROS 导航桥接
支持 AMQP (pika) 和 MQTT (paho-mqtt)，通过 MQ_TYPE 环境变量切换
"""
import json, os, sqlite3, subprocess, threading, time, uuid, signal, math
from datetime import datetime, timezone
import urllib.request
from typing import Optional

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String as RosString
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# ====== 默认环境变量（可通过 export 覆盖） ======
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# mq_adapter.py → mq/ → scripts/ → system/ → lite_cog/ → go2_nav/
NAV_ROOT = os.environ.get("NAV_ROOT", os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))))

MQ_TYPE = os.environ.get("MQ_TYPE", "mqtt")                 # rabbitmq | mqtt
MQ_HOST = os.environ.get("MQ_HOST", "49.233.183.203")
MQ_PORT = int(os.environ.get("MQ_PORT", "1883"))
MQ_USER = os.environ.get("MQ_USER", "server")
MQ_PASS = os.environ.get("MQ_PASS", "5bP!8aS3$kD7vF2&")
MQ_VHOST = os.environ.get("MQ_VHOST", "/")
MQ_EXCHANGE = os.environ.get("MQ_EXCHANGE", "nav.exchange")
MQ_CLIENT_ID = os.environ.get("MQ_CLIENT_ID", "nav-robot-xx1")
GATEWAY_DIR = os.environ.get("GATEWAY_DIR", os.path.join(NAV_ROOT, "ros_web_gui_app/gateway"))
DB_PATH = os.path.join(GATEWAY_DIR, "data", "nav_web.sqlite")
SWITCH_HOOK = os.environ.get("MAP_SWITCH_HOOK", os.path.join(NAV_ROOT, "lite_cog/system/scripts/slam/switch_map_hook.sh"))
TASK_SCRIPT = os.environ.get("TASK_SCRIPT", os.path.join(NAV_ROOT, "lite_cog/pipeline/src/pipeline_tracking/scripts/Task.py"))
TASK_DATA = os.environ.get("TASK_DATA_DIR", os.path.join(NAV_ROOT, "lite_cog/pipeline/src/pipeline_tracking/data"))
NAV_SCRIPT = os.environ.get("NAV_START_SCRIPT", os.path.join(NAV_ROOT, "lite_cog/system/scripts/nav/start_nav.sh"))
RVIZ_CFG = os.environ.get("RVIZ_CONFIG", os.path.join(NAV_ROOT, "lite_cog/nav/src/hdl_localization/rviz/hdl_localization.rviz"))
TF_MAP_FRAME = os.environ.get("TF_MAP_FRAME", "map")
TF_BODY_FRAME = os.environ.get("TF_BODY_FRAME", "base_link")

# topic: AMQP 用 . 分隔 (nav.robot-001.cmd)，MQTT 用 / 分隔 (nav/robot-001/cmd)
_SEP = "/" if MQ_TYPE == "mqtt" else "."
CMD_TOPIC = f"nav{_SEP}{MQ_CLIENT_ID}{_SEP}cmd"

# ====== MQ 协议抽象 ======
if MQ_TYPE == "mqtt":
    import paho.mqtt.client as mqtt_lib

    _mq_client = mqtt_lib.Client(client_id=MQ_CLIENT_ID, protocol=mqtt_lib.MQTTv311)
    _mq_client.username_pw_set(MQ_USER, MQ_PASS)
    _mq_pub_lock = threading.Lock()
    _mq_msg_queue = []  # 消费线程把消息放到这里
    _mq_msg_lock = threading.Lock()

    def mq_connect():
        _mq_client.connect(MQ_HOST, MQ_PORT, keepalive=30)
        _mq_client.loop_start()
        rospy.loginfo(f"[MQ] MQTT connected {MQ_HOST}:{MQ_PORT}")

    def mq_publish(topic_suffix, body):
        routing = f"nav/{MQ_CLIENT_ID}/{topic_suffix}"
        try:
            with _mq_pub_lock:
                _mq_client.publish(routing, json.dumps(body, ensure_ascii=False), qos=int(body.get("_qos", 1)))
        except Exception as e:
            rospy.logerr(f"[MQ] publish {routing}: {e}")

    def mq_subscribe(topic_suffix, qos=1):
        topic = f"nav/{MQ_CLIENT_ID}/{topic_suffix}"
        _mq_client.subscribe(topic, qos=qos)

    def mq_get_messages():
        with _mq_msg_lock:
            msgs = list(_mq_msg_queue)
            _mq_msg_queue.clear()
        return msgs

    def _on_mq_message(client, userdata, msg):
        with _mq_msg_lock:
            _mq_msg_queue.append(msg)

    _mq_client.on_message = _on_mq_message

else:
    import pika as _pika_mod

    _mq_conn = None
    _mq_ch_pub = None

    def mq_connect():
        global _mq_conn, _mq_ch_pub
        params = _pika_mod.ConnectionParameters(
            host=MQ_HOST, port=MQ_PORT, virtual_host=MQ_VHOST,
            credentials=_pika_mod.PlainCredentials(MQ_USER, MQ_PASS), heartbeat=30)
        _mq_conn = _pika_mod.BlockingConnection(params)
        _mq_ch_pub = _mq_conn.channel()
        _mq_ch_pub.exchange_declare(exchange=MQ_EXCHANGE, exchange_type="topic", durable=True)
        rospy.loginfo(f"[MQ] AMQP connected {MQ_HOST}:{MQ_PORT}")

    _pub_lock = threading.Lock()
    def mq_publish(topic_suffix, body):
        routing = f"nav.{MQ_CLIENT_ID}.{topic_suffix}"
        with _pub_lock:
            try:
                if _mq_ch_pub and not _mq_ch_pub.is_closed:
                    _mq_ch_pub.basic_publish(exchange=MQ_EXCHANGE, routing_key=routing,
                                             body=json.dumps(body, ensure_ascii=False))
            except Exception as e:
                rospy.logerr(f"[MQ] publish {routing}: {e}")

    def mq_subscribe(topic_suffix, qos=1):
        pass  # AMQP consumer 单独处理

    def mq_get_messages():
        return []  # AMQP 消费不用队列

# ====== 工具 ======
def _hdr(msg_type):
    return {"msg_type": msg_type}

def _ack(ref, cmd, result, reason=""):
    mq_publish("cmd.ack", {"header": _hdr("cmd_ack"),
        "body": {"ref_msg_id": ref, "cmd": cmd, "result": result, "reason": reason}})

def get_map_list():
    if not os.path.exists(DB_PATH): return []
    try:
        conn = sqlite3.connect(DB_PATH); conn.row_factory = sqlite3.Row
        rows = conn.execute("SELECT id,name,yaml_path,pcd_path,created_at,active FROM maps ORDER BY created_at DESC").fetchall()
        conn.close()
        return [{"id": r["id"], "name": r["name"], "yaml_path": r["yaml_path"],
                 "pcd_path": r["pcd_path"] or "", "created_at": r["created_at"],
                 "active": bool(r["active"])} for r in rows]
    except Exception as e:
        rospy.logerr(f"[MQ] DB: {e}"); return []

# ====== 状态 ======
class NavState:
    def __init__(self):
        self.lock = threading.Lock()
        self.current_map = ""; self.last_goal = None; self.task_proc = None
    def to_dict(self):
        with self.lock:
            return {"current_map": self.current_map}
state = NavState()

# ====== move_base ======
class MBClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.ok = False
    def connect(self, t=10): self.ok = self.client.wait_for_server(rospy.Duration(t)); return self.ok
    def send(self, x, y, yaw, frame="map"):
        g = MoveBaseGoal()
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.header.frame_id = frame
        g.target_pose.pose.position.x = x; g.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        g.target_pose.pose.orientation.x = q[0]; g.target_pose.pose.orientation.y = q[1]
        g.target_pose.pose.orientation.z = q[2]; g.target_pose.pose.orientation.w = q[3]
        self.client.send_goal(g)
    def cancel(self): self.client.cancel_all_goals()
    def state(self): return self.client.get_state()
mb = MBClient()
init_pub = None

def _kill_task():
    with state.lock: p = state.task_proc
    if p and p.poll() is None:
        p.send_signal(signal.SIGINT)
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired: p.kill()
    with state.lock: state.task_proc = None

# ====== 指令处理 ======
def h_map_list(body, ref):
    maps = get_map_list()
    mq_publish("map_list", {"header": _hdr("map_list"), "body": {"maps": maps, "total": len(maps)}})
    _ack(ref, "map_list", "accepted")

def _switch_and_init(mid):
    """切换地图并发布 initialpose 到原点"""
    maps = get_map_list(); t = None
    for m in maps:
        if m["id"] == mid or m["name"] == mid: t = m; break
    if t:
        try:
            rospy.loginfo(f"[MQ] switching map: {t['name']} ({t['id'][:8]}...)")
            subprocess.run(["bash", SWITCH_HOOK, t["yaml_path"], t.get("pcd_path",""), t["name"]],
                           capture_output=True, timeout=30)
            with state.lock: state.current_map = t["name"]
            ip = PoseWithCovarianceStamped()
            ip.header.stamp = rospy.Time.now(); ip.header.frame_id = "map"
            ip.pose.pose.orientation.w = 1.0
            if init_pub: init_pub.publish(ip)
        except: pass
    else:
        rospy.logwarn(f"[MQ] map not found: {mid}")

def _ensure_nav_stack():
    """确保导航栈（hdl_localization + rviz）已启动"""
    # 如果 move_base 未就绪，启动整个导航栈
    if not mb.ok:
        rospy.loginfo("[MQ] nav stack not ready, launching...")
        try:
            subprocess.Popen(["bash", NAV_SCRIPT],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
            for _ in range(60):
                if mb.connect(): break
                time.sleep(0.5)
        except Exception as e:
            rospy.logwarn(f"[MQ] nav stack launch failed: {e}")
    # rviz disabled: web UI replaces visualization
    # try:
    #     if subprocess.run(["pgrep", "-f", "rviz"], capture_output=True).returncode != 0:
    #         rospy.loginfo("[MQ] launching rviz...")
    #         subprocess.Popen(["bash", "-c",
    #             f"source /opt/ros/noetic/setup.bash && rviz -d {RVIZ_CFG}"],
    #             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    #             start_new_session=True)
    # except Exception as e:
    #     rospy.logwarn(f"[MQ] rviz launch failed: {e}")

def h_nav_single(body, ref):
    g = body.get("goal", {})
    x, y, yaw = g.get("x"), g.get("y"), g.get("yaw")
    if x is None: _ack(ref, "nav_single", "rejected", "need x/y/yaw"); return
    mid = body.get("map_id", "")
    if mid: _switch_and_init(mid)
    # 同時啟動雷達和運控
    _call_gateway("/api/lidar/start", {})
    _call_gateway("/api/motion/start", {})
    # gateway 启导航栈 + 通过 gateway /api/nav/simple-goal 发 goal（和 web 2D Nav Goal 完全一致）
    # 始终用 "map" 坐标系，和 web 前端一致（camera_init 没有到 map 的 TF，move_base 会丢弃 goal）
    _gateway_nav_cmd("nav-only")
    rospy.sleep(5.0)  # 等 move_base 启动
    ok, msg = _call_gateway("/api/nav/simple-goal", {"x": float(x), "y": float(y), "yaw": float(yaw), "frame_id": "map"})
    rospy.loginfo(f"[MQ] simple-goal via gateway: ok={ok} x={x} y={y} yaw={yaw}")
    mq_publish("nav_points", {"header": _hdr("nav_points"),
        "body": {"points": [{"id": "goal", "x": float(x), "y": float(y), "yaw": float(yaw)}]}})
    with state.lock: state.last_goal = body
    _ack(ref, "nav_single", "accepted")

def h_nav_multi(body, ref):
    wps = body.get("waypoints", [])
    if not wps: _ack(ref, "nav_multi", "rejected", "need waypoints"); return
    mid = body.get("map_id", "")
    if mid: _switch_and_init(mid)
    os.makedirs(TASK_DATA, exist_ok=True)
    for f in os.listdir(TASK_DATA):
        if f.endswith(".json"): os.unlink(os.path.join(TASK_DATA, f))
    for i, wp in enumerate(wps):
        y = float(wp.get("yaw", 0))
        rec = {"order": i, "robot_pose": {"pos_x": float(wp["x"]), "pos_y": float(wp["y"]), "pos_z": 0,
                "ori_x": 0, "ori_y": 0, "ori_z": math.sin(y/2), "ori_w": math.cos(y/2)},
               "option": {"even_low_speed": False, "even_medium_speed": False, "uneven_high_step": False}}
        with open(os.path.join(TASK_DATA, f"{i}.json"), "w") as fp: json.dump(rec, fp)
    # 发布途经点到 MQ，供 Web 端显示
    nav_pts = [{"id": wp.get("id", f"wp-{i}"), "x": float(wp["x"]), "y": float(wp["y"]),
                "yaw": float(wp.get("yaw", 0))} for i, wp in enumerate(wps)]
    mq_publish("nav_points", {"header": _hdr("nav_points"), "body": {"points": nav_pts}})
    # 同時啟動雷達和運控
    _call_gateway("/api/lidar/start", {})
    _call_gateway("/api/motion/start", {})
    # gateway 启导航栈 + Task.py，设状态为 running
    _gateway_nav_cmd("start")
    _ack(ref, "nav_multi", "accepted")

def h_nav_pause(body, ref):
    mb.cancel()
    _gateway_nav_cmd("pause")
    _ack(ref, "nav_pause", "accepted")

def h_nav_cancel(body, ref):
    global mb
    mb.cancel()
    _gateway_nav_cmd("stop")
    state.last_goal = None
    mb = MBClient()
    _ack(ref, "nav_cancel", "accepted")

def h_nav_resume(body, ref):
    if state.last_goal:
        _gateway_nav_cmd("resume")
        h_nav_single(state.last_goal, ref)
    else: _ack(ref, "nav_resume", "rejected", "no previous goal")

def h_switch_map(body, ref):
    mid = body.get("map_id", "") or body.get("map_name", "")
    if not mid:
        _ack(ref, "switch_map", "rejected", "need map_id or map_name")
        return
    _switch_and_init(mid)
    # 切换后推送更新后的地图列表
    maps = get_map_list()
    mq_publish("map_list", {"header": _hdr("map_list"), "body": {"maps": maps, "total": len(maps)}})
    _ack(ref, "switch_map", "accepted")

def h_relocalize(body, ref):
    pose = body.get("pose", {})
    x, y, yaw = pose.get("x"), pose.get("y"), pose.get("yaw")
    if x is None or y is None or yaw is None:
        _ack(ref, "relocalize", "rejected", "need pose.x, pose.y, pose.yaw")
        return
    frame = body.get("frame_id", "map")
    if init_pub:
        ip = PoseWithCovarianceStamped()
        ip.header.stamp = rospy.Time.now()
        ip.header.frame_id = str(frame)
        ip.pose.pose.position.x = float(x)
        ip.pose.pose.position.y = float(y)
        q = tf.transformations.quaternion_from_euler(0, 0, float(yaw))
        ip.pose.pose.orientation.x = q[0]
        ip.pose.pose.orientation.y = q[1]
        ip.pose.pose.orientation.z = q[2]
        ip.pose.pose.orientation.w = q[3]
        init_pub.publish(ip)
    _ack(ref, "relocalize", "accepted")
    rospy.loginfo(f"[MQ] relocalize: x={x}, y={y}, yaw={yaw}, frame={frame}")

# ====== 运控指令 ======
GATEWAY_URL = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080")
GATEWAY_USER = os.environ.get("GATEWAY_USER", "admin")
GATEWAY_PASS = os.environ.get("GATEWAY_PASS", "admin123")
_gateway_token = None
cmd_vel_pub = None
sport_cmd_pub = None

def _gateway_login():
    global _gateway_token
    try:
        data = json.dumps({"username": GATEWAY_USER, "password": GATEWAY_PASS}).encode()
        req = urllib.request.Request(f"{GATEWAY_URL}/api/auth/login", data=data,
                                      headers={"Content-Type": "application/json"}, method="POST")
        resp = json.loads(urllib.request.urlopen(req, timeout=5).read())
        _gateway_token = resp.get("token", "")
        return bool(_gateway_token)
    except Exception as e:
        rospy.logwarn(f"[MQ] gateway login failed: {e}")
        return False

def _call_gateway(endpoint, data=None, method="POST"):
    global _gateway_token
    if not _gateway_token and not _gateway_login():
        return False, "gateway auth failed"
    url = f"{GATEWAY_URL}{endpoint}"
    body = json.dumps(data or {}).encode() if data else None
    headers = {"Content-Type": "application/json", "Authorization": f"Bearer {_gateway_token}"}
    req = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        resp = urllib.request.urlopen(req, timeout=5)
        if resp.status == 200: return True, resp.read().decode()
        elif resp.status == 401:
            if _gateway_login():
                headers["Authorization"] = f"Bearer {_gateway_token}"
                req2 = urllib.request.Request(url, data=body, headers=headers, method=method)
                resp2 = urllib.request.urlopen(req2, timeout=5)
                return resp2.status == 200, resp2.read().decode()
        return False, f"HTTP {resp.status}"
    except urllib.error.HTTPError as e:
        return False, f"HTTP {e.code}: {e.reason}"
    except Exception as e:
        return False, str(e)

def _gateway_nav_cmd(command):
    """调用 gateway /api/nav/command，gateway 是唯一状态源"""
    ok, msg = _call_gateway("/api/nav/command", {"command": command})
    if ok: rospy.loginfo(f"[MQ] gateway nav {command} OK")
    else: rospy.logwarn(f"[MQ] gateway nav {command} failed: {msg}")
    return ok

def _ensure_motion_pubs():
    global cmd_vel_pub, sport_cmd_pub
    if cmd_vel_pub is None:
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    if sport_cmd_pub is None:
        sport_cmd_pub = rospy.Publisher("/go2/sport_cmd", RosString, queue_size=5)

def h_motor_start(body, ref):
    ok, msg = _call_gateway("/api/motion/start")
    if ok: _ack(ref, "motor_start", "accepted"); rospy.loginfo("[MQ] motor_start → gateway OK")
    else: _ack(ref, "motor_start", "rejected", msg[:200])

def h_motor_stop(body, ref):
    ok, msg = _call_gateway("/api/motion/stop")
    if ok: _ack(ref, "motor_stop", "accepted"); rospy.loginfo("[MQ] motor_stop → gateway OK")
    else: _ack(ref, "motor_stop", "rejected", msg[:200])

def h_motor_stand(body, ref):
    _ensure_motion_pubs()
    if sport_cmd_pub:
        sport_cmd_pub.publish(RosString(data="stand_up")); _ack(ref, "motor_stand", "accepted")
    else: _ack(ref, "motor_stand", "rejected", "publisher not ready")

def h_motor_sit(body, ref):
    _ensure_motion_pubs()
    if sport_cmd_pub:
        sport_cmd_pub.publish(RosString(data="sit")); _ack(ref, "motor_sit", "accepted")
    else: _ack(ref, "motor_sit", "rejected", "publisher not ready")

def h_motor_damp(body, ref):
    _ensure_motion_pubs()
    if sport_cmd_pub:
        sport_cmd_pub.publish(RosString(data="damp")); _ack(ref, "motor_damp", "accepted")
    else: _ack(ref, "motor_damp", "rejected", "publisher not ready")

def h_motor_move(body, ref):
    vx = float(body.get("vx", 0)); vy = float(body.get("vy", 0)); vyaw = float(body.get("vyaw", 0))
    _ensure_motion_pubs()
    if cmd_vel_pub:
        tw = Twist(); tw.linear.x = vx; tw.linear.y = vy; tw.angular.z = vyaw
        cmd_vel_pub.publish(tw); _ack(ref, "motor_move", "accepted")
    else: _ack(ref, "motor_move", "rejected", "publisher not ready")

HANDLERS = {"map_list": h_map_list, "switch_map": h_switch_map,
            "nav_single": h_nav_single,
            "nav_multi": h_nav_multi, "nav_pause": h_nav_pause, "nav_resume": h_nav_resume,
            "nav_cancel": h_nav_cancel, "relocalize": h_relocalize,
            "motor_start": h_motor_start, "motor_stop": h_motor_stop,
            "motor_stand": h_motor_stand, "motor_sit": h_motor_sit,
            "motor_damp": h_motor_damp, "motor_move": h_motor_move}

def process_cmd(payload_bytes):
    """处理收到的指令（AMQP/MQTT 共用）"""
    body_str = payload_bytes.decode("utf-8") if isinstance(payload_bytes, bytes) else payload_bytes
    body_str = body_str.strip().replace("\n", "").replace("\r", "").replace("\t", " ")
    try: msg = json.loads(body_str)
    except json.JSONDecodeError as e:
        rospy.logwarn(f"[MQ] bad JSON: {e} | raw={str(body_str)[:200]}")
        return
    h, p = msg.get("header", {}), msg.get("body", {})
    cmd, ref = p.get("cmd", ""), h.get("msg_id", "")
    rospy.loginfo(f"[MQ] recv cmd: {cmd}")
    f = HANDLERS.get(cmd)
    if f:
        try: f(p, ref)
        except Exception as e: _ack(ref, cmd, "rejected", str(e))
    else: _ack(ref, cmd, "rejected", f"unknown: {cmd}")

# ====== 发布者 ======
class Publisher:
    def __init__(self):
        self.tf_listener = None
        self.last_plan = None
        self._run = False
        self.sub = None
        self._pos_history = []  # [(timestamp, x, y), ...] 用于卡住检测
    def _init(self):
        self.tf_listener = tf.TransformListener()
        self.sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self._on_plan)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self._on_cmd_vel)
        self._last_cmd_vel_time = 0.0
        self._cmd_vel_active = False
        # 体素障碍物点云订阅
        self._last_voxel_grid = []
        self._last_voxel_time = 0.0
        self._voxel_sub = rospy.Subscriber(
            "/move_base/local_costmap/stvl_obstacle_layer/voxel_grid",
            PointCloud2, self._on_voxel_grid)
    def _check_stuck_by_displacement(self, stuck_window=3.0, stuck_threshold=0.3):
        """通过位移判断是否卡住：在 stuck_window 秒内位移 < stuck_threshold 米视为卡住"""
        now = time.time()
        try:
            if self.tf_listener.waitForTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0), rospy.Duration(0.2)):
                pos, _ = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                self._pos_history.append((now, pos[0], pos[1]))
        except: pass
        # 清理超过 5 秒的旧记录
        self._pos_history = [(t, x, y) for t, x, y in self._pos_history if now - t < 5.0]
        if len(self._pos_history) < 2:
            return False  # 数据不足，不判定为卡住
        # 找到 stuck_window 秒前的位置
        ref = self._pos_history[0]
        for p in self._pos_history:
            if now - p[0] <= stuck_window:
                ref = p
                break
        dx = self._pos_history[-1][1] - ref[1]
        dy = self._pos_history[-1][2] - ref[2]
        disp = (dx*dx + dy*dy) ** 0.5
        return disp < stuck_threshold
    def _on_cmd_vel(self, m):
        """追蹤最近一次非零速度指令"""
        now = time.time()
        if abs(m.linear.x) > 0.001 or abs(m.angular.z) > 0.001:
            self._last_cmd_vel_time = now
            self._cmd_vel_active = True
        else:
            # 零速度超过 0.5 秒视为停止
            if now - self._last_cmd_vel_time > 0.5:
                self._cmd_vel_active = False
    def _publish(self, topic_suffix, body):
        """线程安全发布"""
        mq_publish(topic_suffix, body)
    def _on_plan(self, m): self.last_plan = m
    def _on_voxel_grid(self, m):
        """接收 STVL 体素障碍点云，保存最新数据用于 MQTT 发布"""
        try:
            pts = []
            # 限制点数，避免 MQTT 消息过大
            MAX_VOXEL_PTS = 5000
            for i, p in enumerate(pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)):
                if i >= MAX_VOXEL_PTS: break
                pts.append({"x": float(p[0]), "y": float(p[1]), "z": float(p[2])})
            self._last_voxel_grid = pts
            self._last_voxel_time = time.time()
        except Exception as e:
            rospy.logwarn(f"[MQ] voxel_grid parse error: {e}")
    def run(self):
        self._init()
        self._run = True
        last_hb = 0
        while self._run and not rospy.is_shutdown():
            now = time.time()
            # 1. status — 优先，不依赖 TF
            try:
                ok, data = _call_gateway("/api/snapshot", method="GET")
                gw_nav = "idle"; gw_cmd = ""
                if ok:
                    try:
                        d = json.loads(data)
                        r = d.get("runtime", {})
                        gw_nav = r.get("navStatus", "idle")
                        gw_mode = r.get("navMode", "none")
                        gw_cmd = "nav_multi" if gw_mode == "multi" else ("nav_single" if gw_mode == "single" else "")
                        last_voxel = r.get("lastVoxelAt")
                        if last_voxel:
                            voxel_dt = (datetime.now(timezone.utc) - datetime.fromisoformat(last_voxel.replace("Z", "+00:00"))).total_seconds()
                            if gw_nav == "running" and voxel_dt > 3.0:
                                gw_nav = "loc_lost"
                        # 细化 running 状态：有速度指令且正在跟踪 → moving，已到达 → arrived
                        if gw_nav == "running":
                            try:
                                mb_state = mb.state()
                                if mb_state == 3:    # SUCCEEDED: 已到达目标点
                                    gw_nav = "arrived"
                                elif mb_state == 1 and self._cmd_vel_active:  # ACTIVE + 有速度指令
                                    # 有速度输出但位移不足 → 原地踱步/转圈，判定为卡住
                                    if self._check_stuck_by_displacement():
                                        gw_nav = "stuck"
                                    else:
                                        gw_nav = "moving"
                                else:
                                    gw_nav = "stuck"   # 导航激活但机器人卡住/未运动
                            except: pass
                    except: pass
                self._gw_nav = gw_nav
                self._publish("status", {"header": _hdr("nav_status"),
                    "body": {"nav_state": gw_nav, "current_cmd": gw_cmd,
                             "current_cmd_id": "", "current_map": state.current_map}})
            except: pass
            # 2. heartbeat — 不依赖 TF，每 5 秒
            if now - last_hb >= 5:
                last_hb = now
                try: self._publish("heartbeat", {"header": _hdr("heartbeat"),
                    "body": {"online":True,"nav_state": getattr(self, '_gw_nav', 'idle')}})
                except: pass
            # 3. pose — 依赖 TF，加超时保护
            try:
                if self.tf_listener.waitForTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0), rospy.Duration(0.5)):
                    pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                    roll, pitch, yaw = tf.transformations.euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])
                    self._publish("pose", {"header": _hdr("nav_pose"),
                        "body": {"frame_id": TF_MAP_FRAME, "position": {"x":pos[0],"y":pos[1],"z":pos[2]},
                                 "orientation": {"roll": roll, "pitch": pitch, "yaw": yaw},
                                 "localization_quality": "good"}})
            except: pass
            # 4. route — 依赖 TF 和 move_base
            try:
                if self.last_plan and self.last_plan.poses:
                    pts_raw = [{"x":p.pose.position.x,"y":p.pose.position.y} for p in self.last_plan.poses[:200]]
                    path_len = 0.0
                    for i in range(1, len(pts_raw)):
                        dx = pts_raw[i]["x"] - pts_raw[i-1]["x"]
                        dy = pts_raw[i]["y"] - pts_raw[i-1]["y"]
                        path_len += (dx*dx + dy*dy) ** 0.5
                    source = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                    target = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                    try:
                        if self.tf_listener.waitForTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0), rospy.Duration(0.3)):
                            pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                            _, _, yaw = tf.transformations.euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])
                            source = {"x": round(pos[0], 3), "y": round(pos[1], 3), "yaw": round(yaw, 4)}
                    except: pass
                    lg = state.last_goal
                    if lg:
                        g = lg.get("goal", {})
                        target = {"x": float(g.get("x", 0)), "y": float(g.get("y", 0)), "yaw": float(g.get("yaw", 0))}
                    self._publish("route", {"header": _hdr("nav_route"),
                        "body": {"route_id": str(uuid.uuid4()), "source": source, "target": target,
                                 "path": pts_raw, "path_length": round(path_len, 2)}})
            except: pass
            # 5. voxel_grid — 体素障碍点云，节流 2Hz
            try:
                pts = self._last_voxel_grid
                if pts and now - getattr(self, '_last_voxel_pub', 0) >= 0.5:
                    self._last_voxel_pub = now
                    self._publish("voxel_grid", {"header": _hdr("voxel_grid"),
                        "body": {"points": pts, "count": len(pts),
                                 "frame_id": "map"}})
            except: pass
            time.sleep(1.0)
    def start(self):
        t = threading.Thread(target=self.run, daemon=True); t.start()
pub = Publisher()

# ====== 消费者 ======
class Consumer:
    def __init__(self):
        self._run = False
    def run(self):
        self._run = True
        while self._run and not rospy.is_shutdown():
            try:
                if MQ_TYPE == "mqtt":
                    self._run_mqtt()
                else:
                    self._run_amqp()
            except Exception as e:
                rospy.logwarn(f"[MQ] consumer: {e}, retry 5s...")
                time.sleep(5)

    def _run_mqtt(self):
        mq_subscribe("cmd", qos=2)
        # 定义 MQTT 消息回调
        def on_cmd_msg(client, userdata, msg):
            process_cmd(msg.payload)
        _mq_client.on_message = on_cmd_msg
        rospy.loginfo("[MQ] MQTT listening nav/+/cmd")
        while self._run and not rospy.is_shutdown():
            time.sleep(0.5)

    def _run_amqp(self):
        import pika as pk
        params = pk.ConnectionParameters(host=MQ_HOST, port=MQ_PORT, virtual_host=MQ_VHOST,
            credentials=pk.PlainCredentials(MQ_USER, MQ_PASS), heartbeat=30)
        conn = pk.BlockingConnection(params); ch = conn.channel()
        ch.exchange_declare(exchange=MQ_EXCHANGE, exchange_type="topic", durable=True)
        q = f"nav.{MQ_CLIENT_ID}.cmd.q"
        ch.queue_declare(queue=q, durable=True)
        ch.queue_bind(exchange=MQ_EXCHANGE, queue=q, routing_key=CMD_TOPIC)
        def on_msg(ch, method, props, body):
            process_cmd(body)
        ch.basic_consume(queue=q, on_message_callback=on_msg, auto_ack=True)
        rospy.loginfo(f"[MQ] AMQP listening {CMD_TOPIC}")
        while self._run and not rospy.is_shutdown() and conn.is_open:
            conn.process_data_events(time_limit=1)
        try: conn.close()
        except: pass

    def start(self):
        t = threading.Thread(target=self.run, daemon=True); t.start()
consumer = Consumer()

def _read_active_map():
    """启动时从 active/current.txt 读取当前加载的地图"""
    try:
        active_txt = os.path.join(NAV_ROOT, "lite_cog/system/map/active/current.txt")
        with open(active_txt) as f:
            for line in f:
                if line.startswith("NAME="):
                    return line.strip().split("=", 1)[1]
    except: pass
    return ""

def main():
    global init_pub
    rospy.init_node("mq_adapter"); rospy.loginfo(f"[MQ] starting... (type={MQ_TYPE})")
    mq_connect()
    init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    mb.connect()
    with state.lock: state.current_map = _read_active_map()
    rospy.loginfo(f"[MQ] current map: {state.current_map or '(none)'}")
    consumer.start(); pub.start()
    rospy.spin()

if __name__ == "__main__": main()
