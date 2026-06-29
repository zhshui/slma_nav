#!/usr/bin/env python3
"""发送 MQ 消息到 mq_adapter — 支持 MQTT 和 RabbitMQ

用法：直接修改下面 MSG 的 JSON 內容，然後 python3 mq_send.py 即可。
消息格式与 mq_adapter.py 兼容：{header: {msg_type: "nav_cmd"}, body: {cmd: ..., ...}}

可用命令（mq_adapter HANDLERS）：
  导航: map_list, switch_map, nav_single, nav_multi, nav_pause, nav_resume, nav_cancel, relocalize
  运控: motor_start, motor_stop, motor_stand, motor_sit, motor_damp, motor_move

示例：
  motor_stand          → 只需 cmd
  nav_single           → 需 goal: {x, y, yaw}
  nav_multi            → 需 waypoints: [{x, y, yaw}, ...]
  motor_move           → 需 vx, vy, vyaw
  relocalize           → 需 pose: {x, y, yaw}
"""

import sys
import os
import json
import uuid

# ── MQ 配置 ──────────────────────────────────────────────────
MQ_TYPE = os.environ.get("MQ_TYPE", "mqtt")
MQ_HOST = os.environ.get("MQ_HOST", "49.233.183.203")
MQ_PORT = int(os.environ.get("MQ_PORT", "1883"))
MQ_USER = os.environ.get("MQ_USER", "server")
MQ_PASS = os.environ.get("MQ_PASS", "5bP!8aS3$kD7vF2&")
MQ_CLIENT_ID = os.environ.get("MQ_CLIENT_ID", "nav-robot-xx1")
EXCHANGE = os.environ.get("MQ_EXCHANGE", "nav.exchange")
VHOST = os.environ.get("MQ_VHOST", "/")

# mq_adapter 监听的 topic 模式:
#   MQTT:  nav/{MQ_CLIENT_ID}/cmd
#   AMQP:  nav.{MQ_CLIENT_ID}.cmd
_SEP = "/" if MQ_TYPE == "mqtt" else "."
CMD_TOPIC = f"nav{_SEP}{MQ_CLIENT_ID}{_SEP}cmd"

# ═══════════════════════════════════════════════════════════════
#  改这里 ↓ （直接寫 JSON，msg_type 必填，cmd 必填）
# ═══════════════════════════════════════════════════════════════
MSG = json.dumps({
    "header": {"msg_type": "nav_cmd"},
  "body": {
    "cmd": "nav_single",
    "map_id": "scans_4",
    "goal": { "x": 12.34, "y": -5.67, "yaw": 1.57, "frame_id": "camera_init" },
    "options": { "timeout_ms": 60000, "retry_count": 0 }
  }
})
# ═══════════════════════════════════════════════════════════════

if MQ_TYPE == "mqtt":
    import paho.mqtt.client as mqtt_lib

    SENDER_ID = f"{MQ_CLIENT_ID}-send-{uuid.uuid4().hex[:8]}"
    client = mqtt_lib.Client(client_id=SENDER_ID, protocol=mqtt_lib.MQTTv311)
    client.username_pw_set(MQ_USER, MQ_PASS)
    client.connect(MQ_HOST, MQ_PORT, keepalive=30)
    client.loop_start()
    info = client.publish(CMD_TOPIC, MSG, qos=1)
    info.wait_for_publish(timeout=3)
    print(f"📤 MQTT {CMD_TOPIC} ← {MSG}")
    client.loop_stop()
    client.disconnect()
else:
    import pika

    params = pika.ConnectionParameters(
        host=MQ_HOST, port=MQ_PORT, virtual_host=VHOST,
        credentials=pika.PlainCredentials(MQ_USER, MQ_PASS), heartbeat=30,
    )
    conn = pika.BlockingConnection(params)
    ch = conn.channel()
    ch.exchange_declare(exchange=EXCHANGE, exchange_type="topic", durable=True)
    ch.basic_publish(
        exchange=EXCHANGE,
        routing_key=CMD_TOPIC,
        body=MSG,
    )
    print(f"📤 AMQP {CMD_TOPIC} ← {MSG}")
    conn.close()
