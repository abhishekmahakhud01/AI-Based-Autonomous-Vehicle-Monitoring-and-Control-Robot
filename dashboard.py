import threading
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from flask import Flask, jsonify, render_template_string

state = {
    "front": None, "left": None, "right": None, "x": 0.0, "y": 0.0,
    "linear": 0.0, "angular": 0.0, "scan_ok": False,
    "odom_ok": False, "distance": 0.0, "mode": "Waiting for sensor data"
}

class Monitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self.last_xy = None
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

    def scan_cb(self, msg):
        # Determine sectors from actual scan angles, not array positions.
        front, left, right = [], [], []
        readings = []

        for i, raw in enumerate(msg.ranges):
            if not math.isfinite(raw):
                continue
            if not (msg.range_min < raw < msg.range_max):
                continue

            angle = math.degrees(
                msg.angle_min + i * msg.angle_increment
            ) % 360.0
            readings.append((angle, raw))

            if angle <= 65.0 or angle >= 295.0:
                front.append(raw)
            elif angle < 180.0:
                left.append(raw)
            else:
                right.append(raw)

        if readings:
            angle, distance = min(readings, key=lambda item: item[1])
            state["nearest_scan"] = round(distance, 2)
            state["nearest_angle_deg"] = round(angle, 1)
        else:
            state["nearest_scan"] = None
            state["nearest_angle_deg"] = None

        state["front"] = round(min(front), 2) if front else None
        state["left"] = round(min(left), 2) if left else None
        state["right"] = round(min(right), 2) if right else None
        state["scan_ok"] = True

        if front:
            state["mode"] = (
                "Obstacle nearby" if min(front) < 0.80
                else "Front path clear"
            )
        else:
            state["mode"] = "No valid return in front sector"

    def odom_cb(self, msg):
        if self.last_xy is not None:
            step = math.hypot(msg.pose.pose.position.x - self.last_xy[0],
                              msg.pose.pose.position.y - self.last_xy[1])
            if step < 0.5:
                state["distance"] = round(state["distance"] + step, 2)
        self.last_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        state["x"] = round(msg.pose.pose.position.x, 2)
        state["y"] = round(msg.pose.pose.position.y, 2)
        state["odom_ok"] = True

    def cmd_cb(self, msg):
        state["linear"] = round(msg.linear.x, 2)
        state["angular"] = round(msg.angular.z, 2)

app = Flask(__name__)
PAGE = """
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Autonomous Robot Monitor</title>
<style>
body{font-family:Arial;background:#101827;color:#f4f7fb;margin:0;padding:24px}
h1{font-size:25px}.sub{color:#aab8ca}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:14px}
.card{background:#1c293b;border:1px solid #34445b;border-radius:14px;padding:18px}
.label{color:#aab8ca;font-size:14px}.value{font-size:27px;font-weight:bold;margin-top:12px}
#status{padding:14px;border-radius:10px;background:#26384e;margin:18px 0}
small{color:#aab8ca}footer{margin-top:22px;color:#aab8ca}
</style></head><body>
<h1>Autonomous Vehicle Monitoring</h1>
<div class="sub">ROS 2 Humble · TurtleBot3 Burger · Live telemetry</div>
<div id="status">Connecting to robot sensors…</div>
<div class="grid">
<div class="card"><div class="label">Front obstacle distance</div><div class="value" id="front">—</div></div>
<div class="card"><div class="label">Left obstacle distance</div><div class="value" id="left">—</div></div>
<div class="card"><div class="label">Right obstacle distance</div><div class="value" id="right">—</div></div>
<div class="card"><div class="label">Position X</div><div class="value" id="x">—</div></div>
<div class="card"><div class="label">Position Y</div><div class="value" id="y">—</div></div>
<div class="card"><div class="label">Distance travelled</div><div class="value" id="distance">—</div></div>
<div class="card"><div class="label">Linear velocity</div><div class="value" id="linear">—</div></div>
<div class="card"><div class="label">Angular velocity</div><div class="value" id="angular">—</div></div>
</div>
<footer>Refreshes automatically every second. An unavailable LiDAR return is shown as “No return”, not as a measured distance.</footer>
<script>
function val(v){return v===null?'No return':v+' m'}
async function refresh(){try{
let r=await fetch('/api/status');let d=await r.json();
document.getElementById('front').textContent=val(d.front);
document.getElementById('left').textContent=val(d.left);
document.getElementById('right').textContent=val(d.right);
document.getElementById('distance').textContent=d.distance+' m';
document.getElementById('x').textContent=d.x+' m';
document.getElementById('y').textContent=d.y+' m';
document.getElementById('linear').textContent=d.linear+' m/s';
document.getElementById('angular').textContent=d.angular+' rad/s';
document.getElementById('status').textContent='LiDAR: '+(d.scan_ok?'Connected':'Waiting')+
' · Odometry: '+(d.odom_ok?'Connected':'Waiting')+' · '+d.mode;
}catch(e){document.getElementById('status').textContent='Waiting for dashboard data…'}}
refresh();setInterval(refresh,1000);
</script></body></html>
"""

@app.route('/')
def home():
    return render_template_string(PAGE)

@app.route('/api/status')
def status_api():
    return jsonify(state)

def ros_thread():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    threading.Thread(target=ros_thread, daemon=True).start()
    app.run(host='127.0.0.1', port=5000, debug=False)
