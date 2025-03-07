import os
import threading
import yaml
import cv2
import math
import sys
from flask import Blueprint, jsonify, send_file, render_template_string, request
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus

nav_bp = Blueprint('nav', __name__, template_folder='templates')

# 전역 변수들
current_pose_data = None
robot_path_data = []
current_target_data = None
current_user_gps = None

YAML_PATH = "/home/yb/project2/proj2/click_move/test.yaml"
map_data = {}
map_image_path = ""

def load_map():
    global map_data, map_image_path
    if not os.path.exists(YAML_PATH):
        print("❌ YAML 파일을 찾을 수 없습니다.")
        return
    with open(YAML_PATH, "r") as file:
        map_config = yaml.safe_load(file)
    pgm_path = os.path.join(os.path.dirname(YAML_PATH), map_config["image"])
    if not os.path.exists(pgm_path):
        print(f"❌ PGM 파일을 찾을 수 없습니다: {pgm_path}")
        return
    print(f"📂 PGM 파일 로드: {pgm_path}")
    pgm_image = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    height, width = pgm_image.shape[:2]
    # PNG 파일 경로 (절대경로)
    map_image_path = "/home/yb/project2/proj2/click_move/static/map.png"
    cv2.imwrite(map_image_path, pgm_image)
    print(f"✅ PNG 맵 생성 완료: {map_image_path}")
    map_data = {
        "resolution": map_config["resolution"],
        "origin": map_config["origin"],
        "image": "static/map.png",
        "width": width,
        "height": height
    }
    globals()['map_data'] = map_data

@nav_bp.route('/')
def nav_index():
    html_template = '''
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <title>내비게이션</title>
        <style>
            #mapContainer { position: relative; display: inline-block; }
            #mapImage { display: block; }
            #overlay { position: absolute; top: 0; left: 0; cursor: crosshair; }
        </style>
    </head>
    <body>
        <h1>실시간 내비게이션</h1>
        <div id="mapContainer">
            <img id="mapImage" src="/nav/get_map_image" alt="Map">
            <canvas id="overlay"></canvas>
        </div>
        <p id="coords">현재 로봇 위치: </p>
        <p id="userCoords">사용자 위치: </p>
        <script>
            const resolution = {{ map_data.resolution }};
            const origin = {{ map_data.origin }};
            const mapWidth = {{ map_data.width }};
            const mapHeight = {{ map_data.height }};
            const overlay = document.getElementById('overlay');
            overlay.width = mapWidth;
            overlay.height = mapHeight;
            const ctx = overlay.getContext('2d');

            function mapToPixel(mapX, mapY) {
                let pixelX = (mapX - origin[0]) / resolution;
                let pixelY = mapHeight - ((mapY - origin[1]) / resolution);
                return { x: pixelX, y: pixelY };
            }

            // 지도 클릭 시 목표 좌표를 서버로 전송
            overlay.addEventListener('click', function(e) {
                const rect = overlay.getBoundingClientRect();
                const pixelX = Math.floor(e.clientX - rect.left);
                const pixelY = Math.floor(e.clientY - rect.top);
                const mapX = origin[0] + (pixelX * resolution);
                const mapY = origin[1] + ((mapHeight - pixelY) * resolution);
                fetch('/nav/set_target', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ x: mapX, y: mapY })
                })
                .then(response => response.json())
                .then(data => { alert("목표 전송 결과: " + JSON.stringify(data)); })
                .catch(error => { alert("목표 전송 오류: " + error); });
            });

            // 주기적으로 로봇 위치, 경로, 목표를 갱신하여 그리기
            async function updateOverlay() {
                try {
                    const poseResponse = await fetch('/nav/current_pose');
                    const poseData = await poseResponse.json();
                    const pathResponse = await fetch('/nav/robot_path');
                    const pathData = await pathResponse.json();
                    const targetResponse = await fetch('/nav/current_target');
                    const targetData = await targetResponse.json();

                    ctx.clearRect(0, 0, mapWidth, mapHeight);

                    if (pathData && pathData.length > 0) {
                        ctx.beginPath();
                        pathData.forEach((pt, index) => {
                            const pixel = mapToPixel(pt.x, pt.y);
                            if (index === 0) {
                                ctx.moveTo(pixel.x, pixel.y);
                            } else {
                                ctx.lineTo(pixel.x, pixel.y);
                            }
                        });
                        ctx.strokeStyle = 'red';
                        ctx.lineWidth = 2;
                        ctx.stroke();
                    }

                    if (poseData && poseData.x !== undefined) {
                        const pixel = mapToPixel(poseData.x, poseData.y);
                        ctx.beginPath();
                        ctx.arc(pixel.x, pixel.y, 5, 0, 2 * Math.PI);
                        ctx.fillStyle = 'blue';
                        ctx.fill();
                        document.getElementById('coords').textContent =
                            '현재 로봇 위치: x=' + poseData.x.toFixed(2) + ', y=' + poseData.y.toFixed(2);
                    }

                    if (targetData && targetData.x !== undefined) {
                        const pixel = mapToPixel(targetData.x, targetData.y);
                        ctx.beginPath();
                        ctx.arc(pixel.x, pixel.y, 5, 0, 2 * Math.PI);
                        ctx.fillStyle = 'black';
                        ctx.fill();
                    }
                } catch (err) {
                    console.error(err);
                }
            }
            setInterval(updateOverlay, 1000);
        </script>
    </body>
    </html>
    '''
    return render_template_string(html_template, map_data=map_data)

@nav_bp.route('/get_map_image', methods=['GET'])
def get_map_image():
    return send_file(map_image_path, mimetype='image/png')

@nav_bp.route('/set_target', methods=['POST'])
def set_target():
    data = request.get_json()
    if not data or 'x' not in data or 'y' not in data:
        return jsonify({'error': 'Invalid data'}), 400
    x = data['x']
    y = data['y']
    global current_target_data
    current_target_data = {'x': x, 'y': y}
    if nav2_client is not None:
        nav2_client.send_goal(x, y)
        return jsonify({'status': 'Goal sent', 'x': x, 'y': y})
    else:
        return jsonify({'error': 'ROS 노드가 준비되지 않았습니다.'}), 500

@nav_bp.route('/current_pose', methods=['GET'])
def current_pose():
    global current_pose_data
    if current_pose_data is None:
        return jsonify({})
    return jsonify(current_pose_data)

@nav_bp.route('/robot_path', methods=['GET'])
def robot_path():
    global robot_path_data
    return jsonify(robot_path_data)

@nav_bp.route('/current_target', methods=['GET'])
def current_target():
    global current_target_data
    if current_target_data is None:
         return jsonify({})
    return jsonify(current_target_data)

@nav_bp.route('/set_user_gps', methods=['POST'])
def set_user_gps():
    data = request.get_json()
    if not data or 'latitude' not in data or 'longitude' not in data:
        return jsonify({'error': 'Invalid data'}), 400
    global current_user_gps
    current_user_gps = {'latitude': data['latitude'], 'longitude': data['longitude']}
    return jsonify({'status': 'User GPS set', 'latitude': data['latitude'], 'longitude': data['longitude']})

@nav_bp.route('/get_user_gps', methods=['GET'])
def get_user_gps():
    global current_user_gps
    if current_user_gps is None:
         return jsonify({})
    return jsonify(current_user_gps)

# ---------------------------
# ROS 관련 기능
# ---------------------------
def start_ros():
    """ROS 관련 스레드를 시작하는 함수"""
    def ros_spin():
        global nav2_client
        rclpy.init()
        nav2_client = Nav2Client()
        PoseSubscriber(nav2_client)
        rclpy.spin(nav2_client)
        nav2_client.destroy_node()
        rclpy.shutdown()
    threading.Thread(target=ros_spin, daemon=True).start()

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    def send_goal(self, x, y, frame_id='map'):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'목표 좌표 전송 중: x={x}, y={y}')
        timeout_sec = 10.0
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("❌ Navigation2 서버가 실행되지 않았습니다. Nav2를 먼저 실행하세요.")
            return
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표가 거부되었습니다.')
            return
        self.get_logger().info('목표가 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'네비게이션 결과: {result}')

class PoseSubscriber:
    def __init__(self, node: Node):
        self.node = node
        self.subscription = node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        global current_pose_data, robot_path_data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        siny_cosp = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z)
        cosy_cosp = 1.0 - 2.0 * (msg.pose.pose.orientation.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        current_pose_data = {'x': x, 'y': y, 'yaw': yaw}
        if not robot_path_data or (abs(robot_path_data[-1]['x'] - x) > 0.05 or abs(robot_path_data[-1]['y'] - y) > 0.05):
            robot_path_data.append({'x': x, 'y': y})

def init_nav():
    load_map()
    start_ros()
