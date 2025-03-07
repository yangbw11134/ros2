import os
import cv2
import yaml
import numpy as np
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from flask import Flask, jsonify, send_file, render_template_string
import time  # 추가

# Flask 애플리케이션 생성
app = Flask(__name__)

# 파일 경로 설정
BASE_PATH = '/home/yb/project2/proj2/pgm_yaml_create/'
YAML_PATH = os.path.join(BASE_PATH, 'test_map.yaml')
PGM_PATH = os.path.join(BASE_PATH, 'test_map.pgm')
TRAJ_PATH = os.path.join(BASE_PATH, 'trajectory.txt')
MAPPED_IMAGE_PATH = os.path.join(BASE_PATH, 'mapped_trajectory.png')

# 전역 변수 설정
map_data = {}
current_pose_data = {}
robot_path_data = []  
current_target_data = {}
remaining_waypoints = []  # 🔥 남은 웨이포인트 저장

# YAML 로드
with open(YAML_PATH, 'r') as f:
    map_config = yaml.safe_load(f)
resolution = map_config["resolution"]
origin = map_config["origin"]

# trajectory 읽기 및 스케일링
trajectory = []
with open(TRAJ_PATH, 'r') as f:
    for line in f:
        if ':' not in line:
            continue
        parts = line.split(":")[1].split(",")
        x = float(parts[0].split('=')[1])
        y = float(parts[1].split("=")[1])
        theta = float(parts[2].split("=")[1])
        trajectory.append((x, y, theta))

# 스케일링 설정
base_point = (325.0, 325.0)
new_start = (-0.21, -0.54)
scale_factor = 0.1

transformed_traj = []
for (x, y, theta) in trajectory:
    new_x = new_start[0] + (x - base_point[0]) * scale_factor
    new_y = new_start[1] + (y - base_point[1]) * scale_factor
    transformed_traj.append((new_x, new_y, theta))

# 웨이포인트 초기 설정
remaining_waypoints = transformed_traj.copy()  # 🔥 남은 웨이포인트 저장

# 픽셀 좌표 변환 함수
def map_to_pixel(map_x, map_y):
    pixel_x = int((map_x - origin[0]) / resolution)
    pixel_y = int(height - (map_y - origin[1]) / resolution)
    return pixel_x, pixel_y

# 맵 이미지 초기 로드
map_img = cv2.imread(PGM_PATH, cv2.IMREAD_UNCHANGED)
height, width = map_img.shape
map_color = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

# 초기 웨이포인트 표시 (작은 검은 점)
def draw_initial_waypoints():
    global map_color, MAPPED_IMAGE_PATH
    for wx, wy, _ in remaining_waypoints:
        px, py = map_to_pixel(wx, wy)
        cv2.circle(map_color, (px, py), 3, (0, 0, 0), -1)  # 검은색 점
    cv2.imwrite(MAPPED_IMAGE_PATH, map_color)

draw_initial_waypoints()  # 시작 전 초기 웨이포인트 표시

# ROS2 Navigation 액션 클라이언트
class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, on_goal_reached):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, on_goal_reached))

    def goal_response_callback(self, future, on_goal_reached):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().info('❌ 목표가 거부되었습니다.')
            on_goal_reached(False)
            return
        self.get_logger().info('✅ 목표가 수락되었습니다.')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_callback(f, on_goal_reached))

    def result_callback(self, future, on_goal_reached):
        result = future.result().result
        self.get_logger().info(f'🏁 목표 도달 결과: {result}')
        on_goal_reached(True)

# ROS2 위치 수신 노드
class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        global current_pose_data, robot_path_data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_pose_data = {'x': x, 'y': y}
        robot_path_data.append(current_pose_data)

# 경로 실행 함수
def execute_trajectory():
    rclpy.init()
    global nav2_client
    nav2_client = Nav2Client()
    PoseSubscriber()

    for (x, y, _) in transformed_traj:
        goal_reached = threading.Event()

        def on_goal_reached(success):
            if success:
                goal_reached.set()
        
        nav2_client.send_goal(x, y, on_goal_reached)
        goal_reached.wait()

        # 로봇 이동 경로 동적 업데이트 (경로 업데이트 시 기존 웨이포인트는 그대로 유지하거나 덮어쓰기 가능)
        update_robot_path(x, y)

        # 웨이포인트 제거 (목표로 사용한 좌표는 제거)
        remaining_waypoints.pop(0)

        # 로봇이 목표에 도달한 후 대기
        time.sleep(1.5)

        # ROS2 상태 업데이트
        rclpy.spin_once(nav2_client, timeout_sec=0.5)

# 로봇의 이동 경로 업데이트 (현재 위치부터)
def update_robot_path(x, y):
    global map_color, MAPPED_IMAGE_PATH

    pixel_x, pixel_y = map_to_pixel(x, y)
    if robot_path_data:
        prev_x, prev_y = map_to_pixel(robot_path_data[-1]['x'], robot_path_data[-1]['y'])
        cv2.line(map_color, (prev_x, prev_y), (pixel_x, pixel_y), (0, 0, 255), 2)
    
    # 웨이포인트 표시 (여전히 남은 웨이포인트들을 작은 검은 점으로 표시)
    for wx, wy, _ in remaining_waypoints:
        px, py = map_to_pixel(wx, wy)
        cv2.circle(map_color, (px, py), 3, (0, 0, 0), -1)  # 검은색 점

    # 현재 로봇 위치 표시 (파란색)
    cv2.circle(map_color, (pixel_x, pixel_y), 3, (255, 0, 0), -1)

    cv2.imwrite(MAPPED_IMAGE_PATH, map_color)

# Flask 웹 인터페이스
@app.route('/')
def index():
    return render_template_string('''
    <h1>실시간 로봇 이동 경로 및 웨이포인트</h1>
    <img src="/get_map_image" style="width:800px;">
    ''')

@app.route('/get_map_image')
def get_map_image():
    resp = send_file(MAPPED_IMAGE_PATH, mimetype='image/png')
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return resp
@app.route('/current_pose')
def get_pose():
    return jsonify(current_pose_data)

@app.route('/waypoints')
def get_waypoints():
    return jsonify(remaining_waypoints)

if __name__ == '__main__':
    ros_thread = threading.Thread(target=execute_trajectory, daemon=True)
    ros_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=True)
