#vir_map_path.py에서 PID 제거

import numpy as np
import cv2
import heapq
import matplotlib.pyplot as plt
import os

# =========================
# 1. 이미지 전처리
# =========================
image_path = "/home/yb/project2/proj2/pgm_yaml_create/test_map.png"
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
if image is None:
    print(f"파일 {image_path}를 찾을 수 없습니다.")
    exit()

# 이진화: 검정(벽)이 255, 이동 가능 영역이 0으로 뒤집힘
_, binary = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY_INV)
h, w = binary.shape

# =========================
# 2. 내부 이동 가능한 영역 찾기
# =========================
start_x, start_y = 539, 143
if binary[start_y, start_x] != 0:
    print("❌ 시작 지점이 이동 불가능한 위치입니다. 다시 설정하세요.")
    exit()

flood_filled = binary.copy()
mask = np.zeros((h + 2, w + 2), np.uint8)
cv2.floodFill(flood_filled, mask, (start_x, start_y), 255)
interior = cv2.bitwise_and(flood_filled, cv2.bitwise_not(binary))

pixel_to_meter = 0.01228  
margin_m = 0.3        # 벽과 최소 0.3m 이상 떨어진 지점만 웨이포인트로 선택
margin_pixels = margin_m / pixel_to_meter  
dist_transform = cv2.distanceTransform(interior, cv2.DIST_L2, 5)

# =========================
# 3. 그리드 기반 웨이포인트 생성
# =========================
grid_size = 50       # grid 간격 (예: 60픽셀)
grid_nodes = []
node_set = set()

for yy in range(grid_size // 2, h, grid_size):
    for xx in range(grid_size // 2, w, grid_size):
        if interior[yy, xx] == 255 and dist_transform[yy, xx] >= margin_pixels:
            grid_nodes.append((xx, yy))
            node_set.add((xx, yy))

if not grid_nodes:
    print("❌ 웨이포인트가 없습니다. 이미지 처리를 확인하세요.")
    exit()

# =========================
# 4. 시작 지점 및 로봇 초기화
# =========================
visited_nodes = set()
center = (w // 2, h // 2)
start = min(grid_nodes, key=lambda node: np.linalg.norm(np.array(node) - np.array(center)))
robot_position = start
visited_nodes.add(start)
robot = np.array([start[0], start[1], 0.0])
trajectory = [robot.copy()]

# =========================
# 5. A* 경로 탐색 함수
# =========================
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def grid_a_star_search(start, goal, node_set, step):
    directions = [
        (step, 0), (-step, 0), (0, step), (0, -step),
        (step, step), (step, -step), (-step, step), (-step, -step)
    ]
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            break
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in node_set:
                move_cost = np.hypot(dx, dy) / step
                new_cost = cost_so_far[current] + move_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current
                    
    if goal not in came_from:
        return []
    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = came_from[cur]
    path.append(start)
    path.reverse()
    return path

# =========================
# 6. A* 경로 따라가기 함수
# =========================
def move_along_astar_path(robot, path):
    traj = []
    for point in path:
        robot[:2] = np.array(point)
        traj.append(robot.copy())
    return traj

# =========================
# 7. 경로 스무딩 함수
# =========================
def smooth_path(path, window_size=3):
    if len(path) < window_size:
        return path
    smoothed = []
    for i in range(len(path)):
        start_idx = max(0, i - window_size)
        end_idx = min(len(path), i + window_size + 1)
        pts = np.array(path[start_idx:end_idx])
        avg = np.mean(pts, axis=0)
        smoothed.append((avg[0], avg[1], avg[2]))
    return smoothed

# =========================
# 8. 전체 시뮬레이션 루프
# =========================
simulation_steps = 0
max_iterations = 1000

while len(visited_nodes) < len(grid_nodes) and simulation_steps < max_iterations:
    unvisited = [node for node in grid_nodes if node not in visited_nodes]
    if not unvisited:
        break
    next_target = min(unvisited, key=lambda n: np.linalg.norm(np.array(robot_position) - np.array(n)))
    
    path = grid_a_star_search(robot_position, next_target, node_set, grid_size)
    if not path:
        visited_nodes.add(next_target)
        robot_position = next_target
        simulation_steps += 1
        continue
    
    astar_traj = move_along_astar_path(robot.copy(), path)
    smoothed_traj = smooth_path(astar_traj, window_size=3)
    robot = np.array(smoothed_traj[-1])
    trajectory.extend(smoothed_traj)
    robot_position = next_target
    visited_nodes.add(next_target)
    
    # 중간 시각화
    result = cv2.cvtColor(interior, cv2.COLOR_GRAY2BGR)
    for node in grid_nodes:
        color = (255, 0, 0) if node in visited_nodes else (200, 200, 200)
        cv2.circle(result, node, 3, color, -1)
    for i in range(len(path) - 1):
        cv2.line(result, path[i], path[i+1], (0, 255, 0), 2)
    cv2.circle(result, robot_position, 5, (0, 0, 255), -1)
    
    plt.figure(figsize=(6,6))
    plt.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
    plt.title(f"Iteration {simulation_steps+1} : 경로 이동 완료")
    plt.axis("off")
    plt.pause(0.5)
    plt.close()
    
    simulation_steps += 1

# =========================
# 9. 최종 결과 시각화 및 저장
# =========================
final_result = cv2.cvtColor(interior, cv2.COLOR_GRAY2BGR)
for node in grid_nodes:
    cv2.circle(final_result, node, 3, (255, 0, 0), -1)
if len(trajectory) > 1:
    traj_points = np.array([[int(pt[0]), int(pt[1])] for pt in trajectory])
    cv2.polylines(final_result, [traj_points], False, (0, 255, 255), thickness=2)
cv2.circle(final_result, (int(robot[0]), int(robot[1])), 5, (0, 0, 255), -1)
    
plt.figure(figsize=(6,6))
plt.imshow(cv2.cvtColor(final_result, cv2.COLOR_BGR2RGB))
plt.title("최종 경로 시각화")
plt.axis("off")
plt.show()

save_path = "/home/yb/project2/proj2/pgm_yaml_create/"
traj_filename = os.path.join(save_path, "trajectory.txt")
with open(traj_filename, "w") as f:
    for idx, pt in enumerate(trajectory):
        f.write(f"{idx}: x={pt[0]:.2f}, y={pt[1]:.2f}, theta={pt[2]:.2f}\n")
cv2.imwrite(os.path.join(save_path, "final_astar_mapping.png"), final_result)
print("✅ 최종 A-star 기반 경로 저장 완료")
