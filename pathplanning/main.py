import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import math

# --------------------------
# 1. 地图与机器人参数配置
# --------------------------
MAP_SIZE = 100  # 地图尺寸（100x100）
ROBOT_RADIUS = 2.0  # 机器人半径（碰撞检测）
MAX_V = 3.0  # 最大线速度
MAX_W = 1.5  # 最大角速度
DT = 0.1  # 时间步长（0.1秒）
PREDICT_TIME = 2.0  # DWA预测时间（未来2秒轨迹）
LOOK_AHEAD = 5  # 局部目标点超前步数

# 生成随机静态障碍物（多边形）
def generate_static_obstacles(count=8):
    obstacles = []
    for _ in range(count):
        center = (np.random.uniform(10, MAP_SIZE-10), np.random.uniform(10, MAP_SIZE-10))
        size = np.random.uniform(5, 15)
        edges = np.random.randint(4, 8)  # 4-8边形
        angles = np.sort(np.random.uniform(0, 2*math.pi, edges))
        polygon = [(center[0] + size*math.cos(a), center[1] + size*math.sin(a)) for a in angles]
        obstacles.append(polygon)
    return obstacles

# 生成动态障碍物（随机移动）
def generate_dynamic_obstacles(count=3):
    dynamics = []
    for _ in range(count):
        start_pos = (np.random.uniform(10, MAP_SIZE-10), np.random.uniform(10, MAP_SIZE-10))
        speed = np.random.uniform(0.5, 1.5)
        direction = np.random.uniform(0, 2*math.pi)
        radius = np.random.uniform(1.5, 3.0)
        dynamics.append({
            'pos': start_pos,
            'speed': speed,
            'direction': direction,
            'radius': radius
        })
    return dynamics

# 初始化环境
static_obstacles = generate_static_obstacles()
dynamic_obstacles = generate_dynamic_obstacles()
start = (5, 5)  # 起点
goal = (90, 90)  # 终点
robot_state = [start[0], start[1], 0.0, 0.0, 0.0]  # (x, y, 航向角, v, w)


# --------------------------
# 2. 全局路径规划（A*算法）
# --------------------------
def grid_from_polygons(polygons, size=MAP_SIZE):
    """将多边形障碍物转换为网格地图（0=自由，1=障碍）"""
    grid = np.zeros((size, size))
    for poly in polygons:
        # 扫描线填充多边形（简化版）
        for x in range(size):
            for y in range(size):
                if point_in_polygon((x+0.5, y+0.5), poly):
                    grid[x, y] = 1
    return grid

def point_in_polygon(point, polygon):
    """判断点是否在多边形内（射线法）"""
    x, y = point
    n = len(polygon)
    inside = False
    for i in range(n):
        j = (i + 1) % n
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)):
            x_intersect = (y - yi) * (xj - xi) / (yj - yi) + xi
            if x < x_intersect:
                inside = not inside
    return inside

def a_star(grid, start, goal):
    """A*算法实现（网格坐标）"""
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    open_list = [(0, start[0], start[1])]
    came_from = {}
    g_score = { (start[0], start[1]): 0 }
    f_score = { (start[0], start[1]): heuristic(start, goal) }

    while open_list:
        open_list.sort()
        _, x, y = open_list.pop(0)
        current = (x, y)
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx, ny] == 0:
                neighbor = (nx, ny)
                tentative_g = g_score[current] + math.hypot(dx, dy)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    open_list.append((f_score[neighbor], nx, ny))
    return None  # 无路径

def heuristic(a, b):
    """启发函数（欧氏距离）"""
    return math.hypot(a[0]-b[0], a[1]-b[1])

def reconstruct_path(came_from, current):
    """回溯路径"""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return [(p[0]+0.5, p[1]+0.5) for p in reversed(path)]  # 转换为连续坐标

# 生成全局路径
grid_map = grid_from_polygons(static_obstacles)
global_path = a_star(grid_map, start, goal)
if not global_path:
    raise ValueError("无可行全局路径，请重新生成地图")


# --------------------------
# 3. 局部路径规划（DWA算法）
# --------------------------
def get_local_goal(robot_pos, global_path):
    """从全局路径取局部目标点"""
    distances = [math.hypot(x - robot_pos[0], y - robot_pos[1]) for (x, y) in global_path]
    closest_idx = np.argmin(distances)
    local_idx = min(closest_idx + LOOK_AHEAD, len(global_path)-1)
    return global_path[local_idx]

def predict_trajectory(robot_state, v, w):
    """预测未来轨迹"""
    x, y, yaw, _, _ = robot_state
    traj = []
    for _ in range(int(PREDICT_TIME / DT)):
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += w * DT
        traj.append((x, y))
    return traj

def obstacle_cost(traj, static_obs, dynamic_obs):
    """障碍物代价（距离越近代价越高）"""
    min_dist = float('inf')
    # 静态障碍物
    for (x, y) in traj:
        for poly in static_obs:
            dist = point_to_polygon_dist((x, y), poly) - ROBOT_RADIUS
            if dist < min_dist:
                min_dist = dist
        # 动态障碍物
        for dyn in dynamic_obs:
            dx = x - dyn['pos'][0]
            dy = y - dyn['pos'][1]
            dist = math.hypot(dx, dy) - (ROBOT_RADIUS + dyn['radius'])
            if dist < min_dist:
                min_dist = dist
    return 1.0 / (min_dist + 1e-6) if min_dist > 0 else 1e6  # 碰撞时代价无穷大

def point_to_polygon_dist(point, polygon):
    """点到多边形的最短距离"""
    min_dist = float('inf')
    x, y = point
    n = len(polygon)
    for i in range(n):
        a = polygon[i]
        b = polygon[(i+1)%n]
        # 点到线段的距离
        dist = point_to_segment_dist((x,y), a, b)
        if dist < min_dist:
            min_dist = dist
    return min_dist

def point_to_segment_dist(p, a, b):
    """点到线段的距离"""
    px, py = p
    ax, ay = a
    bx, by = b
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    dot = apx * abx + apy * aby
    if dot <= 0:
        return math.hypot(apx, apy)
    ab_len_sq = abx**2 + aby**2
    if dot >= ab_len_sq:
        return math.hypot(px - bx, py - by)
    t = dot / ab_len_sq
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)

def dwa_planner(robot_state, local_goal, static_obs, dynamic_obs):
    """DWA算法：生成最优局部轨迹"""
    best_score = -float('inf')
    best_traj = []
    v_samples = np.linspace(0, MAX_V, 10)  # 线速度采样
    w_samples = np.linspace(-MAX_W, MAX_W, 20)  # 角速度采样

    for v in v_samples:
        for w in w_samples:
            traj = predict_trajectory(robot_state, v, w)
            # 评估轨迹
            obs_cost = obstacle_cost(traj, static_obs, dynamic_obs)
            goal_dist = math.hypot(traj[-1][0]-local_goal[0], traj[-1][1]-local_goal[1])
            goal_cost = goal_dist
            vel_cost = v  # 鼓励高速
            # 总得分（最大化）
            score = -0.5*goal_cost - 2.0*obs_cost + 0.3*vel_cost
            if score > best_score:
                best_score = score
                best_traj = traj
    return best_traj


# --------------------------
# 4. 仿真可视化与主循环
# --------------------------
# 初始化画布
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(0, MAP_SIZE)
ax.set_ylim(0, MAP_SIZE)
ax.set_title("全局+局部路径规划仿真 (A* + DWA)")

# 绘制静态障碍物
for poly in static_obstacles:
    x = [p[0] for p in poly]
    y = [p[1] for p in poly]
    ax.fill(x, y, 'gray')

# 绘制全局路径
global_x = [p[0] for p in global_path]
global_y = [p[1] for p in global_path]
ax.plot(global_x, global_y, 'b--', linewidth=2, label='全局路径')

# 动态绘制元素
robot_dot, = ax.plot([], [], 'ro', markersize=10, label='机器人')
local_traj, = ax.plot([], [], 'g-', linewidth=1.5, label='局部轨迹')
dynamic_circles = [ax.add_patch(Circle((0,0), 0, color='red', alpha=0.5)) for _ in dynamic_obstacles]
local_goal_dot, = ax.plot([], [], 'ko', markersize=8, label='局部目标')

ax.legend()

def update(frame):
    # 更新动态障碍物位置
    for dyn, circle in zip(dynamic_obstacles, dynamic_circles):
        # 随机转向
        if np.random.random() < 0.05:
            dyn['direction'] += np.random.uniform(-math.pi/4, math.pi/4)
        # 移动
        dx = dyn['speed'] * math.cos(dyn['direction']) * DT
        dy = dyn['speed'] * math.sin(dyn['direction']) * DT
        new_x = dyn['pos'][0] + dx
        new_y = dyn['pos'][1] + dy
        # 边界反弹
        if new_x < 5 or new_x > MAP_SIZE-5:
            dyn['direction'] = math.pi - dyn['direction']
            new_x = dyn['pos'][0] - dx
        if new_y < 5 or new_y > MAP_SIZE-5:
            dyn['direction'] = -dyn['direction']
            new_y = dyn['pos'][1] - dy
        dyn['pos'] = (new_x, new_y)
        circle.set_center((new_x, new_y))
        circle.set_radius(dyn['radius'])

    # 机器人规划与移动
    global robot_state
    robot_pos = (robot_state[0], robot_state[1])
    # 获取局部目标
    local_goal = get_local_goal(robot_pos, global_path)
    # DWA规划局部轨迹
    best_traj = dwa_planner(robot_state, local_goal, static_obstacles, dynamic_obstacles)
    # 更新机器人状态（沿最优轨迹移动）
    if best_traj and len(best_traj) > 1:
        dx = best_traj[1][0] - best_traj[0][0]
        dy = best_traj[1][1] - best_traj[0][1]
        robot_state[0] = best_traj[1][0]
        robot_state[1] = best_traj[1][1]
        robot_state[2] = math.atan2(dy, dx)  # 更新航向角
        robot_state[3] = math.hypot(dx, dy) / DT  # 线速度
        robot_state[4] = (robot_state[2] - math.atan2(best_traj[0][1]-robot_pos[1], best_traj[0][0]-robot_pos[0])) / DT  # 角速度

    # 更新绘图
    robot_dot.set_data(robot_state[0], robot_state[1])
    local_goal_dot.set_data(local_goal[0], local_goal[1])
    if best_traj:
        local_traj.set_data([p[0] for p in best_traj], [p[1] for p in best_traj])
    return [robot_dot, local_traj, local_goal_dot] + dynamic_circles

# 启动动画（100ms/帧）
ani = animation.FuncAnimation(fig, update, interval=100, blit=True, cache_frame_data=False)
plt.show()