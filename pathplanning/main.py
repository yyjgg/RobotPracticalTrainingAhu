import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import a_star
import dwa


plt.rcParams['font.sans-serif'] = ['SimHei'] 
plt.rcParams['axes.unicode_minus'] = False  


# 仿真参数
show_animation = True
ROBOT_RADIUS = 1.0
GRID_SIZE = 2.0

# 重规划常量
STUCK_LINEAR_VELOCITY_THRESHOLD = 0.05 
STUCK_COUNT_MAX = 5                     

# --- 辅助函数：障碍物检查 (保持不变) ---
def is_path_point_obstructed(x, y, combined_ob, robot_radius):
    if combined_ob.size == 0:
        return False
    ox = combined_ob[:, 0]
    oy = combined_ob[:, 1]
    distances = np.hypot(ox - x, oy - y)
    return np.min(distances) < robot_radius

# --- 地图设计函数 (保持不变) ---
def _get_common_borders():
    ox, oy = [], []
    for i in range(60): ox.append(i); oy.append(0.0)
    for i in range(60): ox.append(60.0); oy.append(i)
    for i in range(61): ox.append(i); oy.append(60.0)
    for i in range(61): ox.append(0.0); oy.append(i)
    return ox, oy

def design_map_simple():
    static_ox, static_oy = _get_common_borders()
    for i in range(40): static_ox.append(20.0); static_oy.append(i)
    for i in range(40): static_ox.append(40.0); static_oy.append(60.0 - i)
    sx, sy = 10.0, 10.0
    gx, gy = 50.0, 50.0
    print("已加载：简单迷宫")
    return static_ox, static_oy, sx, sy, gx, gy

def design_map_complex():
    static_ox, static_oy = _get_common_borders()
    for i in range(15, 55): static_ox.append(20.0); static_oy.append(i)
    for i in range(5, 45): static_ox.append(40.0); static_oy.append(i)
    for i in range(15, 30): static_ox.append(i); static_oy.append(15.0)
    for i in range(35, 55): static_ox.append(i); static_oy.append(30.0)
    for i in range(5, 55, 5): static_ox.append(i); static_oy.append(50.0)
    for i in range(42, 58, 2):
        for j in range(2, 25, 2):
            static_ox.append(i)
            static_oy.append(j)
    for i in range(45, 55): static_ox.append(45.0); static_oy.append(i)
    for i in range(45, 55): static_ox.append(55.0); static_oy.append(i)
    sx, sy = 5.0, 5.0    
    gx, gy = 50.0, 55.0  
    print("已加载：复杂迷宫")
    return static_ox, static_oy, sx, sy, gx, gy

def design_map_dynamic_complex():
    static_ox, static_oy, sx, sy, gx, gy = design_map_complex()
    print("已加载：动态复杂迷宫 (等待 A* 路径计算完毕后添加动态障碍)")
    return static_ox, static_oy, sx, sy, gx, gy

# --- 辅助函数：将动态障碍放置在 A* 路径上 (保持不变) ---
def place_dynamic_obs_on_path(rx, ry):
    dynamic_ox, dynamic_oy = [], []
    path_length = len(rx)
    
    if path_length < 100:
        print("警告：路径点过少，动态障碍放置可能不理想。")
        
    idx1 = path_length // 4
    if idx1 < path_length:
        mx, my = rx[idx1], ry[idx1]
        for i in np.arange(-3.0, 3.0, 0.5): # 放大障碍物
            for j in np.arange(-3.0, 3.0, 0.5):
                dynamic_ox.append(mx + i)
                dynamic_oy.append(my + j)
        print(f"   - 动态障碍 1 (大型) 放置于路径点 {idx1} 附近 ({mx:.1f}, {my:.1f})")

    idx2 = int(path_length * 0.6)
    if idx2 < path_length:
        mx, my = rx[idx2], ry[idx2]
        for i in np.arange(-0.5, 0.5, 0.5):
            for j in np.arange(-0.5, 0.5, 0.5):
                dynamic_ox.append(mx + i + 1.5)
                dynamic_oy.append(my + j)
        print(f"   - 动态障碍 2 (小型) 放置于路径点 {idx2} 附近 ({mx+1.5:.1f}, {my:.1f})")
        
    return dynamic_ox, dynamic_oy

# --- 主程序 ---
def main():
    print("----------------------------------")
    print("   Hybrid A* + DWA 路径规划仿真   ")
    print("----------------------------------")
    print("请选择地图类型:")
    print("1. 简单迷宫")
    print("2. 复杂迷宫")
    print("3. 动态复杂迷宫 (A* 路径上将出现红色障碍)")
    
    choice = input("请输入选项 (1, 2 或 3): ").strip()
    
    if choice == '2':
        static_ox, static_oy, sx, sy, gx, gy = design_map_complex()
        is_dynamic_path_obs = False
    elif choice == '3':
        static_ox, static_oy, sx, sy, gx, gy = design_map_dynamic_complex()
        is_dynamic_path_obs = True 
    else:
        static_ox, static_oy, sx, sy, gx, gy = design_map_simple()
        is_dynamic_path_obs = False

    # 1. A* 全局规划阶段
    ox_a_star = static_ox 
    oy_a_star = static_oy

    if show_animation:
        plt.figure(figsize=(8, 8)) 
        plt.plot(ox_a_star, oy_a_star, ".k", label="静态障碍物") 
        plt.plot(sx, sy, "og", label="起点") 
        plt.plot(gx, gy, "xb", label="终点") 
        # 这里先不显示 A* 路径，因为它可能被动态障碍覆盖，待动态障碍放置后再显示
        # 如果是动态迷宫，动态障碍物还没放置，所以这里也不显示其图例
        plt.legend(loc='upper right') # 提前显示起点终点静态障碍
        plt.grid(True)
        plt.axis("equal")
        plt.title("A* 全局路径规划 (基于静态地图)")
        
    print("1. 正在运行 A* 全局规划...")
    a_star_planner = a_star.AStarPlanner(ox_a_star, oy_a_star, GRID_SIZE, ROBOT_RADIUS)
    
    rx, ry = a_star_planner.planning(sx, sy, gx, gy, show_process=show_animation)
    
    rx = rx[::-1]
    ry = ry[::-1]
    
    if not rx:
        print("A* 无法找到路径！")
        if show_animation: plt.show()
        return

    # 2. 添加动态障碍物 (必须在 A* 运行后)
    dynamic_ox, dynamic_oy = [], []
    if is_dynamic_path_obs:
        dynamic_ox, dynamic_oy = place_dynamic_obs_on_path(rx, ry)
        # 此时 A* 规划已完成，绘制完整的 A* 路径和动态障碍物
        if show_animation:
            # 清除之前的图例，重新绘制以包含 A* 路径和动态障碍物
            plt.cla() 
            plt.plot(ox_a_star, oy_a_star, ".k", label="静态障碍物") 
            plt.plot(sx, sy, "og", label="起点") 
            plt.plot(gx, gy, "xb", label="终点") 
            plt.plot(dynamic_ox, dynamic_oy, "or", markersize=5, label="动态障碍物 (A*未知)") # 红色圆点
            plt.plot(rx, ry, "-r", linewidth=2, label="A*全局路径") # 红色线表示全局路径
            plt.legend(loc='upper right')
            plt.title("A* 全局路径及动态障碍物")
            plt.pause(0.5) # 暂停一下，让用户看清
            print("2. 动态障碍物已添加，DWA 将开始追踪并避障。")
    else:
        # 如果没有动态障碍物，也显示 A* 路径
        if show_animation:
            plt.plot(rx, ry, "-r", linewidth=2, label="A*全局路径")
            plt.legend(loc='upper right')
            plt.title("A* 全局路径已规划")
            plt.pause(0.5)
            print("2. A* 全局路径已规划，DWA 将开始追踪。")


    # 3. DWA 局部规划阶段
    print("3. 开始 DWA 轨迹追踪...")
    
    x = np.array([sx, sy, math.pi / 8.0, 0.0, 0.0])
    goal = np.array([gx, gy])
    
    static_ob_np = np.array([static_ox, static_oy]).T
    dynamic_ob_np = np.array([dynamic_ox, dynamic_oy]).T if dynamic_ox else np.empty((0, 2))
    
    config = dwa.Config()
    config.robot_radius = ROBOT_RADIUS

    trajectory = np.array(x)
    target_ind = 0
    stuck_counter = 0 
    
    while True:
        combined_ob = np.vstack((static_ob_np, dynamic_ob_np))
        
        dist_to_goal = math.hypot(x[0] - gx, x[1] - gy)
        min_dist = float("inf")
        for i in range(target_ind, len(rx)):
            d = math.hypot(rx[i] - x[0], ry[i] - x[1])
            if d < min_dist:
                min_dist = d
                target_ind = i
            else:
                break
        
        lookahead_offset = 2 
        local_goal_found = False
        while not local_goal_found:
            current_lookahead_idx = min(target_ind + lookahead_offset, len(rx) - 1)
            temp_goal = np.array([rx[current_lookahead_idx], ry[current_lookahead_idx]])

            if current_lookahead_idx == len(rx) - 1:
                local_goal = temp_goal
                local_goal_found = True
            elif not is_path_point_obstructed(temp_goal[0], temp_goal[1], combined_ob, config.robot_radius):
                local_goal = temp_goal
                local_goal_found = True
            else:
                lookahead_offset += 1
                if lookahead_offset > 50: 
                    local_goal = temp_goal
                    local_goal_found = True
                    # print("Warning: Lookahead search reached limit.") # 可以选择打印警告

        u, predicted_traj, all_candidates = dwa.dwa_control(x, config, local_goal, combined_ob)
        
        if abs(u[0]) < STUCK_LINEAR_VELOCITY_THRESHOLD and abs(u[1]) < 0.1: 
            stuck_counter += 1
        else:
            stuck_counter = 0

        if stuck_counter >= STUCK_COUNT_MAX:
            print("\n!!! --- DWA 陷入局部困境，触发全局路径重规划 --- !!!")
            
            # 将动态障碍物变为静态障碍物，并清空动态列表
            static_ox.extend(dynamic_ox)
            static_oy.extend(dynamic_oy)
            dynamic_ox, dynamic_oy = [], []
            
            print("    -> 重新计算 A* 路径...")
            a_star_planner_new = a_star.AStarPlanner(static_ox, static_oy, GRID_SIZE, ROBOT_RADIUS)
            rx_new, ry_new = a_star_planner_new.planning(x[0], x[1], gx, gy, show_process=False) 
            
            if not rx_new:
                print("    -> 警告：重规划失败！目标点被完全封锁。")
                break 
                
            rx, ry = rx_new[::-1], ry_new[::-1] 
            target_ind = 0                      
            stuck_counter = 0                   
            print("    -> 全局路径更新完成，继续 DWA 追踪。")
            
            static_ob_np = np.array([static_ox, static_oy]).T 
            dynamic_ob_np = np.empty((0, 2)) # 确保动态障碍物在DWA输入中也被清空

        x = dwa.motion(x, u, config.dt)
        trajectory = np.vstack((trajectory, x))

        if dist_to_goal <= config.robot_radius:
            print("到达目标点!")
            break

        if show_animation:
            plt.cla()
            # 1. 绘制静态障碍物 (黑色点)
            plt.plot(static_ox, static_oy, ".k", label="静态障碍物")
            # 2. 绘制动态障碍物 (红色圆点)
            if dynamic_ox:
                plt.plot(dynamic_ox, dynamic_oy, "or", markersize=5, label="动态障碍物")
                
            # 3. 绘制起点、终点和局部目标
            plt.plot(sx, sy, "og", label="起点") # 起点只需绘制一次，但为了图例可重复绘制
            plt.plot(gx, gy, "xb", label="终点") # 终点也同理
            plt.plot(local_goal[0], local_goal[1], "xg", markersize=8, label="局部目标") # 局部目标，加大标记

            # 4. 绘制 A* 全局路径
            plt.plot(rx, ry, "-r", linewidth=2, label="A*全局路径")
            
            # 5. 绘制 DWA 采样过程 (绿色半透明细线)
            for i in range(0, len(all_candidates), 5): 
                cand = all_candidates[i]
                plt.plot(cand[:, 0], cand[:, 1], "-", color=[0, 1, 0, 0.15]) 
            plt.plot([], [], "-", color=[0, 1, 0, 0.15], label="DWA采样路径") # 为采样路径添加一个图例条目

            # 6. 绘制 DWA 选定的最优预测轨迹 (蓝色粗线)
            plt.plot(predicted_traj[:, 0], predicted_traj[:, 1], "-b", linewidth=2, label="DWA最优轨迹")
            
            # 7. 绘制机器人历史轨迹 (黑色实线)
            plt.plot(trajectory[:, 0], trajectory[:, 1], "-k", label="机器人历史轨迹")
            # 8. 绘制当前机器人位置和朝向 (蓝色圆和线)
            draw_robot(x[0], x[1], x[2], config, label="机器人") # 给机器人添加图例

            plt.title("DWA 局部规划 (停滞计数: {})".format(stuck_counter))
            plt.axis("equal")
            plt.grid(True)
            plt.legend(loc='upper right', prop={'size': 9}) # 调整图例字体大小和位置
            plt.pause(0.001)

    print("Done")
    if show_animation:
        plt.show()

# draw_robot 函数需要修改以支持 label 参数
def draw_robot(x, y, yaw, config, label=None):
    circle = plt.Circle((x, y), config.robot_radius, color="b", fill=False)
    plt.gcf().gca().add_artist(circle)
    plt.plot([x, x + math.cos(yaw) * config.robot_radius], 
             [y, y + math.sin(yaw) * config.robot_radius], "-b", label=label) # 添加label参数

if __name__ == '__main__':
    main()