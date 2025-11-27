import math
import numpy as np

class Config:
    def __init__(self):
        self.max_speed = 1.0
        self.min_speed = -0.5
        self.max_yaw_rate = 100.0 * math.pi / 180.0
        self.max_accel = 0.2
        self.max_delta_yaw_rate = 100.0 * math.pi / 180.0
        self.v_resolution = 0.01
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0
        self.dt = 0.1
        self.predict_time = 3.0
        
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        
        self.robot_stuck_flag_cons = 0.001
        self.robot_type = 1
        self.robot_radius = 1.0

def motion(x, u, dt):
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    return x

def calc_dynamic_window(x, config):
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw

def predict_trajectory(x_init, v, y, config):
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt
    return trajectory

def calc_to_goal_cost(trajectory, goal):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
    return cost

def calc_obstacle_cost(trajectory, ob, config):
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)
    
    if config.robot_type == 1:
        pass

    min_r = np.min(r)
    if min_r <= config.robot_radius:
        return float("Inf")
    return 1.0 / min_r

def dwa_control(x, config, goal, ob):
    dw = calc_dynamic_window(x, config)
    
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])
    
    # 新增：用于存储所有采样轨迹
    all_candidates = []

    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
            trajectory = predict_trajectory(x, v, y, config)
            
            # 将当前预测的轨迹加入候选列表
            all_candidates.append(trajectory)

            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            
            final_cost = to_goal_cost + speed_cost + ob_cost

            if final_cost < min_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
    
    # 返回三个值：最优控制、最优轨迹、所有候选轨迹
    return best_u, best_trajectory, all_candidates