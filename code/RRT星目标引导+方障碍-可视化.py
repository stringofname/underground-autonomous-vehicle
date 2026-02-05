import numpy as np
import matplotlib.pyplot as plt
import random
import time
from matplotlib.patches import Rectangle

# 定义环境类
class Environment:
    def __init__(self, x_max, y_max, obstacles=None):
        self.x_max = x_max
        self.y_max = y_max
        self.obstacles = obstacles if obstacles is not None else []

    def plot(self):
        plt.xlim(0, self.x_max)
        plt.ylim(0, self.y_max)
        for obs in self.obstacles:
            plt.gca().add_patch(Rectangle((obs[0], obs[1]), obs[2], obs[3], color="gray"))
    
    def is_collision_free(self, p1, p2):
        for obs in self.obstacles:
            # 简化为矩形障碍物检查
            x1, y1, w, h = obs
            if max(p1[0], p2[0]) > x1 and min(p1[0], p2[0]) < x1 + w and \
               max(p1[1], p2[1]) > y1 and min(p1[1], p2[1]) < y1 + h:
                return False
        return True

# 目标引导采样（Goal Biasing）
def biased_sample(goal, env, bias_rate=0.2):
    if random.random() < bias_rate:
        return goal  # 偏向目标
    else:
        return (random.uniform(0, env.x_max), random.uniform(0, env.y_max))

# 路径平滑（Bézier平滑）
def smooth_path(path, smooth_iterations=5):
    for _ in range(smooth_iterations):
        for i in range(1, len(path) - 1):
            path[i] = ((path[i-1][0] + path[i+1][0]) / 2, (path[i-1][1] + path[i+1][1]) / 2)
    return path

# RRT*算法
def rrt_star(start, goal, env, max_iter=1000, step_size=5, goal_bias=0.1, smooth_iterations=5):
    nodes = [start]
    edges = []
    for i in range(max_iter):
        # 1. 目标引导采样
        sample = biased_sample(goal, env, goal_bias)
        
        # 2. 找到最近节点
        nearest_node = min(nodes, key=lambda node: np.linalg.norm(np.array(node) - np.array(sample)))
        
        # 3. 扩展树
        theta = np.arctan2(sample[1] - nearest_node[1], sample[0] - nearest_node[0])
        new_node = (nearest_node[0] + step_size * np.cos(theta), nearest_node[1] + step_size * np.sin(theta))
        
        # 4. 检查是否碰撞
        if env.is_collision_free(nearest_node, new_node):
            nodes.append(new_node)
            edges.append((nearest_node, new_node))
            
            # 5. 可视化当前树
            plt.clf()
            env.plot()
            for edge in edges:
                plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color="blue", lw=1)
            for node in nodes:
                plt.plot(node[0], node[1], "go")
            plt.plot(start[0], start[1], "ro")
            plt.plot(goal[0], goal[1], "bo")
            plt.title(f"Iteration {i+1}")
            plt.pause(0.1)
            time.sleep(0.5)
    
    # 6. 路径平滑
    path = [goal]
    current_node = goal
    while current_node != start:
        for edge in edges:
            if edge[1] == current_node:
                current_node = edge[0]
                path.append(current_node)
                break
    path = path[::-1]
    smooth_path(path, smooth_iterations)

    # 7. 可视化最终路径
    plt.clf()
    env.plot()
    for edge in edges:
        plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color="blue", lw=1)
    for node in nodes:
        plt.plot(node[0], node[1], "go")
    for p in path:
        plt.plot(p[0], p[1], "ro")
    plt.plot(start[0], start[1], "ro")
    plt.plot(goal[0], goal[1], "bo")
    plt.title("Final Path")
    plt.show()

# 示例环境设置
env = Environment(100, 100, obstacles=[(30, 10, 20, 20), (60, 50, 20, 10)])

# 起点和终点
start = (10, 10)
goal = (90, 90)

# 运行RRT*算法
plt.figure(figsize=(8, 8))
rrt_star(start, goal, env)
