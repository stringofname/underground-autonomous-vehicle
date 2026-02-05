import numpy as np
import matplotlib.pyplot as plt
import time
import random
from scipy.spatial import KDTree
# 定义节点类
class Node:
    def __init__(self, x, y):
        self.x = x  # x 坐标
        self.y = y  # y 坐标
        self.parent = None  # 父节点
        self.cost = 0  # 代价（路径长度）

# 路径平滑函数
def smooth_path1(path, obstacles, step_size=10):
    """
    简单路径平滑方法：尝试将路径中的每对相邻节点连接起来
    :param path: 原始路径
    :param obstacles: 障碍物列表，每个障碍物表示为 (x, y, r)
    :param step_size: 步长
    :return: 平滑后的路径
    """
    smooth_path = [path[0]]
    for i in range(1, len(path) - 1):
        # 检查能否直接连接当前节点和下一个节点
        if not any(np.hypot(path[i][0] - ox, path[i][1] - oy) <= r for (ox, oy, r) in obstacles):
            smooth_path.append(path[i])
    smooth_path.append(path[-1])
    return smooth_path

# 二次 Bézier 曲线平滑
def bezier_smooth(path, num_points=100):
    """
    使用二次 Bézier 曲线平滑路径
    :param path: 输入的路径，格式为 [(x1, y1), (x2, y2), ..., (xn, yn)]
    :param num_points: 每对点之间生成的平滑点数量
    :return: 平滑后的路径
    """
    smooth_path = [path[0]]
    for i in range(1, len(path) - 1, 2):
        P0, P1, P2 = path[i - 1], path[i], path[i + 1]

        # 使用二次 Bézier 曲线公式进行插值
        bezier_curve = []
        for t in np.linspace(0, 1, num_points):
            x = (1 - t)**2 * P0[0] + 2 * (1 - t) * t * P1[0] + t**2 * P2[0]
            y = (1 - t)**2 * P0[1] + 2 * (1 - t) * t * P1[1] + t**2 * P2[1]
            bezier_curve.append((x, y))
        
        smooth_path.extend(bezier_curve[:-1])  # 忽略最后一个点，因为它与下一个点相同
    smooth_path.append(path[-1])  # 添加路径的最后一个点
    return smooth_path

# RRT* 算法
class RRTStar:
    def __init__(self, start, goal, obstacles, max_iter=500, step_size=10, search_radius=20):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.tree = [Node(start[0], start[1])]
        self.kd_tree = KDTree([(node.x, node.y) for node in self.tree])
        self.obstacle_tree = KDTree([(ox, oy) for (ox, oy, r) in obstacles])

    def is_collision(self, node):
        for (ox, oy, r) in self.obstacles:
            if np.hypot(node.x - ox, node.y - oy) <= r:
                return True
        return False

    def find_nearest_node(self, q_rand):
        _, index = self.kd_tree.query((q_rand.x, q_rand.y))
        return self.tree[index]

    def find_near_nodes(self, q_new):
        near_indices = self.kd_tree.query_ball_point((q_new.x, q_new.y), self.search_radius)
        return [self.tree[i] for i in near_indices]
    #动态步长
    def dynamic_step_size(self, node):
        """
        根据当前节点到目标点的距离动态调整步长
        """
        dist_to_goal = np.hypot(node.x - self.goal[0], node.y - self.goal[1])
        min_step = 5  # 最小步长
        max_step = 30  # 最大步长
        step = max(min_step, min(max_step, dist_to_goal / 5))
        return step

    def rrt_star(self):
        plt.figure()
        plt.xlim(0, 100)
        plt.ylim(0, 100)

        # 画出起点、终点和障碍物
        plt.scatter(self.start[0], self.start[1], c='green', s=100, label="Start")
        plt.scatter(self.goal[0], self.goal[1], c='red', s=100, label="Goal")
        for (ox, oy, r) in self.obstacles:
            circle = plt.Circle((ox, oy), r, color='gray')
            plt.gca().add_patch(circle)
        start_time=time.perf_counter()
        for _ in range(self.max_iter):
            # 生成随机点,并以一定概率将终点作为随机点
            random_number = random.random()
            if random_number < 0.3:
                q_rand = Node(self.goal[0], self.goal[1])
            else:
                q_rand = Node(np.random.uniform(0, 100), np.random.uniform(0, 100))

            # 找到树中最近的节点
            q_near = self.find_nearest_node(q_rand)

            # 动态调整步长
            self.step_size = self.dynamic_step_size(q_near)
            # 计算方向
            theta = np.arctan2(q_rand.y - q_near.y, q_rand.x - q_near.x)

            # 生成新节点
            q_new = Node(q_near.x + self.step_size * np.cos(theta), q_near.y + self.step_size * np.sin(theta))
            q_new.parent = q_near
            q_new.cost = q_near.cost + self.step_size

            # 碰撞检测
            if self.is_collision(q_new):
                continue

            # 重新连接：寻找半径内的更优父节点
            near_nodes = self.find_near_nodes(q_new)
            if near_nodes:
                q_min = q_near
                min_cost = q_near.cost + np.hypot(q_near.x - q_new.x, q_near.y - q_new.y)
                for node in near_nodes:
                    new_cost = node.cost + np.hypot(node.x - q_new.x, node.y - q_new.y)
                    if new_cost < min_cost:
                        q_min = node
                        min_cost = new_cost
                q_new.parent = q_min
                q_new.cost = min_cost

            self.tree.append(q_new)
            self.kd_tree = KDTree([(node.x, node.y) for node in self.tree])

            # 可视化
            plt.title(f"Iteration {_+1}")
            plt.plot([q_new.parent.x, q_new.x], [q_new.parent.y, q_new.y], 'b-', alpha=0.6)
            plt.pause(0.2)  # 暂停 0.1 秒

            # 检查是否接近目标点
            if np.hypot(q_new.x - self.goal[0], q_new.y - self.goal[1]) < self.step_size:
                goal_node = Node(self.goal[0], self.goal[1])
                goal_node.parent = q_new
                self.tree.append(goal_node)

                # 生成路径
                path = []
                current = goal_node
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                # 简单平滑
                smooth_path_result = smooth_path1(path, self.obstacles)
                smooth_path_result = smooth_path1(path, self.obstacles)
                # 应用 Bézier 曲线平滑
                smooth_path_result = bezier_smooth(smooth_path_result)

                # 画出平滑后的路径
                plt.plot(*zip(*smooth_path_result), 'r-', linewidth=2)
                end_time=time.perf_counter()
                print(f"耗时：{end_time-start_time:.6f}秒")
                plt.pause(1)
                plt.show()
                return smooth_path_result[::-1]

        plt.show()
        return None

# 测试
start = (10, 10)
goal = (90, 90)
obstacles = [(50, 50, 15), (30, 70, 10)]

# 运行 RRT*
rrt_star = RRTStar(start, goal, obstacles)
path = rrt_star.rrt_star()


