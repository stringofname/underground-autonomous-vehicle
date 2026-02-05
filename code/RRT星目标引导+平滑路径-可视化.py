import numpy as np
import matplotlib.pyplot as plt
import time
import random

# 定义节点类
class Node:
    def __init__(self, x, y):
        self.x = x  # x 坐标
        self.y = y  # y 坐标
        self.parent = None  # 父节点
        self.cost = 0  # 代价（路径长度）

# 路径平滑函数
def smooth_path(path, obstacles, step_size=10):
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

# RRT* 算法
def rrt_star(start, goal, obstacles, max_iter=500, step_size=10, search_radius=20):
    """
    RRT* 算法实现
    :param start: 起点坐标 (x, y)
    :param goal: 目标点坐标 (x, y)
    :param obstacles: 障碍物列表，每个障碍物表示为 (x, y, r)
    :param max_iter: 最大迭代次数
    :param step_size: 每次扩展的步长
    :param search_radius: 重新连接的搜索半径
    :return: 找到的路径
    """
    tree = [Node(start[0], start[1])]  # 初始化树

    plt.figure()
    plt.xlim(0, 100)
    plt.ylim(0, 100)

    # 画出起点、终点和障碍物
    plt.scatter(start[0], start[1], c='green', s=100, label="Start")
    plt.scatter(goal[0], goal[1], c='red', s=100, label="Goal")
    for (ox, oy, r) in obstacles:
        circle = plt.Circle((ox, oy), r, color='gray')
        plt.gca().add_patch(circle)

    for _ in range(max_iter):
        # 生成随机点,并以一定概率将终点作为随机点
        random_number = random.random()
        if random_number < 0.1:
            q_rand=Node(goal[0],goal[1])
        else:
            q_rand = Node(np.random.uniform(0, 100), np.random.uniform(0, 100))

        # 找到树中最近的节点
        q_near = min(tree, key=lambda node: np.hypot(node.x - q_rand.x, node.y - q_rand.y))

        # 计算方向
        theta = np.arctan2(q_rand.y - q_near.y, q_rand.x - q_near.x)

        # 生成新节点
        q_new = Node(q_near.x + step_size * np.cos(theta), q_near.y + step_size * np.sin(theta))
        q_new.parent = q_near
        q_new.cost = q_near.cost + step_size

        # 碰撞检测
        if any(np.hypot(q_new.x - ox, q_new.y - oy) <= r for (ox, oy, r) in obstacles):
            continue

        # 重新连接：寻找半径内的更优父节点
        near_nodes = [node for node in tree if np.hypot(node.x - q_new.x, node.y - q_new.y) < search_radius]
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

        tree.append(q_new)

        # 可视化
        plt.plot([q_new.parent.x, q_new.x], [q_new.parent.y, q_new.y], 'b-', alpha=0.6)
        plt.pause(0.5)  # 暂停 0.1 秒

        # 检查是否接近目标点
        if np.hypot(q_new.x - goal[0], q_new.y - goal[1]) < step_size:
            goal_node = Node(goal[0], goal[1])
            goal_node.parent = q_new
            tree.append(goal_node)

            # 生成路径
            path = []
            current = goal_node
            while current:
                path.append((current.x, current.y))
                current = current.parent

            # 应用路径平滑
            smooth_path_result = smooth_path(path, obstacles)

            # 画出平滑后的路径
            plt.plot(*zip(*smooth_path_result), 'r-', linewidth=2)
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
path = rrt_star(start, goal, obstacles)
