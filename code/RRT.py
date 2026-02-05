import numpy as np  # 导入numpy库，用于数学计算
import matplotlib.pyplot as plt  # 导入matplotlib库，用于可视化

# 定义节点类，表示树中的每个节点
class Node:
    def __init__(self, x, y):
        self.x = x  # 节点的x坐标
        self.y = y  # 节点的y坐标
        self.parent = None  # 父节点，用于回溯路径

# RRT算法实现
def rrt(start, goal, obstacles, max_iter=1000, step_size=10):
    """
    RRT算法实现
    :param start: 起点坐标 (x, y)
    :param goal: 目标点坐标 (x, y)
    :param obstacles: 障碍物列表，每个障碍物表示为 (x, y, r)，其中 (x, y) 是中心，r 是半径
    :param max_iter: 最大迭代次数
    :param step_size: 每次扩展的步长
    :return: 找到的路径（列表形式，包含路径上的点）
    """
    tree = [Node(start[0], start[1])]  # 初始化树，根节点为起点
    for _ in range(max_iter):  # 迭代max_iter次
        # 随机生成一个点 q_rand
        q_rand = Node(np.random.uniform(0, 100), np.random.uniform(0, 100))
        
        # 在树中找到距离 q_rand 最近的节点 q_near
        q_near = tree[0]
        for node in tree:
            if np.hypot(node.x - q_rand.x, node.y - q_rand.y) < np.hypot(q_near.x - q_rand.x, q_near.y - q_rand.y):
                q_near = node
        
        # 计算 q_near 到 q_rand 的方向角
        theta = np.arctan2(q_rand.y - q_near.y, q_rand.x - q_near.x)
        
        # 从 q_near 向 q_rand 方向扩展一个新节点 q_new
        q_new = Node(q_near.x + step_size * np.cos(theta), q_near.y + step_size * np.sin(theta))
        q_new.parent = q_near  # 设置 q_new 的父节点为 q_near
        
        # 碰撞检测：检查 q_new 是否与任何障碍物碰撞
        collision = False
        for (ox, oy, r) in obstacles:
            if np.hypot(q_new.x - ox, q_new.y - oy) <= r:
                collision = True
                break
        
        # 如果没有碰撞，将 q_new 加入树中
        if not collision:
            tree.append(q_new)
            
            # 如果 q_new 接近目标点，则生成路径并返回
            if np.hypot(q_new.x - goal[0], q_new.y - goal[1]) < step_size:
                path = []
                current = q_new
                while current:  # 从 q_new 回溯到起点
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1]  # 返回反向路径（从起点到目标点）
    
    # 如果未找到路径，返回 None
    return None

# 测试
start = (10, 10)  # 起点坐标
goal = (90, 90)  # 目标点坐标
obstacles = [(50, 50, 15), (30, 70, 10)]  # 障碍物列表，每个障碍物表示为 (x, y, r)

# 调用 RRT 算法生成路径
path = rrt(start, goal, obstacles)

# 可视化
plt.figure()  # 创建画布
plt.scatter(start[0], start[1], c='green', s=100)  # 绘制起点
plt.scatter(goal[0], goal[1], c='red', s=100)  # 绘制目标点

# 绘制障碍物
for (x, y, r) in obstacles:
    circle = plt.Circle((x, y), r, color='gray')  # 创建圆形障碍物
    plt.gca().add_patch(circle)  # 将障碍物添加到画布

# 如果找到路径，绘制路径
if path:
    plt.plot(*zip(*path), 'b-')  # 将路径点连接成线

# 设置坐标轴范围
plt.xlim(0, 100)
plt.ylim(0, 100)

# 显示图像
plt.show()