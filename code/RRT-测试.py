import numpy as np
import matplotlib.pyplot as plt
import time
import random
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0
def rrt(start, goal, obstacles, max_iter=1000, step_size=10):
    tree = [Node(start[0], start[1])]
    
    for _ in range(max_iter):
        
        # 生成随机点,并以一定概率将终点作为随机点
        random_number = random.random()
        if random_number < 0.3:
            q_rand = Node(goal[0], goal[1])
        else:
            q_rand = Node(np.random.uniform(0, 100), np.random.uniform(0, 100))
        
        q_near = min(tree, key=lambda node: np.hypot(node.x - q_rand.x, node.y - q_rand.y))
        
        theta = np.arctan2(q_rand.y - q_near.y, q_rand.x - q_near.x)
        q_new = Node(q_near.x + step_size * np.cos(theta), q_near.y + step_size * np.sin(theta))
        q_new.parent = q_near
        q_new.cost = q_near.cost + step_size
        collision = any(np.hypot(q_new.x - ox, q_new.y - oy) <= r for (ox, oy, r) in obstacles)
        
        if not collision:
            tree.append(q_new)
            


        tree.append(q_new)

        tree.append(q_new)
        if np.hypot(q_new.x - goal[0], q_new.y - goal[1]) < step_size:
                path = []
                current = q_new
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return _
    
    return None

def average_steps(start, goal, obstacles, step_size, trials=50):
    total_steps = 0
    for _ in range(trials):
        steps= rrt(start, goal, obstacles, step_size=step_size)
        if steps is not None:
            total_steps += steps
    return total_steps / trials

# Parameters
start = (10, 10)
goal = (90, 90)
obstacles = [(50, 50, 0), (30, 70, 0)]

# Test different step sizes and calculate average steps
average_results = []

for i in range(20,100,10):
    avg_steps = average_steps(start, (i,i), obstacles, 10)
    average_results.append((i, avg_steps))
    print(f"goal: {i}, Average Steps: {avg_steps}")
    
# Optionally, plot the results
step_sizes, avg_steps = zip(*average_results)
plt.plot(step_sizes, avg_steps, marker='o')
plt.xlabel('Goal')
plt.ylabel('Average Steps')
plt.title('Average Steps vs Goal')
plt.show()
