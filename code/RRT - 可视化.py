import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def rrt(start, goal, obstacles, max_iter=1000, step_size=10):
    tree = [Node(start[0], start[1])]
    plt.figure()
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.scatter(start[0], start[1], c='green', s=100)
    plt.scatter(goal[0], goal[1], c='red', s=100)
    
    for (x, y, r) in obstacles:
        circle = plt.Circle((x, y), r, color='gray')
        plt.gca().add_patch(circle)
    
    for _ in range(max_iter):
        q_rand = Node(np.random.uniform(0, 100), np.random.uniform(0, 100))
        
        q_near = min(tree, key=lambda node: np.hypot(node.x - q_rand.x, node.y - q_rand.y))
        
        theta = np.arctan2(q_rand.y - q_near.y, q_rand.x - q_near.x)
        q_new = Node(q_near.x + step_size * np.cos(theta), q_near.y + step_size * np.sin(theta))
        q_new.parent = q_near
        
        collision = any(np.hypot(q_new.x - ox, q_new.y - oy) <= r for (ox, oy, r) in obstacles)
        
        if not collision:
            tree.append(q_new)
            plt.plot([q_near.x, q_new.x], [q_near.y, q_new.y], 'b-')
            plt.title(f"Iteration {_+1}")
            plt.pause(0.1)
            
            if np.hypot(q_new.x - goal[0], q_new.y - goal[1]) < step_size:
                path = []
                current = q_new
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                plt.plot(*zip(*path[::-1]), 'r-', linewidth=2)
                plt.show()
                return path[::-1]
    
    plt.show()
    return None

start = (10, 10)
goal = (90, 90)
obstacles = [(50, 50, 0), (30, 70, 0)]
path = rrt(start, goal, obstacles)
