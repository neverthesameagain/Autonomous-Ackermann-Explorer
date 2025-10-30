import heapq
import numpy as np

class Node:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
    def __lt__(self, other):
        return self.cost < other.cost

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(grid, start, goal):
    rows, cols = grid.shape
    open_list = []
    closed = set()
    start_node = Node(*start, cost=0)
    heapq.heappush(open_list, (0, start_node))

    moves = [(-1,0), (1,0), (0,-1), (0,1),
             (-1,-1), (-1,1), (1,-1), (1,1)]

    while open_list:
        _, current = heapq.heappop(open_list)
        if (current.x, current.y) in closed:
            continue
        closed.add((current.x, current.y))

        if (current.x, current.y) == goal:
            # reconstruct path
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        for dx, dy in moves:
            nx, ny = current.x + dx, current.y + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] == 0:
                cost = current.cost + heuristic((nx, ny), goal)
                node = Node(nx, ny, cost, current)
                heapq.heappush(open_list, (node.cost, node))
    return None
